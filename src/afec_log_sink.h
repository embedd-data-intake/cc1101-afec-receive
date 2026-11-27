#pragma once
// Log sink: where the JSON log lines (library events + firmware events) go. Firmware-only; the
// library just calls node.printDebug. Keep this file identical in the sender and the receiver.
//
// Chosen at build time with -D AFEC_LOG_SINK=<bitmask>:
//   0  off     no log output at all; node.printDebug stays null, so the library skips formatting
//   1  serial  JSON lines on the UART at 115200 (default; what campaign.py reads)
//   2  mqtt    one MQTT message per line, QoS 1, topic afec/<run_id>/<tx|rx>/log
//   3  both    serial and mqtt
//
// MQTT path: the protocol runs in loop() on core 1. write() copies the line into a FreeRTOS ring
// buffer without blocking; a publisher task pinned to core 0 (with the WiFi driver and lwIP) takes
// lines out and publishes them while the run is going. If the ring is full the line is dropped and
// counted, never waited for; a `log_drop` event reports the count once space frees up. `t` is
// millis() at the time of logging, so publish delay does not move timestamps.
//
// <run_id> is the `run=` of the last `mark` ("boot" until then). Commands still arrive over serial.
// WiFi and broker settings come from src/secrets.h (gitignored; copy secrets.example.h).
#include <Arduino.h>

#ifndef AFEC_LOG_SINK
#define AFEC_LOG_SINK 1
#endif
#define AFEC_SINK_SERIAL 1
#define AFEC_SINK_MQTT   2

#if AFEC_LOG_SINK & AFEC_SINK_MQTT
#include <WiFi.h>
#include <mqtt_client.h>
#include <freertos/ringbuf.h>
#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "AFEC_LOG_SINK includes MQTT: copy src/secrets.example.h to src/secrets.h and fill it in"
#endif
#ifndef AFEC_MQTT_USER
#define AFEC_MQTT_USER nullptr
#endif
#ifndef AFEC_MQTT_PASS
#define AFEC_MQTT_PASS nullptr
#endif
#ifndef AFEC_MQTT_RING_BYTES
#define AFEC_MQTT_RING_BYTES 32768      // ~25 s of DBG logging at the campaign's rate
#endif
#ifndef AFEC_MQTT_QOS
#define AFEC_MQTT_QOS 1
#endif
#endif

namespace logsink {

constexpr bool enabled() { return AFEC_LOG_SINK != 0; }

inline const char* name() {
    switch (AFEC_LOG_SINK) {
        case 0:  return "off";
        case 1:  return "serial";
        case 2:  return "mqtt";
        default: return "serial+mqtt";
    }
}

#if AFEC_LOG_SINK & AFEC_SINK_MQTT
struct Mqtt {
    RingbufHandle_t ring = nullptr;
    esp_mqtt_client_handle_t cli = nullptr;
    char run[40] = "boot";              // producer side (core 1) only: copied into every item
    const char* role = "node";          // "tx" | "rx"
    uint8_t node = 0;
    char clientId[24];
    volatile bool connected = false;
    // Written by one side each, read by both; 32-bit aligned stores are atomic on the ESP32.
    volatile uint32_t enq = 0, dropped = 0;           // producer
    volatile uint32_t pub = 0, retries = 0, conns = 0; // consumer
};
inline Mqtt& mq() { static Mqtt m; return m; }

inline void mqttEvent(void*, esp_event_base_t, int32_t id, void*) {
    if (id == MQTT_EVENT_CONNECTED)    { mq().connected = true; mq().conns++; }
    if (id == MQTT_EVENT_DISCONNECTED) { mq().connected = false; }
}

// Item layout in the ring: "<run_id>\0<line without the trailing newline>".
inline void publisher(void*) {
    Mqtt& m = mq();
    uint32_t reportedDrops = 0;
    char topic[64], lastRun[40] = "boot";   // run of the last published line, for log_drop
    for (;;) {
        if (!m.connected) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
        size_t n = 0;
        char* it = (char*)xRingbufferReceive(m.ring, &n, pdMS_TO_TICKS(200));
        if (it) {
            const size_t rl = strnlen(it, n);
            snprintf(topic, sizeof topic, "afec/%s/%s/log", it, m.role);
            // Hold the item until the broker client takes it; meanwhile new lines queue or drop.
            while (esp_mqtt_client_publish(m.cli, topic, it + rl + 1, (int)(n - rl - 1), AFEC_MQTT_QOS, 0) < 0) {
                m.retries++;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            m.pub++;
            strlcpy(lastRun, it, sizeof lastRun);
            vRingbufferReturnItem(m.ring, it);
        }
        const uint32_t d = m.dropped;
        if (d != reportedDrops && xRingbufferGetCurFreeSize(m.ring) > AFEC_MQTT_RING_BYTES / 2) {
            char line[128];
            const int k = snprintf(line, sizeof line,
                                   "{\"t\":%lu,\"node\":%u,\"lv\":\"W\",\"ev\":\"log_drop\",\"dropped\":%lu,\"total\":%lu}",
                                   (unsigned long)millis(), (unsigned)m.node, (unsigned long)(d - reportedDrops),
                                   (unsigned long)d);
            snprintf(topic, sizeof topic, "afec/%s/%s/log", lastRun, m.role);
            if (esp_mqtt_client_publish(m.cli, topic, line, k, AFEC_MQTT_QOS, 0) >= 0) reportedDrops = d;
        }
    }
}

inline void mqttBegin(uint8_t nodeId, const char* role) {
    Mqtt& m = mq();
    m.node = nodeId; m.role = role;
    snprintf(m.clientId, sizeof m.clientId, "afec-node%u", (unsigned)nodeId);
    m.ring = xRingbufferCreate(AFEC_MQTT_RING_BYTES, RINGBUF_TYPE_NOSPLIT);

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(AFEC_WIFI_SSID, AFEC_WIFI_PASS);       // async; esp-mqtt retries until the network is up

    esp_mqtt_client_config_t c = {};
    c.uri = AFEC_MQTT_URI;
    c.client_id = m.clientId;
    c.username = AFEC_MQTT_USER;
    c.password = AFEC_MQTT_PASS;
    c.reconnect_timeout_ms = 2000;
    m.cli = esp_mqtt_client_init(&c);
    esp_mqtt_client_register_event(m.cli, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqttEvent, nullptr);
    esp_mqtt_client_start(m.cli);
    xTaskCreatePinnedToCore(publisher, "afec_log", 4096, nullptr, 1, nullptr, 0);
}

// Never blocks: drops (and counts) the line if the ring is full or not created.
inline void mqttWrite(const char* line, size_t n) {
    Mqtt& m = mq();
    while (n && (line[n - 1] == '\n' || line[n - 1] == '\r')) n--;
    if (!n) return;
    const size_t rl = strlen(m.run);
    void* slot = nullptr;
    if (!m.ring || xRingbufferSendAcquire(m.ring, &slot, rl + 1 + n, 0) != pdTRUE) { m.dropped++; return; }
    memcpy(slot, m.run, rl + 1);
    memcpy((char*)slot + rl + 1, line, n);
    xRingbufferSendComplete(m.ring, slot);
    m.enq++;
}
#endif

// role: "tx" (sender) or "rx" (receiver); it names the MQTT topic. Call once in setup(), after Serial.begin().
inline void begin(uint8_t nodeId, const char* role) {
    (void)nodeId; (void)role;
#if AFEC_LOG_SINK & AFEC_SINK_MQTT
    mqttBegin(nodeId, role);
#endif
}

// One complete log line, including its trailing '\n'.
inline void write(const char* line, size_t n) {
    (void)line; (void)n;
#if AFEC_LOG_SINK & AFEC_SINK_SERIAL
    Serial.write((const uint8_t*)line, n);
#endif
#if AFEC_LOG_SINK & AFEC_SINK_MQTT
    mqttWrite(line, n);
#endif
}

// From `mark run=<id>`: lines logged from now on go to afec/<id>/<role>/log.
// MQTT wildcards and separators are replaced; an empty id keeps the current one.
inline void setRun(const char* id) {
    (void)id;
#if AFEC_LOG_SINK & AFEC_SINK_MQTT
    if (!id || !*id) return;
    char* r = mq().run;
    size_t i = 0;
    for (; id[i] && i + 1 < sizeof mq().run; i++) r[i] = (id[i] == '/' || id[i] == '+' || id[i] == '#') ? '_' : id[i];
    r[i] = 0;
#endif
}

} // namespace logsink
