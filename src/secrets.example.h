#pragma once
// Copy to src/secrets.h (gitignored) and fill in. Only used when AFEC_LOG_SINK includes MQTT (2 or 3).
#define AFEC_WIFI_SSID "my-ssid"
#define AFEC_WIFI_PASS "my-password"
#define AFEC_MQTT_URI  "mqtt://192.168.1.10:1883"   // the broker on the bench network
// Optional:
// #define AFEC_MQTT_USER "afec"
// #define AFEC_MQTT_PASS "secret"
// #define AFEC_MQTT_RING_BYTES 32768   // log ring between the protocol (core 1) and the publisher (core 0)
// #define AFEC_MQTT_QOS 1
