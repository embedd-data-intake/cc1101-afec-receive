#include <Arduino.h>
#include <SPI.h>

#include <cc1101.h>
#include <hal/cc1101_hal_arduino.h>
#include <afec_node.h>

#include "afec_platform_arduino.h"
#include "config.h"

CC1101HalArduino hal(SPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN, 4000000, PIN_GDO0);
CC1101 radio;

afec::Node node;

void debugPrint(const char *format, ...) {
    char buf[192];
    va_list ap;
    va_start(ap, format);
    int n = vsnprintf(buf, sizeof buf, format, ap);
    va_end(ap);
    Serial.write((const uint8_t*)buf, n < (int)sizeof buf ? n : sizeof buf - 1);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    CC1101::Config cfg;
    cfg.freqMHz    = RF_FREQ_MHZ;
    cfg.syncWord   = RF_SYNC_WORD;
    cfg.txPowerDbm = RF_TX_DBM;
    cfg.whitening  = true;
    cfg.hwCrc      = false;

    afec::NodeConfig nc; nc.id = NODE_ID; nc.bitrateBps = 4800;

    node.printDebug = debugPrint;

    if (!radio.begin(hal, cfg)) {
        Serial.println("CC1101 not found - check wiring/power");
        while (true) delay(1000);
    }
    if (USE_IRQ) {
        if (!radio.enableInterrupt()) Serial.println("IRQ setup failed, falling back to polling");
    }
    Serial.printf("CC1101 ready, VERSION=0x%02X, f=%.3f MHz\n", radio.version(), RF_FREQ_MHZ);

    node.begin(radio, nc, afec::arduinoPlatform());

    node.onData([](void*, uint8_t src, const uint8_t* p, uint8_t n, uint8_t lhs){
        for (int i = 0; i < n; i++) Serial.printf("%02X ", p[i]);
        Serial.print(" | ");
        for (int i = 0; i < n; i++) Serial.print((p[i] >= 32 && p[i] < 127) ? (char)p[i] : '.');
        Serial.println();
    });
}

void loop() {
    node.poll();
}
