// CC1101 (433 MHz) <-> ESP32, Arduino framework.
//
// Wiring (CC1101 module is 3.3 V ONLY - do not feed 5 V):
//   CC1101 VCC  -> 3V3
//   CC1101 GND  -> GND
//   CC1101 SCK  -> GPIO18   (PIN_SCK)
//   CC1101 SO   -> GPIO19   (PIN_MISO)   (a.k.a. MISO / GDO1)
//   CC1101 SI   -> GPIO23   (PIN_MOSI)
//   CC1101 CSN  -> GPIO27   (PIN_CSN)
//   GDO0        -> GPIO26   (PIN_GDO0)   only for interrupt mode (USE_IRQ)
//   GDO2        -> GPIO16   (PIN_GDO2)   (not used)
//   + attach a 433 MHz antenna (~17.3 cm wire for a quarter wave) before transmitting!

#include <Arduino.h>
#include <SPI.h>
#include "cc1101.h"
#include "hal/cc1101_hal_arduino.h"   // swap for your own HAL on other frameworks

// ===================== USER CONFIG =====================
// Pins (change for your board; on S3/C3 pick free GPIOs)
static constexpr int8_t PIN_SCK  = 18;
static constexpr int8_t PIN_MISO = 19;
static constexpr int8_t PIN_MOSI = 23;
static constexpr int8_t PIN_CSN  = 27;
static constexpr int8_t PIN_GDO0 = 26;     // CC1101 GDO0 -> GPIO4 (only needed for USE_IRQ)
static constexpr int8_t PIN_GDO2 = 16;

// Radio parameters
static constexpr double   RF_FREQ_MHZ   = 433.92;   // carrier frequency
static constexpr uint16_t RF_SYNC_WORD  = 0x6B1D;   // your "network ID" - must match on all nodes
static constexpr int8_t   RF_TX_DBM     = 0;        // -30,-20,-15,-10,0,5,7,10

static constexpr bool     USE_IRQ        = true;    // true = interrupt on GDO0, false = pure polling
static constexpr uint32_t TX_INTERVAL_MS = 2000;

// HAL = the only platform-specific part. Optional 6th arg: SPI clock in Hz (default 4 MHz).
CC1101HalArduino hal(SPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN, 4000000, PIN_GDO0);
CC1101 radio;
static uint32_t counter = 0;
static uint32_t lastTx  = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    CC1101::Config cfg;
    cfg.freqMHz    = RF_FREQ_MHZ;
    cfg.syncWord   = RF_SYNC_WORD;
    cfg.txPowerDbm = RF_TX_DBM;
    cfg.whitening  = true;     // keep identical on both ends
    cfg.hwCrc      = false;    // true = radio drops corrupted packets

    if (!radio.begin(hal, cfg)) {
        Serial.println("CC1101 not found - check wiring/power");
        while (true) delay(1000);
    }
    if (USE_IRQ) {
        // With IRQ on, radio.receive() costs nothing (no SPI) until GDO0 flags a finished packet.
        // Optional: enableInterrupt(myIsrCb, arg) also calls your ISR-safe callback on every event.
        if (!radio.enableInterrupt()) Serial.println("IRQ setup failed, falling back to polling");
    }
    Serial.printf("CC1101 ready, VERSION=0x%02X, f=%.3f MHz\n", radio.version(), RF_FREQ_MHZ);
}

void loop() {
    uint8_t buf[CC1101::MAX_PAYLOAD];
    CC1101::RxInfo info;
    int16_t n = radio.receive(buf, sizeof(buf), &info);
    if (n > 0) {
        Serial.printf("RX %d bytes, RSSI=%d dBm, LQI=%u: ", n, info.rssiDbm, info.lqi);
        for (int i = 0; i < n; i++) Serial.printf("%02X ", buf[i]);
        Serial.print(" | ");
        for (int i = 0; i < n; i++) Serial.print((buf[i] >= 32 && buf[i] < 127) ? (char)buf[i] : '.');
        Serial.println();
    }

    delay(1);
}
