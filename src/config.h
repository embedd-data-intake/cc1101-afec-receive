// CC1101 (433 MHz) <-> ESP32, Arduino framework.
//
//   CC1101 VCC  -> 3V3
//   CC1101 GND  -> GND
//   CC1101 SCK  -> GPIO19   (PIN_SCK)
//   CC1101 SO   -> GPIO18   (PIN_MISO)   (a.k.a. MISO / GDO1)
//   CC1101 SI   -> GPIO23   (PIN_MOSI)
//   CC1101 CSN  -> GPIO27   (PIN_CSN)
//   GDO0        -> GPIO26   (PIN_GDO0)   only for interrupt mode (USE_IRQ)
//   GDO2        -> GPIO16   (PIN_GDO2)   (not used)

#pragma once

static constexpr int8_t PIN_SCK  = 19;
static constexpr int8_t PIN_MISO = 18;
static constexpr int8_t PIN_MOSI = 23;
static constexpr int8_t PIN_CSN  = 27;
static constexpr int8_t PIN_GDO0 = 26;
static constexpr int8_t PIN_GDO2 = 16;

static constexpr int8_t NODE_ID = 1;

static constexpr double   RF_FREQ_MHZ   = 433.92;
static constexpr uint16_t RF_SYNC_WORD  = 0x6B1D;
static constexpr int8_t   RF_TX_DBM     = 0;

static constexpr bool     USE_IRQ        = true;
static constexpr uint32_t TX_INTERVAL_MS = 2000;

#define DEBUG
