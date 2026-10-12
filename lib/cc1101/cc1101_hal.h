#pragma once
// Hardware abstraction layer for the CC1101 driver.
// The driver core (cc1101.h/.cpp) only talks to this interface, it includes no
// framework headers. To port to another platform (ESP-IDF, STM32 HAL, Zephyr,
// Linux spidev, ...) subclass this and implement the pure virtuals.
#include <stdint.h>

// Marks code that runs in interrupt context. On ESP32 it expands to IRAM_ATTR.
// Override with -DCC1101_ISR_ATTR=... for other platforms (e.g. RAM-function attribute on STM32).
#ifndef CC1101_ISR_ATTR
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
#include "esp_attr.h"
#define CC1101_ISR_ATTR IRAM_ATTR
#else
#define CC1101_ISR_ATTR
#endif
#endif

typedef void (*CC1101IsrFn)(void* arg);

class CC1101Hal {
    public:
    virtual ~CC1101Hal() {}

    // Called once from CC1101::begin(). Configure pins / start the SPI peripheral here.
    // Return false to abort begin().
    virtual bool init() { return true; }

    // ---- SPI: mode 0 (CPOL=0, CPHA=0), MSB first, <= 10 MHz (4 MHz is a safe default) ----
    // spiBegin()/spiEnd() bracket one chip-select cycle (acquire bus, apply settings).
    virtual void    spiBegin() = 0;
    virtual void    spiEnd() = 0;
    virtual uint8_t spiTransfer(uint8_t out) = 0;     // full duplex, 1 byte

    // ---- GPIO ----
    virtual void csWrite(bool high) = 0;              // CSN pin: false = selected
    // Read the MISO line as a plain GPIO. The CC1101 pulls it low when its crystal is
    // ready, and the driver waits for that after CSN goes low.
    // Optional: if you can't read MISO as GPIO leave the default (returns false = "ready"),
    // the chip is always ready when it's not in sleep, which this driver never uses.
    virtual bool misoRead() { return false; }

    // ---- Interrupt on GDO0 (optional) ----
    // The driver configures GDO0 so that it FALLS when a packet has been received or sent.
    // irqAttach(): call fn(arg) from interrupt context on every FALLING edge of the GDO0 pin.
    // Return false if the HAL has no GDO0 pin / no interrupt support: the driver then
    // stays in polling mode. fn is tiny and ISR-safe, don't block in it.
    virtual bool irqAttach(CC1101IsrFn fn, void* arg) { (void)fn; (void)arg; return false; }
    virtual void irqDetach() {}

    // ---- Time ----
    virtual uint32_t nowMs() = 0;                     // monotonic milliseconds (wraparound ok)
    virtual uint32_t nowUs() = 0;                     // monotonic microseconds (wraparound ok)
    virtual void     delayMs(uint32_t ms) = 0;        // may yield to an RTOS
    virtual void     delayUs(uint32_t us) = 0;
};
