#pragma once
// Arduino-framework HAL for the CC1101 driver (ESP32 and other Arduino cores).
// Header-only: it is compiled only if you #include it, so non-Arduino builds are unaffected.
#include <Arduino.h>
#include <SPI.h>
#include "../cc1101_hal.h"

class CC1101HalArduino : public CC1101Hal {
    public:
    // spi  : SPI instance (e.g. SPI)
    // sck/miso/mosi: pins. Only used on ESP32 (custom pin mapping); ignored elsewhere,
    //                other cores use their fixed SPI pins.
    // csn  : chip-select GPIO, driven manually
    // spiHz: SPI clock, CC1101 max 10 MHz (6.5 MHz for burst access), 4 MHz is safe on jumper wires
    // gdo0 : GPIO wired to the CC1101 GDO0 pin, needed only for interrupt mode (-1 = not wired)
    CC1101HalArduino(SPIClass& spi, int8_t sck, int8_t miso, int8_t mosi, int8_t csn,
        uint32_t spiHz = 4000000, int8_t gdo0 = -1)
        : _spi(spi), _sck(sck), _miso(miso), _mosi(mosi), _csn(csn),
        _settings(spiHz, MSBFIRST, SPI_MODE0), _gdo0(gdo0) {}

        ~CC1101HalArduino() override { irqDetach(); }

        bool init() override {
            pinMode(_csn, OUTPUT);
            digitalWrite(_csn, HIGH);
            #if defined(ARDUINO_ARCH_ESP32)
            _spi.begin(_sck, _miso, _mosi, -1);      // -1: CS is driven manually
            #else
            (void)_sck; (void)_mosi;
            _spi.begin();
            #endif
            pinMode(_miso, INPUT);                   // so misoRead() works (chip-ready check)
            return true;
        }

        void    spiBegin() override { _spi.beginTransaction(_settings); }
        void    spiEnd() override   { _spi.endTransaction(); }
        uint8_t spiTransfer(uint8_t out) override { return _spi.transfer(out); }

        void csWrite(bool high) override { digitalWrite(_csn, high ? HIGH : LOW); }
        bool misoRead() override         { return digitalRead(_miso) != 0; }

        // ---- interrupt on GDO0 (falling edge) ----
        bool irqAttach(CC1101IsrFn fn, void* arg) override {
            if (_gdo0 < 0) return false;
            _fn = fn; _arg = arg;
            pinMode(_gdo0, INPUT);
            #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
            attachInterruptArg(digitalPinToInterrupt(_gdo0), &CC1101HalArduino::tramp, this, FALLING);
            #else
            inst() = this;                           // generic cores: no arg support -> one instance only
            attachInterrupt(digitalPinToInterrupt(_gdo0), &CC1101HalArduino::trampNoArg, FALLING);
            #endif
            _attached = true;
            return true;
        }
        void irqDetach() override {
            if (_attached) { detachInterrupt(digitalPinToInterrupt(_gdo0)); _attached = false; }
            _fn = nullptr;
        }

        uint32_t nowMs() override        { return ::millis(); }
        uint32_t nowUs() override        { return ::micros(); }
        void     delayMs(uint32_t ms) override { ::delay(ms); }
        void     delayUs(uint32_t us) override { ::delayMicroseconds(us); }

        private:
        #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
        static void CC1101_ISR_ATTR tramp(void* p) {
            CC1101HalArduino* s = static_cast<CC1101HalArduino*>(p);
            CC1101IsrFn f = s->_fn;
            if (f) f(s->_arg);
        }
        #else
        static CC1101HalArduino*& inst() { static CC1101HalArduino* s = nullptr; return s; }
        static void trampNoArg() {
            CC1101HalArduino* s = inst();
            if (s && s->_fn) s->_fn(s->_arg);
        }
        #endif

        SPIClass&   _spi;
        int8_t      _sck, _miso, _mosi, _csn;
        SPISettings _settings;
        int8_t      _gdo0;
        CC1101IsrFn _fn = nullptr;
        void*       _arg = nullptr;
        bool        _attached = false;
    };
