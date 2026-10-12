#pragma once
// ESP-IDF HAL for the CC1101 driver (spi_master + gpio + esp_timer + FreeRTOS).
// Header-only, works on any ESP32 family chip, ESP-IDF 4.4+ / 5.x.
//
// Notes:
//  * CSN is driven manually (spics_io_num = -1) because the driver keeps CSN low across
//    several bytes. Each byte is a 1-byte polling transaction; the bus is held between
//    spiBegin() and spiEnd() so the per-byte overhead stays small.
//  * Any GPIOs work through the GPIO matrix at 4 MHz.
//  * Set CONFIG_FREERTOS_HZ=1000 (see sdkconfig.defaults) so delayMs(1) really sleeps ~1 ms.
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../cc1101_hal.h"

class CC1101HalEspIdf : public CC1101Hal {
    public:
    // host   : SPI2_HOST (works on all chips) or SPI3_HOST
    // sck/miso/mosi/csn : GPIO numbers
    // spiHz  : SPI clock, CC1101 max 10 MHz (6.5 MHz for burst access), 4 MHz is safe on jumper wires
    // initBus: true  = this HAL calls spi_bus_initialize() (an already-initialised bus is tolerated)
    //          false = you set the bus up yourself, the HAL only adds the device
    // gdo0   : GPIO wired to the CC1101 GDO0 pin, needed only for interrupt mode (-1 = not wired)
    CC1101HalEspIdf(spi_host_device_t host, int sck, int miso, int mosi, int csn,
        uint32_t spiHz = 4000000, bool initBus = true, int gdo0 = -1)
        : _host(host), _sck(sck), _miso(miso), _mosi(mosi), _csn(csn),
        _hz(spiHz), _initBus(initBus), _gdo0(gdo0) {}

        ~CC1101HalEspIdf() override {
            irqDetach();
            if (_dev) spi_bus_remove_device(_dev);
        }

        bool init() override {
            // CSN: push-pull output, idle high
            gpio_config_t io = {};
            io.pin_bit_mask = 1ULL << _csn;
            io.mode         = GPIO_MODE_OUTPUT;
            io.pull_up_en   = GPIO_PULLUP_DISABLE;
            io.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io.intr_type    = GPIO_INTR_DISABLE;
            if (gpio_config(&io) != ESP_OK) return false;
            gpio_set_level((gpio_num_t)_csn, 1);

            if (_initBus) {
                spi_bus_config_t bus = {};
                bus.mosi_io_num     = _mosi;
                bus.miso_io_num     = _miso;
                bus.sclk_io_num     = _sck;
                bus.quadwp_io_num   = -1;
                bus.quadhd_io_num   = -1;
                bus.max_transfer_sz = 64;                        // driver only does 1-byte transfers
                esp_err_t e = spi_bus_initialize(_host, &bus, SPI_DMA_DISABLED);
                if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return false;
            }

            spi_device_interface_config_t dev = {};
            dev.clock_speed_hz = (int)_hz;
            dev.mode           = 0;                            // CPOL=0, CPHA=0
            dev.spics_io_num   = -1;                           // CS handled by csWrite()
            dev.queue_size     = 1;
            return spi_bus_add_device(_host, &dev, &_dev) == ESP_OK;
        }

        void spiBegin() override { spi_device_acquire_bus(_dev, portMAX_DELAY); }
        void spiEnd() override   { spi_device_release_bus(_dev); }

        uint8_t spiTransfer(uint8_t out) override {
            spi_transaction_t t = {};
            t.flags      = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
            t.length     = 8;                                 // bits
            t.tx_data[0] = out;
            spi_device_polling_transmit(_dev, &t);
            return t.rx_data[0];
        }

        void csWrite(bool high) override { gpio_set_level((gpio_num_t)_csn, high ? 1 : 0); }
        bool misoRead() override         { return gpio_get_level((gpio_num_t)_miso) != 0; }

        // ---- interrupt on GDO0 (falling edge) via the GPIO ISR service ----
        bool irqAttach(CC1101IsrFn fn, void* arg) override {
            if (_gdo0 < 0) return false;
            _fn = fn; _arg = arg;

            gpio_config_t io = {};
            io.pin_bit_mask = 1ULL << _gdo0;
            io.mode         = GPIO_MODE_INPUT;
            io.pull_up_en   = GPIO_PULLUP_DISABLE;
            io.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io.intr_type    = GPIO_INTR_NEGEDGE;
            if (gpio_config(&io) != ESP_OK) return false;

            // Service may already be installed by your app: that's fine.
            esp_err_t e = gpio_install_isr_service(0);
            if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return false;

            if (gpio_isr_handler_add((gpio_num_t)_gdo0, &CC1101HalEspIdf::tramp, this) != ESP_OK) return false;
            _attached = true;
            return true;
        }
        void irqDetach() override {
            if (_attached) { gpio_isr_handler_remove((gpio_num_t)_gdo0); _attached = false; }
            _fn = nullptr;
        }

        uint32_t nowMs() override { return (uint32_t)(esp_timer_get_time() / 1000); }
        uint32_t nowUs() override { return (uint32_t)esp_timer_get_time(); }

        void delayMs(uint32_t ms) override {
            TickType_t ticks = pdMS_TO_TICKS(ms);
            vTaskDelay(ticks ? ticks : 1);                     // at least 1 tick, never busy-spin
        }
        void delayUs(uint32_t us) override {
            int64_t end = esp_timer_get_time() + (int64_t)us;
            while (esp_timer_get_time() < end) {}
        }

        private:
        static void CC1101_ISR_ATTR tramp(void* p) {
            CC1101HalEspIdf* s = static_cast<CC1101HalEspIdf*>(p);
            CC1101IsrFn f = s->_fn;
            if (f) f(s->_arg);
        }

        spi_host_device_t   _host;
        int                 _sck, _miso, _mosi, _csn;
        uint32_t            _hz;
        bool                _initBus;
        int                 _gdo0;
        spi_device_handle_t _dev = nullptr;
        CC1101IsrFn         _fn = nullptr;
        void*               _arg = nullptr;
        bool                _attached = false;
    };
