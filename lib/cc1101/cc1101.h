#pragma once
// Minimal bare-register CC1101 driver (no 3rd-party radio libs).
// Platform independent: all hardware access goes through CC1101Hal (see cc1101_hal.h).
// For Arduino use hal/cc1101_hal_arduino.h.
#include <stdint.h>
#include <stddef.h>
#include "cc1101_hal.h"

class CC1101 {
    public:
    // CC1101 FIFO is 64 bytes. On the air/FIFO a packet is:
    //   [length byte][payload ...]            (TX: 1 + payload  <= 64)
    //   RX additionally gets 2 appended status bytes (RSSI, LQI) -> payload max = 61.
    // The length byte is handled by the radio, your payload is whatever you want.
    static constexpr uint8_t MAX_PAYLOAD = 61;

    struct Config {
        // ---- Carrier frequency in MHz (433 module: 433.05 .. 434.79 is the EU ISM band).
        //      Accepted range 387..464 MHz. Resolution ~397 Hz.
        double   freqMHz    = 433.92;

        // ---- 16-bit sync word: THE mechanism that separates your packets from others.
        //      The radio only accepts a packet if it sees this word (sent twice = 32 bits,
        //      30/32 must match). Pick any "random" value, same on TX and RX.
        //      Avoid 0x0000, 0xFFFF, 0xAAAA, 0x5555 and TI's default 0xD391.
        uint16_t syncWord   = 0x6B1D;

        // ---- TX power, approximate for 433 MHz. Allowed: -30,-20,-15,-10,0,5,7,10 dBm
        //      (a value in between rounds down). Mind your local regulations!
        int8_t   txPowerDbm = 0;

        // ---- Data whitening (PN9). Transparent to you, keeps the bit stream balanced
        //      (long runs of 0s/1s hurt reception). Must match on both sides.
        bool     whitening  = true;

        // ---- Hardware CRC-16. OFF by default so packets with bit errors still reach you
        //      (you said you'll add your own ECC). If ON, bad packets are silently dropped
        //      by the radio and RxInfo::crcOk is meaningful.
        bool     hwCrc      = false;
    };

    struct RxInfo {
        int16_t rssiDbm;  // received signal strength
        uint8_t lqi;      // link quality indicator (lower = better)
        bool    crcOk;    // only meaningful when Config::hwCrc == true
    };

    // Pass your HAL instance (must outlive this object). Calls hal.init().
    // Returns false if the chip doesn't answer (wiring / power problem).
    bool begin(CC1101Hal& hal, const Config& cfg);

    bool setFrequency(double mhz);          // can be changed at runtime
    void setSyncWord(uint16_t sw);          // can be changed at runtime

    // Blocking send (returns when the packet is out). 1..MAX_PAYLOAD bytes.
    bool send(const uint8_t* data, uint8_t len);

    // Non-blocking poll. Returns payload length (>0) if a packet arrived,
    // 0 if nothing yet, -1 if the packet didn't fit in maxLen (dropped).
    // The radio is parked in RX automatically between calls.
    int16_t receive(uint8_t* buf, uint8_t maxLen, RxInfo* info = nullptr);

    uint8_t version();                      // chip VERSION register (typ. 0x14 / 0x04)

    private:
    void    reset();
    void    applyConfig();
    void    startRx();
    bool    goIdle();
    uint8_t marcState();
    uint8_t rxBytes();

    uint8_t strobe(uint8_t cmd);
    void    writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    uint8_t readStatus(uint8_t reg);
    void    writeBurst(uint8_t reg, const uint8_t* d, uint8_t n);
    void    readBurst(uint8_t reg, uint8_t* d, uint8_t n);
    void    select();
    void    deselect();

    CC1101Hal* _hal = nullptr;
    Config     _cfg;
    bool       _rx  = false;
};
