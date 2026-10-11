#include "cc1101.h"

// ======================= SPI settings =======================
static constexpr uint32_t SPI_HZ = 4000000;      // CC1101 max 10 MHz (6.5 MHz burst); 4 MHz is safe on jumper wires

// ======================= Timeouts ===========================
// Max time to wait for a TX to finish. Airtime of 62 bytes @ 4.8 kBaud ~ 150 ms.
// RAISE this if you lower the data rate below.
static constexpr uint32_t TX_TIMEOUT_MS = 500;

// ======================= Registers ==========================
namespace {
    enum : uint8_t {
        HDR_READ = 0x80, HDR_BURST = 0x40,

        REG_IOCFG0 = 0x02, REG_FIFOTHR = 0x03, REG_SYNC1 = 0x04, REG_SYNC0 = 0x05,
        REG_PKTLEN = 0x06, REG_PKTCTRL1 = 0x07, REG_PKTCTRL0 = 0x08, REG_ADDR = 0x09,
        REG_CHANNR = 0x0A, REG_FSCTRL1 = 0x0B, REG_FSCTRL0 = 0x0C,
        REG_FREQ2 = 0x0D, REG_FREQ1 = 0x0E, REG_FREQ0 = 0x0F,
        REG_MDMCFG4 = 0x10, REG_MDMCFG3 = 0x11, REG_MDMCFG2 = 0x12, REG_MDMCFG1 = 0x13,
        REG_MDMCFG0 = 0x14, REG_DEVIATN = 0x15, REG_MCSM1 = 0x17, REG_MCSM0 = 0x18,
        REG_FOCCFG = 0x19, REG_BSCFG = 0x1A, REG_AGCCTRL2 = 0x1B, REG_AGCCTRL1 = 0x1C,
        REG_AGCCTRL0 = 0x1D, REG_FREND1 = 0x21, REG_FREND0 = 0x22,
        REG_FSCAL3 = 0x23, REG_FSCAL2 = 0x24, REG_FSCAL1 = 0x25, REG_FSCAL0 = 0x26,
        REG_TEST2 = 0x2C, REG_TEST1 = 0x2D, REG_TEST0 = 0x2E,
        REG_PATABLE = 0x3E, REG_FIFO = 0x3F,

        CMD_SRES = 0x30, CMD_SRX = 0x34, CMD_STX = 0x35, CMD_SIDLE = 0x36,
        CMD_SFRX = 0x3A, CMD_SFTX = 0x3B,

        ST_VERSION = 0x31, ST_MARCSTATE = 0x35, ST_TXBYTES = 0x3A, ST_RXBYTES = 0x3B,

        MARC_IDLE = 0x01, MARC_RXFIFO_OVERFLOW = 0x11,
    };

    struct RegVal { uint8_t reg, val; };

    // -------- Radio configuration (433 MHz band) --------
    // Currently: GFSK, 4.8 kBaud, ~25 kHz deviation, ~100 kHz RX bandwidth,
    // 4-byte preamble, 32-bit sync (30/32 must match), variable length packets.
    // Values based on TI SmartRF Studio output for 433 MHz.
    //
    // >>> To change the DATA RATE / BANDWIDTH / DEVIATION edit these three:
    //   MDMCFG4 [7:4]=RX BW exponent/mantissa, [3:0]=data rate exponent
    //   MDMCFG3 = data rate mantissa
    //   DEVIATN = frequency deviation
    //   Known-good alternative for 38.4 kBaud:  MDMCFG4=0xCA, MDMCFG3=0x83, DEVIATN=0x34
    //   (then keep an eye on TX_TIMEOUT_MS and expect shorter range)
    //
    // >>> MDMCFG2 [6:4] = modulation: 000 = 2-FSK, 001 = GFSK, 011 = OOK/ASK, 111 = MSK
    //     MDMCFG2 [2:0] = sync mode: 011 = 30/32 sync bits (used here), 110 = 16/16 + carrier sense
    //     MDMCFG1 bit7  = hardware FEC (convolutional) - kept OFF since you do your own ECC
    const RegVal RADIO_CFG[] = {
        {REG_IOCFG0,   0x06},   // GDO0: asserts on sync word, deasserts at end of packet (optional pin, not required)
        {REG_FIFOTHR,  0x07},
        {REG_PKTLEN,   CC1101::MAX_PAYLOAD},  // max accepted length in variable-length mode
        {REG_PKTCTRL1, 0x04},   // no HW address filter, append RSSI+LQI to RX FIFO
        {REG_ADDR,     0x00},
        {REG_CHANNR,   0x00},
        {REG_FSCTRL1,  0x06},   // IF frequency ~152 kHz
        {REG_FSCTRL0,  0x00},
        {REG_MDMCFG4,  0xC7},   // RX BW ~100 kHz, data rate exp
        {REG_MDMCFG3,  0x83},   // data rate mantissa -> 4.8 kBaud
        {REG_MDMCFG2,  0x13},   // GFSK, 30/32 sync word
        {REG_MDMCFG1,  0x22},   // FEC off, 4 preamble bytes
        {REG_MDMCFG0,  0xF8},
        {REG_DEVIATN,  0x40},   // ~25 kHz deviation
        // MCSM1: CCA_MODE [5:4], RXOFF_MODE [3:2], TXOFF_MODE [1:0]
        //   0x00 = transmit immediately, go IDLE after RX/TX (this driver relies on IDLE after packets)
        //   0x30 = listen-before-talk: TX only if channel is clear (send() then returns false when busy)
        {REG_MCSM1,    0x00},
        {REG_MCSM0,    0x18},   // auto-calibrate when leaving IDLE
        {REG_FOCCFG,   0x16},
        {REG_BSCFG,    0x6C},
        {REG_AGCCTRL2, 0x43},
        {REG_AGCCTRL1, 0x40},
        {REG_AGCCTRL0, 0x91},
        {REG_FREND1,   0x56},
        {REG_FREND0,   0x10},
        {REG_FSCAL3,   0xE9},
        {REG_FSCAL2,   0x2A},
        {REG_FSCAL1,   0x00},
        {REG_FSCAL0,   0x1F},
        {REG_TEST2,    0x81},
        {REG_TEST1,    0x35},
        {REG_TEST0,    0x09},
    };

    // PATABLE values for ~433 MHz (from the datasheet table, approximate).
    struct PaEntry { int8_t dbm; uint8_t pa; };
    const PaEntry PA_TABLE_433[] = {
        {-30, 0x12}, {-20, 0x0E}, {-15, 0x1D}, {-10, 0x34},
        {  0, 0x60}, {  5, 0x84}, {  7, 0xC8}, { 10, 0xC0},
    };

    uint8_t paFromDbm(int8_t dbm) {
        uint8_t pa = PA_TABLE_433[0].pa;
        for (const PaEntry& e : PA_TABLE_433) if (e.dbm <= dbm) pa = e.pa;
        return pa;
    }
}  // namespace

// ======================= Low-level SPI ======================
void CC1101::select() {
    digitalWrite(_csn, LOW);
    // CC1101 pulls MISO low when its crystal is ready
    uint32_t t = micros();
    while (digitalRead(_miso) && (micros() - t) < 2000) {}
}
void CC1101::deselect() { digitalWrite(_csn, HIGH); }

uint8_t CC1101::strobe(uint8_t cmd) {
    _spi->beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    select();
    uint8_t st = _spi->transfer(cmd);
    deselect();
    _spi->endTransaction();
    return st;
}

void CC1101::writeReg(uint8_t reg, uint8_t val) {
    _spi->beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    select();
    _spi->transfer(reg);
    _spi->transfer(val);
    deselect();
    _spi->endTransaction();
}

uint8_t CC1101::readReg(uint8_t reg) {
    _spi->beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    select();
    _spi->transfer(reg | HDR_READ);
    uint8_t v = _spi->transfer(0);
    deselect();
    _spi->endTransaction();
    return v;
}

// Status registers (0x30..0x3D) need the burst bit set when reading,
// otherwise they are interpreted as command strobes.
uint8_t CC1101::readStatus(uint8_t reg) {
    _spi->beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    select();
    _spi->transfer(reg | HDR_READ | HDR_BURST);
    uint8_t v = _spi->transfer(0);
    deselect();
    _spi->endTransaction();
    return v;
}

void CC1101::writeBurst(uint8_t reg, const uint8_t* d, uint8_t n) {
    _spi->beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    select();
    _spi->transfer(reg | HDR_BURST);
    for (uint8_t i = 0; i < n; i++) _spi->transfer(d[i]);
    deselect();
    _spi->endTransaction();
}

void CC1101::readBurst(uint8_t reg, uint8_t* d, uint8_t n) {
    _spi->beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    select();
    _spi->transfer(reg | HDR_READ | HDR_BURST);
    for (uint8_t i = 0; i < n; i++) d[i] = _spi->transfer(0);
    deselect();
    _spi->endTransaction();
}

// ======================= State helpers ======================
uint8_t CC1101::marcState() { return readStatus(ST_MARCSTATE) & 0x1F; }

bool CC1101::goIdle() {
    strobe(CMD_SIDLE);
    uint32_t t = millis();
    while (marcState() != MARC_IDLE) {
        if (millis() - t > 20) return false;
    }
    return true;
}

// RXBYTES can return a wrong value if read while the FIFO is being updated
// (silicon erratum): read until two consecutive reads agree.
uint8_t CC1101::rxBytes() {
    uint8_t a = readStatus(ST_RXBYTES), b;
    for (uint8_t i = 0; i < 5; i++) {
        b = readStatus(ST_RXBYTES);
        if (a == b) return a;
        a = b;
    }
    return a;
}

void CC1101::startRx() {
    goIdle();
    strobe(CMD_SFRX);
    strobe(CMD_SRX);
    uint32_t t = millis();
    while (marcState() == MARC_IDLE && millis() - t < 5) {}   // wait until it left IDLE
    _rx = true;
}

// ======================= Public API =========================
void CC1101::reset() {
    // Power-on reset sequence from the datasheet
    digitalWrite(_csn, HIGH); delayMicroseconds(5);
    digitalWrite(_csn, LOW);  delayMicroseconds(10);
    digitalWrite(_csn, HIGH); delayMicroseconds(45);

    _spi->beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    select();
    _spi->transfer(CMD_SRES);
    uint32_t t = micros();
    while (digitalRead(_miso) && (micros() - t) < 5000) {}    // wait for reset to complete
    deselect();
    _spi->endTransaction();
    delay(1);
}

uint8_t CC1101::version() { return readStatus(ST_VERSION); }

void CC1101::applyConfig() {
    for (const RegVal& rv : RADIO_CFG) writeReg(rv.reg, rv.val);

    // PKTCTRL0: bit6 whitening, bit2 CRC enable, [1:0]=01 variable packet length
    writeReg(REG_PKTCTRL0, 0x01 | (_cfg.whitening ? 0x40 : 0x00) | (_cfg.hwCrc ? 0x04 : 0x00));

    // PATABLE (index 0 is used for FSK/GFSK)
    uint8_t pa[8] = {paFromDbm(_cfg.txPowerDbm), 0, 0, 0, 0, 0, 0, 0};
    writeBurst(REG_PATABLE, pa, 8);

    writeReg(REG_SYNC1, _cfg.syncWord >> 8);
    writeReg(REG_SYNC0, _cfg.syncWord & 0xFF);
}

bool CC1101::begin(SPIClass& spi, int8_t sck, int8_t miso, int8_t mosi, int8_t csn,
    const Config& cfg) {
        _spi = &spi; _miso = miso; _csn = csn; _cfg = cfg;

        pinMode(_csn, OUTPUT);
        digitalWrite(_csn, HIGH);
        _spi->begin(sck, miso, mosi, -1);       // CS is driven manually
        pinMode(_miso, INPUT);

        reset();

        uint8_t v = version();
        if (v == 0x00 || v == 0xFF) return false;   // no chip / wiring problem

        goIdle();
        applyConfig();
        if (!setFrequency(_cfg.freqMHz)) return false;
        startRx();
        return true;
    }

    bool CC1101::setFrequency(double mhz) {
        // Module is a 433 band part; this register set is tuned for 387..464 MHz.
        // (For 868/915 MHz you'd need other FSCAL/TEST values and a different matching network.)
        if (mhz < 387.0 || mhz > 464.0) return false;

        // FREQ = f_carrier * 2^16 / f_xosc   (f_xosc = 26 MHz)
        uint32_t w = (uint32_t)(mhz * 65536.0 / 26.0 + 0.5);

        bool wasRx = _rx;
        goIdle();
        writeReg(REG_FREQ2, (w >> 16) & 0xFF);
        writeReg(REG_FREQ1, (w >> 8) & 0xFF);
        writeReg(REG_FREQ0, w & 0xFF);
        _cfg.freqMHz = mhz;
        if (wasRx) startRx();                      // auto-calibration happens on IDLE->RX
        return true;
    }

    void CC1101::setSyncWord(uint16_t sw) {
        bool wasRx = _rx;
        goIdle();
        _cfg.syncWord = sw;
        writeReg(REG_SYNC1, sw >> 8);
        writeReg(REG_SYNC0, sw & 0xFF);
        if (wasRx) startRx();
    }

    bool CC1101::send(const uint8_t* data, uint8_t len) {
        if (len == 0 || len > MAX_PAYLOAD) return false;

        goIdle();
        strobe(CMD_SFTX);

        uint8_t frame[MAX_PAYLOAD + 1];
        frame[0] = len;                            // length byte, consumed by the radio (variable length mode)
        memcpy(frame + 1, data, len);
        writeBurst(REG_FIFO, frame, len + 1);

        strobe(CMD_STX);

        bool ok = false;
        uint32_t t = millis();
        while (millis() - t < TX_TIMEOUT_MS) {
            uint8_t tx = readStatus(ST_TXBYTES);
            if (tx & 0x80) break;                                    // TX FIFO underflow
            if ((tx & 0x7F) == 0 && marcState() == MARC_IDLE) { ok = true; break; }
            delay(1);
        }

        if (!ok) { goIdle(); strobe(CMD_SFTX); }   // CCA blocked / error: clean up
        startRx();
        return ok;
    }

    int16_t CC1101::receive(uint8_t* buf, uint8_t maxLen, RxInfo* info) {
        uint8_t marc = marcState();

        if (marc == MARC_RXFIFO_OVERFLOW) { startRx(); return 0; }
        if (marc != MARC_IDLE) return 0;           // still in RX (or calibrating): nothing complete yet

        // IDLE while we expect RX => either a packet finished (RXOFF_MODE=IDLE) or we lost RX
        uint8_t n = rxBytes();
        if (n == 0 || (n & 0x80)) { startRx(); return 0; }

        uint8_t raw[64];
        if (n > sizeof(raw)) n = sizeof(raw);
        readBurst(REG_FIFO, raw, n);

        uint8_t len = raw[0];
        int16_t ret;
        if (len == 0 || (uint16_t)len + 3 > n) {            // malformed (needs len byte + payload + 2 status)
            ret = 0;
        } else if (len > maxLen) {
            ret = -1;
        } else {
            memcpy(buf, raw + 1, len);
            if (info) {
                uint8_t rssiRaw = raw[len + 1], lqiCrc = raw[len + 2];
                int16_t r = (rssiRaw >= 128) ? ((int16_t)rssiRaw - 256) : (int16_t)rssiRaw;
                info->rssiDbm = (r / 2) - 74;                    // RSSI offset ~74 dB at 433 MHz
                info->lqi     = lqiCrc & 0x7F;
                info->crcOk   = (lqiCrc & 0x80) != 0;
            }
            ret = len;
        }

        startRx();
        return ret;
    }
