#pragma once
// AFEC protocol family on top of cc1101.h. One engine, five modes, so C1..C5 share framing,
// CRC-16, ACK format, ACK timeout and retry budget; only the error-handling rule differs.
//
//   Mode::RAW    C1  CRC-16 only, no ACK, no retry
//   Mode::ARQ    C2  CRC-16 + stop-and-wait ARQ (ACK, up to maxRetries)
//   Mode::HWFEC  C3  as C2, radio runs CC1101 hardware FEC (needs the driver patch, see below)
//   Mode::RSSI   C4  same ladder/codecs/NACK as AFEC; level from quantised RSSI + hysteresis
//   Mode::AFEC   C5  ladder driven by the 4-bit Link Health Score (Table 1)
//
// C3 needs your driver in FIXED-length mode with FEC_EN (datasheet: FEC works only with
// PKTCTRL0.LENGTH_CONFIG=0). See cc1101_fec_patch.txt. Use Config.hwCrc=false for all modes.
#include <stdint.h>
#include "cc1101.h"
#include "afec_frame.h"
#include "afec_lhs.h"

namespace afec {

enum class Mode : uint8_t { RAW, ARQ, HWFEC, RSSI, AFEC };

struct Platform {
    uint32_t (*millis)()                 = nullptr;   // required
    uint32_t (*random)(uint32_t maxExcl) = nullptr;   // required, [0,maxExcl)
    void     (*yield)()                  = nullptr;   // optional
};

// C4 rule. q = RSSI quantised to 4 bits over [minDbm,maxDbm]; the ACK carries q in the same 4 bits
// AFEC uses for its score. Step up from level i when q < up[i]; step down when q >= up[i-1]+hys.
// Defaults are placeholders: tune once, then freeze.
struct RssiPolicy {
    int16_t minDbm = -110, maxDbm = -50;
    uint8_t up[MAX_LEVEL] = { 11, 9, 7, 5, 3 };
    uint8_t hys = 1;
};

struct NodeConfig {
    uint8_t  id            = 1;          // 0x01..0xFE (0xFF = broadcast)
    Mode     mode          = Mode::AFEC;
    uint8_t  initialLevel  = 2;          // C4/C5 start level
    uint8_t  fixedPayloadLen = 0;        // C3: ACKs are padded to this size (fixed-length radio)
    uint8_t  defaultTtl    = 3;
    uint32_t bitrateBps    = 4800;       // for airtime/duty accounting
    uint8_t  dutyPercent   = 10;
    uint32_t dutyBurstMs   = 36000;
    uint16_t ackTimeoutMs  = 400;        // same for C2..C5
    uint8_t  maxRetries    = 3;          // R_max, same for C2..C5
    uint8_t  downgradeAfter= 2;          // C5: consecutive 0xF before stepping down (paper: 1)
    uint8_t  nackStep      = 2;          // C4/C5: levels to jump on NACK
    uint16_t relayJitterMinMs = 10, relayJitterMaxMs = 120;
    uint16_t nackHoldoffMs = 150;
    LhsParams  lhs;
    RssiPolicy rssi;
};

enum class Result : uint8_t { OK, NO_ACK, DUTY_LIMIT, TOO_LONG, RADIO_ERR };

struct Stats {
    uint32_t txFrames = 0, txRetries = 0, acksRx = 0, nacksRx = 0, timeouts = 0;
    uint32_t rxOk = 0, rxDup = 0, rxBadHeader = 0, rxEccFail = 0, rxCrcFail = 0;
    uint32_t acksTx = 0, nacksTx = 0, relayed = 0, relayDropped = 0, dutyBlocked = 0;
    uint32_t eccCorrected = 0;
};

class Node {
public:
    // Called when a DATA frame for us (or broadcast) arrives. Don't call send() inside it.
    using RxCb = void (*)(void* ctx, uint8_t src, const uint8_t* payload, uint8_t len, uint8_t fb);

    bool begin(CC1101& radio, const NodeConfig& cfg, const Platform& plat);   // radio already begin()'d
    void onData(RxCb cb, void* ctx = nullptr) { _cb = cb; _cbCtx = ctx; }

    Result send(uint8_t dst, const uint8_t* payload, uint8_t len, bool reliable = true);
    void poll();                                   // call from loop()

    uint8_t peerLevel(uint8_t id);                 // LEVEL_NONE for C1-C3
    void    resetLink(uint8_t id);                 // back to initialLevel (start of a block)
    const Stats& stats() const { return _st; }

private:
    struct Peer  { uint8_t id = 0; bool used = false; uint8_t level = 0, rxFb = 0xA, streak = 0; uint32_t lastUse = 0; };
    struct Wait  { bool active = false; uint8_t dst = 0, seq = 0; bool ack = false, nack = false; uint8_t fb = 0; };
    struct Relay { bool used = false; uint32_t due = 0; Frame f; };
    struct Seen {
        uint8_t src = 0;
        uint8_t seq = 0;
        bool used = false;

        Seen(uint8_t s, uint8_t q, bool u) : src(s), seq(q), used(u) {}
        Seen() = default;
    };
    static constexpr uint8_t NPEERS = 8, NSEEN = 32, NRELAY = 2;

    bool    adaptive() const { return _cfg.mode == Mode::RSSI || _cfg.mode == Mode::AFEC; }
    uint8_t levelFor(const Peer& p) const;
    uint8_t rssiQ(int16_t rssi) const;
    uint8_t feedback(uint8_t score, int16_t rssi) const;
    void    stepLevel(uint8_t& level, uint8_t& streak, uint8_t fb);
    void    nackJump(uint8_t& level);
    Peer&   peer(uint8_t id);
    bool    seenOrAdd(uint8_t src, uint8_t seq);

    Result  transmit(const Frame& f, uint8_t level);
    void    processRx();
    void    handleAir(const uint8_t* buf, uint8_t n, const CC1101::RxInfo& ri);
    void    sendAck(const Frame& rx, uint8_t fb);
    void    sendNack(uint8_t fb);
    void    queueRelay(Frame f, uint8_t fb);
    void    pumpRelay();
    void    refillDuty();
    uint32_t airtimeUs(uint8_t bytes) const;

    CC1101*    _r = nullptr;
    NodeConfig _cfg;
    Platform   _p;
    Lhs        _lhs;
    RxCb _cb = nullptr; void* _cbCtx = nullptr;

    Peer  _peers[NPEERS];
    Seen  _seen[NSEEN]; uint8_t _seenHead = 0;
    Relay _rq[NRELAY];
    Wait  _wait;
    Stats _st;

    uint8_t  _seq = 0, _hopLevel = 0, _hopStreak = 0;
    uint32_t _tick = 0, _lastNack = 0; bool _nackEver = false;
    int64_t  _budgetUs = 0; uint32_t _lastRefill = 0;
};

} // namespace afec
