#pragma once
// AFEC - protocol node on top of the CC1101 driver (cc1101.h).
//
//   TX:  payload -> frame (CRC + ECC @ per-peer level) -> duty check -> CSMA -> CC1101::send()
//   RX:  CC1101::receive() -> decode -> LHS score -> ACK/NACK feedback -> deliver / relay
//   Closed loop: the receiver computes the 4-bit LHS, returns it in the ACK, the transmitter
//   runs Table 1 on it and picks the ECC level for the next packet.
//
// Radio config to use:  Config.hwCrc = false (we carry our own CRC-16 inside the ECC),
//                       Config.whitening = true, same freq/sync word on all nodes.
#include <stdint.h>
#include "cc1101.h"
#include "afec_frame.h"
#include "afec_lhs.h"

namespace afec {

struct Platform {
    uint32_t (*millis)()                  = nullptr;   // required
    uint32_t (*random)(uint32_t maxExcl)  = nullptr;   // required, returns [0, maxExcl)
    bool     (*channelBusy)()             = nullptr;   // optional carrier sense (RSSI/CCA)
    void     (*yield)()                   = nullptr;   // optional (e.g. Arduino yield())
};

struct NodeConfig {
    uint8_t  id            = 1;       // 0x01..0xFE (0xFF = broadcast)
    uint8_t  initialLevel  = 2;       // ladder start (BCH t=5)
    uint8_t  defaultTtl    = 3;       // 0..15 hops
    uint32_t bitrateBps    = 4800;    // must match your CC1101 modem setup (airtime / duty maths)
    uint8_t  dutyPercent   = 10;      // ETSI EN 300 220-2 style limit
    uint32_t dutyBurstMs   = 36000;   // bucket capacity (how much airtime may be burst)
    uint16_t ackTimeoutMs  = 400;
    uint8_t  maxRetries    = 3;
    uint8_t  downgradeAfter= 2;       // consecutive 0xF scores before stepping down (paper: 1)
    uint8_t  nackStep      = 2;       // levels to jump on NACK
    uint16_t relayJitterMinMs = 10;
    uint16_t relayJitterMaxMs = 120;
    uint16_t nackHoldoffMs = 150;     // min gap between NACKs we emit
    LhsParams lhs;
};

enum class Result : uint8_t { OK, NO_ACK, DUTY_LIMIT, TOO_LONG, RADIO_ERR };

struct Stats {
    uint32_t txFrames = 0, txRetries = 0, acksRx = 0, nacksRx = 0, timeouts = 0;
    uint32_t rxOk = 0, rxDup = 0, rxBadHeader = 0, rxEccFail = 0, rxCrcFail = 0;
    uint32_t acksTx = 0, nacksTx = 0, relayed = 0, relayDropped = 0, dutyBlocked = 0;
    uint32_t eccCorrected = 0;   // total symbols/bits repaired
};

class Node {
public:
    // Called from poll()/send() context when a DATA frame for us (or broadcast) arrives.
    // Do NOT call send() from inside it - set a flag and send from loop().
    using RxCb = void (*)(void* ctx, uint8_t src, const uint8_t* payload, uint8_t len, uint8_t lhs);

    // `radio` must already be begin()'d and outlive the node.
    bool begin(CC1101& radio, const NodeConfig& cfg, const Platform& plat);
    void onData(RxCb cb, void* ctx = nullptr) { _cb = cb; _cbCtx = ctx; }

    // reliable=true: waits for ACK, adapts level, retries (NACK -> immediate, timeout -> max ECC).
    Result send(uint8_t dst, const uint8_t* payload, uint8_t len, bool reliable = true);

    void poll();                                   // call from loop(): RX, ACK, relay queue

    uint8_t peerLevel(uint8_t id);                 // current ladder level towards `id`
    uint8_t hopLevel() const { return _hopLevel; } // level used for relayed frames
    const Stats& stats() const { return _st; }

private:
    struct Peer  { uint8_t id = 0; bool used = false; uint8_t level = 0, rxScore = 0xA, streak = 0; uint32_t lastUse = 0; };
    struct Wait  { bool active = false; uint8_t dst = 0, seq = 0; bool ack = false, nack = false; uint8_t score = 0; };
    struct Relay { bool used = false; uint32_t due = 0; Frame f; };
    struct Seen  { uint8_t src, seq; bool used; };

    static constexpr uint8_t NPEERS = 8, NSEEN = 32, NRELAY = 2;

    Peer&  peer(uint8_t id);
    void   applyScore(Peer& p, uint8_t score);
    void   applyHop(uint8_t score);
    bool   seenOrAdd(uint8_t src, uint8_t seq);

    Result transmit(const Frame& f, uint8_t level, bool csma = true);
    void   processRx();
    void   handleAir(const uint8_t* buf, uint8_t n, const CC1101::RxInfo& ri);
    void   sendAck(const Frame& rx, uint8_t score);
    void   sendNack(uint8_t score);
    void   queueRelay(Frame f, uint8_t score);
    void   pumpRelay();
    void   refillDuty();
    uint32_t airtimeUs(uint8_t bytes) const;

    CC1101*    _r = nullptr;
    NodeConfig _cfg;
    Platform   _p;
    Lhs        _lhs;
    RxCb       _cb = nullptr; void* _cbCtx = nullptr;

    Peer   _peers[NPEERS];
    Seen   _seen[NSEEN]; uint8_t _seenHead = 0;
    Relay  _rq[NRELAY];
    Wait   _wait;
    Stats  _st;

    uint8_t  _seq = 0, _hopLevel = 0, _hopStreak = 0;
    uint32_t _tick = 0, _lastNack = 0; bool _nackEver = false;
    int64_t  _budgetUs = 0; uint32_t _lastRefill = 0;
};

} // namespace afec
