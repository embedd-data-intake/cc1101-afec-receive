#include "afec_node.h"
#include <string.h>

namespace afec {

bool Node::begin(CC1101& radio, const NodeConfig& cfg, const Platform& plat) {
    if (!plat.millis || !plat.random) return false;
    _r = &radio; _cfg = cfg; _p = plat;
    if (_cfg.initialLevel > MAX_LEVEL) _cfg.initialLevel = MAX_LEVEL;
    eccInit();
    _lhs.begin(_cfg.lhs);
    _hopLevel = _cfg.initialLevel;
    _budgetUs = (int64_t)_cfg.dutyBurstMs * 1000;
    _lastRefill = _p.millis();
    return true;
}

// ------------------------------------------------------------------ level selection
Node::Peer& Node::peer(uint8_t id) {
    Peer* victim = &_peers[0];
    for (auto& p : _peers) {
        if (p.used && p.id == id) { p.lastUse = ++_tick; return p; }
        if (!p.used) victim = &p;
        else if (victim->used && p.lastUse < victim->lastUse) victim = &p;
    }
    *victim = Peer(); victim->used = true; victim->id = id;
    victim->level = _cfg.initialLevel; victim->lastUse = ++_tick;
    return *victim;
}

void Node::resetLink(uint8_t id) {
    Peer& p = peer(id); p.level = _cfg.initialLevel; p.streak = 0; p.rxFb = 0xA;
    _hopLevel = _cfg.initialLevel; _hopStreak = 0;
}

uint8_t Node::peerLevel(uint8_t id) { return levelFor(peer(id)); }

uint8_t Node::levelFor(const Peer& p) const { return adaptive() ? p.level : LEVEL_NONE; }

uint8_t Node::rssiQ(int16_t r) const {
    const RssiPolicy& p = _cfg.rssi;
    if (r < p.minDbm) r = p.minDbm;
    if (r > p.maxDbm) r = p.maxDbm;
    return (uint8_t)(((int32_t)(r - p.minDbm) * 15) / (p.maxDbm - p.minDbm));
}

uint8_t Node::feedback(uint8_t score, int16_t rssi) const {
    if (_cfg.mode == Mode::AFEC) return score;
    if (_cfg.mode == Mode::RSSI) return rssiQ(rssi);
    return 0;
}

// One level step per ACK. C5: Table 1. C4: threshold + hysteresis on the same 4 bits.
void Node::stepLevel(uint8_t& level, uint8_t& streak, uint8_t fb) {
    if (_cfg.mode == Mode::AFEC) {
        switch (Lhs::decide(fb)) {
            case Action::DOWNGRADE:    if (++streak >= _cfg.downgradeAfter) { if (level > 0) level--; streak = 0; } return;
            case Action::HOLD:         streak = 0; return;
            case Action::UPGRADE:      streak = 0; if (level < MAX_LEVEL) level++; return;
            case Action::NACK_UPGRADE: streak = 0; nackJump(level); return;
            case Action::MAX_PROTECT:  streak = 0; level = MAX_LEVEL; return;
        }
    } else if (_cfg.mode == Mode::RSSI) {
        const RssiPolicy& p = _cfg.rssi;
        if (level < MAX_LEVEL && fb < p.up[level]) level++;
        else if (level > 0 && fb >= p.up[level - 1] + p.hys) level--;
    }
}

void Node::nackJump(uint8_t& level) {
    level = (level + _cfg.nackStep > MAX_LEVEL) ? MAX_LEVEL : level + _cfg.nackStep;
}

bool Node::seenOrAdd(uint8_t src, uint8_t seq) {
    for (auto& s : _seen) if (s.used && s.src == src && s.seq == seq) return true;
    _seen[_seenHead] = { src, seq, true };
    _seenHead = (_seenHead + 1) % NSEEN;
    return false;
}

// ------------------------------------------------------------------ duty cycle
uint32_t Node::airtimeUs(uint8_t bytes) const {
    uint32_t us = (uint32_t)(bytes + 9) * 8UL * 1000000UL / _cfg.bitrateBps;
    return _cfg.mode == Mode::HWFEC ? us * 2 : us;               // rate-1/2 FEC doubles airtime
}

void Node::refillDuty() {
    uint32_t now = _p.millis(), dt = now - _lastRefill; _lastRefill = now;
    _budgetUs += (int64_t)dt * 10 * _cfg.dutyPercent;
    int64_t cap = (int64_t)_cfg.dutyBurstMs * 1000;
    if (_budgetUs > cap) _budgetUs = cap;
}

// ------------------------------------------------------------------ TX path
Result Node::transmit(const Frame& fin, uint8_t level) {
    Frame f = fin;
    if (_cfg.fixedPayloadLen && f.plen < _cfg.fixedPayloadLen) {      // C3 fixed-length radio: pad
        memset(f.payload + f.plen, 0, _cfg.fixedPayloadLen - f.plen); f.plen = _cfg.fixedPayloadLen;
    }
    uint8_t buf[AIR_MAX], n = 0;
    level = fitLevel(level, f.plen);
    if (!encodeFrame(f, level, buf, &n)) return Result::TOO_LONG;

    refillDuty();
    uint32_t at = airtimeUs(n);
    if (_budgetUs < (int64_t)at) { _st.dutyBlocked++; return Result::DUTY_LIMIT; }
    if (!_r->send(buf, n)) return Result::RADIO_ERR;
    _budgetUs -= at; _st.txFrames++;
    return Result::OK;
}

Result Node::send(uint8_t dst, const uint8_t* payload, uint8_t len, bool reliable) {
    if (dst == BROADCAST || _cfg.mode == Mode::RAW) reliable = false;
    if (len > maxPayload(adaptive() ? 0 : LEVEL_NONE) || (len && !payload)) return Result::TOO_LONG;

    Frame f;
    f.src = _cfg.id; f.dst = dst; f.seq = _seq++; f.ttl = _cfg.defaultTtl;
    f.type = MsgType::DATA; f.flags = reliable ? FLAG_ACK_REQ : 0;
    f.plen = len; if (len) memcpy(f.payload, payload, len);
    seenOrAdd(f.src, f.seq);

    const uint8_t tries = reliable ? 1 + _cfg.maxRetries : 1;
    for (uint8_t a = 0; a < tries; a++) {
        Peer& pr = peer(dst);
        f.lhs = pr.rxFb & 0x0F;
        if (a) _st.txRetries++;

        if (reliable) { _wait = Wait(); _wait.active = true; _wait.dst = dst; _wait.seq = f.seq; }
        Result r = transmit(f, levelFor(pr));
        if (r != Result::OK) { _wait.active = false; return r; }
        if (!reliable) return Result::OK;

        uint32_t t0 = _p.millis();
        while (!_wait.ack && !_wait.nack && (uint32_t)(_p.millis() - t0) < _cfg.ackTimeoutMs) {
            processRx();
            if (_p.yield) _p.yield();
        }
        _wait.active = false;
        Peer& after = peer(dst);
        if (_wait.ack)  { if (adaptive()) stepLevel(after.level, after.streak, _wait.fb); return Result::OK; }
        if (_wait.nack) { if (adaptive()) { after.streak = 0; nackJump(after.level); } continue; }   // retransmit at once
        _st.timeouts++;
        if (adaptive()) { after.streak = 0; after.level = MAX_LEVEL; }                              // "0x0": max protection
    }
    return Result::NO_ACK;
}

// ------------------------------------------------------------------ RX path
void Node::poll() { processRx(); pumpRelay(); }

void Node::processRx() {
    uint8_t buf[AIR_MAX]; CC1101::RxInfo ri;
    for (uint8_t guard = 0; guard < 4; guard++) {
        int16_t n = _r->receive(buf, sizeof buf, &ri);
        if (n <= 0) return;
        handleAir(buf, (uint8_t)n, ri);
    }
}

void Node::handleAir(const uint8_t* buf, uint8_t n, const CC1101::RxInfo& ri) {
    Frame f; DecodeInfo di;
    switch (decodeFrame(buf, n, f, di)) {
        case DecodeStatus::OK: break;
        case DecodeStatus::TOO_SHORT:
        case DecodeStatus::BAD_HEADER: _st.rxBadHeader++; return;            // silent: sender times out
        case DecodeStatus::ECC_FAIL:   _st.rxEccFail++; if (adaptive()) sendNack(feedback(_lhs.score(0, di.t, ri.rssiDbm, ri.lqi, true), ri.rssiDbm)); return;
        case DecodeStatus::CRC_FAIL:   _st.rxCrcFail++; if (adaptive()) sendNack(feedback(_lhs.score(0, di.t, ri.rssiDbm, ri.lqi, true), ri.rssiDbm)); return;
    }
    _st.rxOk++; _st.eccCorrected += di.corrected;
    uint8_t fb = feedback(_lhs.score(di.corrected, di.t, ri.rssiDbm, ri.lqi, false), ri.rssiDbm);

    if (f.type == MsgType::NACK) {                                            // link-local, never relayed
        if (_wait.active) { _wait.nack = true; _wait.fb = f.lhs; _st.nacksRx++; }
        return;
    }
    if (f.src == _cfg.id) return;
    peer(f.src).rxFb = fb;
    if (adaptive()) stepLevel(_hopLevel, _hopStreak, fb);
    bool dup = seenOrAdd(f.src, f.seq);
    bool forMe = (f.dst == _cfg.id), bcast = (f.dst == BROADCAST);

    if (f.type == MsgType::ACK && forMe) {
        if (_wait.active && f.plen >= 1 && f.payload[0] == _wait.seq && f.src == _wait.dst) { _wait.ack = true; _wait.fb = f.lhs; _st.acksRx++; }
        return;
    }
    if (f.type == MsgType::DATA && (forMe || bcast)) {
        if (forMe && (f.flags & FLAG_ACK_REQ)) sendAck(f, fb);                // re-ACK duplicates too
        if (dup) _st.rxDup++; else if (_cb) _cb(_cbCtx, f.src, f.payload, f.plen, fb);
        if (!forMe && !dup) queueRelay(f, fb);
        return;
    }
    if (!dup) queueRelay(f, fb);
}

void Node::sendAck(const Frame& rx, uint8_t fb) {
    Frame a;
    a.src = _cfg.id; a.dst = rx.src; a.seq = _seq++; a.ttl = _cfg.defaultTtl;
    a.type = MsgType::ACK; a.lhs = fb; a.plen = 1; a.payload[0] = rx.seq;
    seenOrAdd(a.src, a.seq);
    if (transmit(a, levelFor(peer(rx.src))) == Result::OK) _st.acksTx++;
}

void Node::sendNack(uint8_t fb) {
    uint32_t now = _p.millis();
    if (_nackEver && (uint32_t)(now - _lastNack) < _cfg.nackHoldoffMs) return;
    _lastNack = now; _nackEver = true;
    Frame a;
    a.src = _cfg.id; a.dst = BROADCAST; a.seq = _seq++; a.ttl = 0;
    a.type = MsgType::NACK; a.lhs = fb; a.plen = 0;
    if (transmit(a, MAX_LEVEL) == Result::OK) _st.nacksTx++;
}

// ------------------------------------------------------------------ relay (store & forward)
void Node::queueRelay(Frame f, uint8_t fb) {
    if (f.ttl == 0) return;
    for (auto& s : _rq) if (!s.used) {
        f.ttl--; f.lhs = fb & 0x0F; s.f = f; s.used = true;
        s.due = _p.millis() + _cfg.relayJitterMinMs + _p.random((uint32_t)(_cfg.relayJitterMaxMs - _cfg.relayJitterMinMs + 1));
        return;
    }
    _st.relayDropped++;
}

void Node::pumpRelay() {
    for (auto& s : _rq) {
        if (!s.used || (int32_t)(_p.millis() - s.due) < 0) continue;
        Peer tmp; tmp.level = _hopLevel;
        if (transmit(s.f, levelFor(tmp)) == Result::OK) _st.relayed++; else _st.relayDropped++;
        s.used = false;
    }
}

} // namespace afec
