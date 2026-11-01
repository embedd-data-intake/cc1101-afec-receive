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

// ------------------------------------------------------------------ peers / adaptation
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

uint8_t Node::peerLevel(uint8_t id) { return peer(id).level; }

// Table 1
void Node::applyScore(Peer& p, uint8_t score) {
    switch (Lhs::decide(score)) {
        case Action::DOWNGRADE:
            if (++p.streak >= _cfg.downgradeAfter) { if (p.level > 0) p.level--; p.streak = 0; }
            return;
        case Action::HOLD:         p.streak = 0; return;
        case Action::UPGRADE:      p.streak = 0; if (p.level < MAX_LEVEL) p.level++; return;
        case Action::NACK_UPGRADE: p.streak = 0; p.level = (p.level + _cfg.nackStep > MAX_LEVEL) ? MAX_LEVEL : p.level + _cfg.nackStep; return;
        case Action::MAX_PROTECT:  p.streak = 0; p.level = MAX_LEVEL; return;
    }
}

// Level used when re-encoding relayed frames follows the *local* LHS (hop-by-hop).
void Node::applyHop(uint8_t score) {
    Peer tmp; tmp.level = _hopLevel; tmp.streak = _hopStreak;
    applyScore(tmp, score);
    _hopLevel = tmp.level; _hopStreak = tmp.streak;
}

bool Node::seenOrAdd(uint8_t src, uint8_t seq) {
    for (auto& s : _seen) if (s.used && s.src == src && s.seq == seq) return true;
    _seen[_seenHead] = { src, seq, true };
    _seenHead = (_seenHead + 1) % NSEEN;
    return false;
}

// ------------------------------------------------------------------ duty cycle / airtime
uint32_t Node::airtimeUs(uint8_t bytes) const {
    // preamble(4) + sync(4) + length(1) + frame, 8 bit each
    return (uint32_t)(bytes + 9) * 8UL * 1000000UL / _cfg.bitrateBps;
}

void Node::refillDuty() {
    uint32_t now = _p.millis();
    uint32_t dt = now - _lastRefill; _lastRefill = now;
    _budgetUs += (int64_t)dt * 10 * _cfg.dutyPercent;        // dt[ms]*1000us*duty/100
    int64_t cap = (int64_t)_cfg.dutyBurstMs * 1000;
    if (_budgetUs > cap) _budgetUs = cap;
}

// ------------------------------------------------------------------ TX path
Result Node::transmit(const Frame& f, uint8_t level, bool csma) {
    uint8_t buf[AIR_MAX], n = 0;
    level = fitLevel(level, f.plen);
    if (!encodeFrame(f, level, buf, &n)) return Result::TOO_LONG;

    refillDuty();
    uint32_t at = airtimeUs(n);
    if (_budgetUs < (int64_t)at) { _st.dutyBlocked++; return Result::DUTY_LIMIT; }

    if (csma && _p.channelBusy) {                       // listen-before-talk, binary-exp. backoff
        for (uint8_t a = 0; a < 6 && _p.channelBusy(); a++) {
            uint32_t wait = 2 + _p.random(4UL << a), t0 = _p.millis();
            while (_p.millis() - t0 < wait) if (_p.yield) _p.yield();
        }
    }
    if (!_r->send(buf, n)) return Result::RADIO_ERR;
    _budgetUs -= at; _st.txFrames++;
    return Result::OK;
}

Result Node::send(uint8_t dst, const uint8_t* payload, uint8_t len, bool reliable) {
    if (dst == BROADCAST) reliable = false;
    if (len > maxPayload(0) || (len && !payload)) return Result::TOO_LONG;

    Frame f;
    f.src = _cfg.id; f.dst = dst; f.seq = _seq++; f.ttl = _cfg.defaultTtl;
    f.type = MsgType::DATA; f.flags = reliable ? FLAG_ACK_REQ : 0;
    f.plen = len; if (len) memcpy(f.payload, payload, len);
    seenOrAdd(f.src, f.seq);                            // never relay our own frame back

    const uint8_t tries = reliable ? 1 + _cfg.maxRetries : 1;
    for (uint8_t a = 0; a < tries; a++) {
        Peer& pr = peer(dst);
        f.lhs = pr.rxScore;                             // our view of the reverse link
        if (a) _st.txRetries++;

        if (reliable) { _wait = Wait(); _wait.active = true; _wait.dst = dst; _wait.seq = f.seq; }
        Result r = transmit(f, pr.level);
        if (r != Result::OK) { _wait.active = false; return r; }
        if (!reliable) return Result::OK;

        uint32_t t0 = _p.millis();
        while (!_wait.ack && !_wait.nack && (uint32_t)(_p.millis() - t0) < _cfg.ackTimeoutMs) {
            processRx();
            if (_p.yield) _p.yield();
        }
        _wait.active = false;
        Peer& after = peer(dst);
        if (_wait.ack)  { applyScore(after, _wait.score); return Result::OK; }
        if (_wait.nack) { applyScore(after, _wait.score); continue; }          // retransmit at once
        _st.timeouts++;                                                        // score 0x0
        applyScore(after, 0);
    }
    return Result::NO_ACK;
}

// ------------------------------------------------------------------ RX path
void Node::poll() { processRx(); pumpRelay(); }

void Node::processRx() {
    uint8_t buf[AIR_MAX]; CC1101::RxInfo ri;
    for (uint8_t guard = 0; guard < 4; guard++) {       // drain a few, never starve the caller
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
        case DecodeStatus::BAD_HEADER: _st.rxBadHeader++; return;       // silent: sender times out -> 0x0
        case DecodeStatus::ECC_FAIL:   _st.rxEccFail++;  sendNack(_lhs.score(0, di.t, ri.rssiDbm, ri.lqi, true)); return;
        case DecodeStatus::CRC_FAIL:   _st.rxCrcFail++;  sendNack(_lhs.score(0, di.t, ri.rssiDbm, ri.lqi, true)); return;
    }
    _st.rxOk++; _st.eccCorrected += di.corrected;
    uint8_t score = _lhs.score(di.corrected, di.t, ri.rssiDbm, ri.lqi, false);

    if (f.type == MsgType::NACK) {                                      // link-local, never relayed
        if (_wait.active) { _wait.nack = true; _wait.score = f.lhs; _st.nacksRx++; }
        return;
    }
    if (f.src == _cfg.id) return;
    peer(f.src).rxScore = score;
    applyHop(score);
    bool dup = seenOrAdd(f.src, f.seq);

    bool forMe = (f.dst == _cfg.id), bcast = (f.dst == BROADCAST);

    if (f.type == MsgType::ACK && forMe) {
        if (_wait.active && f.plen >= 1 && f.payload[0] == _wait.seq && f.src == _wait.dst) {
            _wait.ack = true; _wait.score = f.lhs; _st.acksRx++;
        }
        return;
    }
    if (f.type == MsgType::DATA && (forMe || bcast)) {
        if (dup) _st.rxDup++;
        else if (_cb) _cb(_cbCtx, f.src, f.payload, f.plen, score);
        if (forMe && (f.flags & FLAG_ACK_REQ)) sendAck(f, score);       // re-ACK duplicates too
        if (!forMe && !dup) queueRelay(f, score);                       // broadcast floods onward
        return;
    }
    if (!dup) queueRelay(f, score);                                     // not ours -> store & forward
}

void Node::sendAck(const Frame& rx, uint8_t score) {
    Frame a;
    a.src = _cfg.id; a.dst = rx.src; a.seq = _seq++; a.ttl = _cfg.defaultTtl;
    a.type = MsgType::ACK; a.lhs = score; a.plen = 1; a.payload[0] = rx.seq;
    seenOrAdd(a.src, a.seq);
    if (transmit(a, peer(rx.src).level) == Result::OK) _st.acksTx++;
}

void Node::sendNack(uint8_t score) {
    uint32_t now = _p.millis();
    if (_nackEver && (uint32_t)(now - _lastNack) < _cfg.nackHoldoffMs) return;
    _lastNack = now; _nackEver = true;
    Frame a;                                            // sender of the damaged frame is unknowable
    a.src = _cfg.id; a.dst = BROADCAST; a.seq = _seq++; a.ttl = 0;
    a.type = MsgType::NACK; a.lhs = score; a.plen = 0;
    if (transmit(a, MAX_LEVEL) == Result::OK) _st.nacksTx++;
}

// ------------------------------------------------------------------ relay (store & forward)
void Node::queueRelay(Frame f, uint8_t score) {
    if (f.ttl == 0) return;
    for (auto& s : _rq) if (!s.used) {
        f.ttl--; f.lhs = score;
        s.f = f; s.used = true;
        s.due = _p.millis() + _cfg.relayJitterMinMs +
                _p.random((uint32_t)(_cfg.relayJitterMaxMs - _cfg.relayJitterMinMs + 1));
        return;
    }
    _st.relayDropped++;
}

void Node::pumpRelay() {
    for (auto& s : _rq) {
        if (!s.used || (int32_t)(_p.millis() - s.due) < 0) continue;
        if (_p.channelBusy && _p.channelBusy()) {       // medium busy -> fresh jitter
            s.due = _p.millis() + _cfg.relayJitterMinMs + _p.random(_cfg.relayJitterMaxMs);
            continue;
        }
        Result r = transmit(s.f, _hopLevel, false);
        if (r == Result::OK) _st.relayed++; else _st.relayDropped++;
        s.used = false;
    }
}

} // namespace afec
