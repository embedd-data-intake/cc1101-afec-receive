#pragma once
// Serial command layer for the measurement campaign (run plan P0.1). Firmware-only: the library
// core is unchanged. Keep this file identical in the sender and the receiver.
//
// Commands are single lines at 115200 baud: "<name> key=value key=value ...".
// Every reply is a JSON line in the library's log format, so the parser treats it as an event:
//   {"t":<millis>,"node":<id>,"lv":"I|W|E","ev":"<event>",...fields}
// Errors are "cmd_err" events; nothing is changed when a command is rejected.
#include <Arduino.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <afec_node.h>
#include "afec_log_sink.h"

namespace cmd {

inline uint8_t& nodeId() { static uint8_t id = 0; return id; }

// One JSON line to the log sink. `fields` starts with a comma, e.g. ",\"x\":%u". Values must not
// need escaping. A no-op when the sink is off (AFEC_LOG_SINK=0).
inline void ev(char lv, const char* name, const char* fields = "", ...) {
    if (!logsink::enabled()) return;
    char body[256];
    va_list ap;
    va_start(ap, fields);
    int n = vsnprintf(body, sizeof body, fields, ap);
    va_end(ap);
    if (n < 0) body[0] = 0;
    char line[320];
    n = snprintf(line, sizeof line, "{\"t\":%lu,\"node\":%u,\"lv\":\"%c\",\"ev\":\"%s\"%s}\n",
                 (unsigned long)millis(), (unsigned)nodeId(), lv, name, body);
    if (n < 0) return;
    if (n >= (int)sizeof line) { line[sizeof line - 2] = '\n'; n = sizeof line - 1; }
    logsink::write(line, (size_t)n);
}

// Copy user text into a JSON-safe string (no quotes, backslashes or control chars).
inline const char* safe(const char* s, char* out, size_t n) {
    size_t i = 0;
    for (; s && *s && i + 1 < n; s++) out[i++] = (*s == '"' || *s == '\\' || *s < 32 || *s > 126) ? '?' : *s;
    out[i] = 0;
    return out;
}

inline bool fail(const char* cmdName, const char* err, const char* detail = "") {
    char c[24], d[48];
    ev('W', "cmd_err", ",\"cmd\":\"%s\",\"err\":\"%s\",\"detail\":\"%s\"",
       safe(cmdName, c, sizeof c), err, safe(detail, d, sizeof d));
    return false;
}

// ------------------------------------------------------------------ line input
class LineReader {
public:
    // Returns a complete, NUL-terminated line, or nullptr. Valid until the next call.
    char* poll() {
        while (Serial.available()) {
            int c = Serial.read();
            if (c == '\r' || c == '\n') {
                if (_overflow) { _n = 0; _overflow = false; fail("?", "line too long"); continue; }
                if (!_n) continue;
                _buf[_n] = 0; _n = 0;
                return _buf;
            }
            if (_n < sizeof(_buf) - 1) _buf[_n++] = (char)c;
            else _overflow = true;
        }
        return nullptr;
    }
private:
    char _buf[192];
    uint8_t _n = 0;
    bool _overflow = false;
};

// ------------------------------------------------------------------ commands over MQTT
// Removes a trailing-style ` id=<n>` token from `line` in place. true if one was found.
inline bool takeId(char* line, uint32_t& id) {
    for (char* p = line; (p = strstr(p, "id=")) != nullptr; p += 3) {
        if (p == line || (p[-1] != ' ' && p[-1] != '\t')) continue;      // part of another key
        char* end = nullptr;
        const unsigned long v = strtoul(p + 3, &end, 10);
        if (end == p + 3 || (*end && *end != ' ' && *end != '\t')) return false;
        id = (uint32_t)v;
        memmove(p - 1, end, strlen(end) + 1);
        return true;
    }
    return false;
}

// Next command from afec/cmd/<role> (MQTT builds), or nullptr. Logs `cmd_rx` (id, cmd) for every
// command it hands out. A command whose `id=` was among the last 8 is a QoS 1 redelivery: it is
// logged as `cmd_dup` and not run again (a second `cfg` would re-initialise the radio).
inline char* pollRemote() {
    char* line = logsink::popCommand();
    if (!line) return nullptr;
    static uint32_t seen[8]; static uint8_t nSeen = 0, next = 0;
    uint32_t id = 0;
    const bool hasId = takeId(line, id);
    char name[24];
    size_t k = strcspn(line, " \t");
    if (k >= sizeof name) k = sizeof name - 1;
    memcpy(name, line, k); name[k] = 0;
    if (hasId) {
        for (uint8_t i = 0; i < nSeen; i++)
            if (seen[i] == id) {
                ev('W', "cmd_dup", ",\"id\":%lu,\"cmd\":\"%s\"", (unsigned long)id, safe(name, name, sizeof name));
                return nullptr;
            }
        seen[next] = id; next = (next + 1) % 8; if (nSeen < 8) nSeen++;
    }
    ev('I', "cmd_rx", ",\"id\":%lu,\"cmd\":\"%s\",\"src\":\"mqtt\"", (unsigned long)id, safe(name, name, sizeof name));
    return line;
}

// ------------------------------------------------------------------ arguments
struct Args {
    static constexpr uint8_t MAX = 14;
    const char* name = "";
    const char* key[MAX];
    const char* val[MAX];
    uint8_t n = 0;

    // Tokenises `line` in place. false (and logs) on a token without '=' or too many tokens.
    bool parse(char* line) {
        char* save = nullptr;
        char* tok = strtok_r(line, " \t", &save);
        if (!tok) return false;
        name = tok;
        while ((tok = strtok_r(nullptr, " \t", &save))) {
            char* eq = strchr(tok, '=');
            if (!eq || eq == tok) return fail(name, "expected key=value", tok);
            if (n >= MAX) return fail(name, "too many arguments");
            *eq = 0; key[n] = tok; val[n] = eq + 1; n++;
        }
        return true;
    }

    const char* get(const char* k) const {
        for (uint8_t i = 0; i < n; i++) if (!strcmp(key[i], k)) return val[i];
        return nullptr;
    }
    bool has(const char* k) const { return get(k) != nullptr; }

    // Rejects keys not in the nullptr-terminated `allowed` list.
    bool only(const char* const* allowed) const {
        for (uint8_t i = 0; i < n; i++) {
            bool ok = false;
            for (const char* const* a = allowed; *a; a++) if (!strcmp(key[i], *a)) { ok = true; break; }
            if (!ok) return fail(name, "unknown key", key[i]);
        }
        return true;
    }

    // Integer in [lo,hi]. Absent: true, `out` untouched. Malformed or out of range: false (logged).
    template <typename T>
    bool num(const char* k, long lo, long hi, T& out) const {
        const char* v = get(k);
        if (!v) return true;
        char* end = nullptr;
        long x = strtol(v, &end, 10);
        if (!*v || *end || x < lo || x > hi) {
            char msg[40]; snprintf(msg, sizeof msg, "%s must be %ld..%ld", k, lo, hi);
            return fail(name, "bad value", msg);
        }
        out = (T)x;
        return true;
    }
};

// ------------------------------------------------------------------ cfg
inline const char* modeName(afec::Mode m) {
    static const char* const names[] = { "C1", "C2", "C3", "C4", "C5" };
    return names[(uint8_t)m < 5 ? (uint8_t)m : 4];
}

// cfg mode=<1..5> init=<0..5> retries=<n> ackto=<ms> tx=<dBm> dg=<n> nackstep=<n>
//     rssi_min=<dBm> rssi_max=<dBm> rssi_up=<a,b,c,d,e> hys=<n>
// Absent keys keep their current value, so the C4 policy can be set once and stays frozen.
// All-or-nothing: on any error `nc` and `txDbm` are left unchanged.
inline bool parseCfg(const Args& a, afec::NodeConfig& nc, int8_t& txDbm) {
    static const char* const keys[] = { "mode", "init", "retries", "ackto", "tx", "dg", "nackstep",
                                        "rssi_min", "rssi_max", "rssi_up", "hys", nullptr };
    if (!a.only(keys)) return false;
    afec::NodeConfig c = nc;
    int8_t tx = txDbm;
    uint8_t mode = (uint8_t)c.mode + 1;
    if (!a.num("mode", 1, 5, mode)) return false;
    if (mode == 3) return fail(a.name, "C3 excluded", "needs the fixed-length FEC driver patch");
    c.mode = (afec::Mode)(mode - 1);
    if (!a.num("init", 0, afec::MAX_LEVEL, c.initialLevel)) return false;
    if (!a.num("retries", 0, 15, c.maxRetries)) return false;
    if (!a.num("ackto", 50, 5000, c.ackTimeoutMs)) return false;
    if (!a.num("tx", -30, 10, tx)) return false;
    if (!a.num("dg", 1, 15, c.downgradeAfter)) return false;
    if (!a.num("nackstep", 1, afec::MAX_LEVEL, c.nackStep)) return false;
    if (!a.num("rssi_min", -140, 0, c.rssi.minDbm)) return false;
    if (!a.num("rssi_max", -140, 0, c.rssi.maxDbm)) return false;
    if (!a.num("hys", 0, 15, c.rssi.hys)) return false;
    if (c.rssi.minDbm >= c.rssi.maxDbm) return fail(a.name, "bad value", "rssi_min must be < rssi_max");
    if (const char* up = a.get("rssi_up")) {
        uint8_t v[afec::MAX_LEVEL];
        const char* p = up;
        for (uint8_t i = 0; i < afec::MAX_LEVEL; i++) {
            char* end = nullptr;
            long x = strtol(p, &end, 10);
            const bool last = (i == afec::MAX_LEVEL - 1);
            if (end == p || x < 0 || x > 15 || (last ? *end != 0 : *end != ','))
                return fail(a.name, "bad value", "rssi_up must be 5 comma-separated values 0..15");
            v[i] = (uint8_t)x;
            p = end + 1;
        }
        memcpy(c.rssi.up, v, sizeof v);
    }
    nc = c;
    txDbm = tx;
    return true;
}

inline void printCfg(const afec::NodeConfig& c, int8_t txDbm) {
    ev('I', "cfg_ok",
       ",\"cfg\":\"%s\",\"mode\":%u,\"init_lvl\":%u,\"retries\":%u,\"ack_to_ms\":%u,\"tx_dbm\":%d,"
       "\"dg\":%u,\"nack_step\":%u,\"rssi_min\":%d,\"rssi_max\":%d,\"rssi_up\":\"%u,%u,%u,%u,%u\",\"hys\":%u",
       modeName(c.mode), (unsigned)c.mode, c.initialLevel, c.maxRetries, (unsigned)c.ackTimeoutMs, txDbm,
       c.downgradeAfter, c.nackStep, c.rssi.minDbm, c.rssi.maxDbm,
       c.rssi.up[0], c.rssi.up[1], c.rssi.up[2], c.rssi.up[3], c.rssi.up[4], c.rssi.hys);
}

inline void printStats(const afec::Stats& s) {
    ev('I', "stats",
       ",\"tx_frames\":%lu,\"tx_retries\":%lu,\"acks_rx\":%lu,\"nacks_rx\":%lu,\"timeouts\":%lu,"
       "\"rx_ok\":%lu,\"rx_dup\":%lu,\"rx_bad_header\":%lu,\"rx_ecc_fail\":%lu,\"rx_crc_fail\":%lu,"
       "\"acks_tx\":%lu,\"nacks_tx\":%lu,\"relayed\":%lu,\"relay_dropped\":%lu,\"duty_blocked\":%lu,\"ecc_corrected\":%lu",
       (unsigned long)s.txFrames, (unsigned long)s.txRetries, (unsigned long)s.acksRx, (unsigned long)s.nacksRx,
       (unsigned long)s.timeouts, (unsigned long)s.rxOk, (unsigned long)s.rxDup, (unsigned long)s.rxBadHeader,
       (unsigned long)s.rxEccFail, (unsigned long)s.rxCrcFail, (unsigned long)s.acksTx, (unsigned long)s.nacksTx,
       (unsigned long)s.relayed, (unsigned long)s.relayDropped, (unsigned long)s.dutyBlocked,
       (unsigned long)s.eccCorrected);
#if AFEC_LOG_SINK & AFEC_SINK_MQTT
    // Check `pub` against the number of messages the broker side received for this node.
    const logsink::Mqtt& m = logsink::mq();
    ev('I', "log_stats", ",\"sink\":\"%s\",\"enq\":%lu,\"dropped\":%lu,\"pub\":%lu,\"retries\":%lu,\"conns\":%lu,\"connected\":%u,"
       "\"cmd_rx\":%lu,\"cmd_lost\":%lu",
       logsink::name(), (unsigned long)m.enq, (unsigned long)m.dropped, (unsigned long)m.pub,
       (unsigned long)m.retries, (unsigned long)m.conns, (unsigned)m.connected,
       (unsigned long)m.cmdRx, (unsigned long)m.cmdLost);
#endif
}

// mark run=<id> block=<n> cfg=<Cx>: a block marker for splitting the logs. Returns the block id.
inline bool mark(const Args& a, uint16_t& block) {
    static const char* const keys[] = { "run", "block", "cfg", nullptr };
    if (!a.only(keys)) return false;
    if (!a.has("block")) return fail(a.name, "missing key", "block");
    uint16_t b = 0;
    if (!a.num("block", 0, 65535, b)) return false;
    char run[40], cfg[12];
    safe(a.get("run"), run, sizeof run);
    logsink::setRun(run);                        // the mark itself already goes to the new run's topic
    ev('I', "mark", ",\"run\":\"%s\",\"block\":%u,\"cfg\":\"%s\"",
       run, b, safe(a.get("cfg"), cfg, sizeof cfg));
    block = b;
    return true;
}

// ------------------------------------------------------------------ synthetic payload
// [0..3] aseq (u32 LE, per-boot app counter)  [4..5] block (u16 LE)  [6..7] idx in block (u16 LE)
// [8..len-5] filler (xorshift32 seeded from aseq)  [len-4..len-1] CRC32/IEEE over bytes 0..len-5
constexpr uint8_t SYN_MIN = 12;

inline uint32_t crc32(const uint8_t* d, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *d++;
        for (uint8_t k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

inline void put16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
inline void put32(uint8_t* p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }
inline uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t get32(const uint8_t* p) { return get16(p) | ((uint32_t)get16(p + 2) << 16); }

inline void synBuild(uint8_t* p, uint8_t len, uint32_t aseq, uint16_t block, uint16_t idx) {
    put32(p, aseq); put16(p + 4, block); put16(p + 6, idx);
    uint32_t x = (aseq * 2654435761u) | 1u;
    for (uint8_t i = 8; i < len - 4; i++) { x ^= x << 13; x ^= x >> 17; x ^= x << 5; p[i] = (uint8_t)x; }
    put32(p + len - 4, crc32(p, len - 4));
}

// true = CRC32 matches. The header fields are filled in either way (untrustworthy when false).
inline bool synCheck(const uint8_t* p, uint8_t len, uint32_t& aseq, uint16_t& block, uint16_t& idx) {
    aseq = 0; block = 0; idx = 0;
    if (len < SYN_MIN) return false;
    aseq = get32(p); block = get16(p + 4); idx = get16(p + 6);
    return get32(p + len - 4) == crc32(p, len - 4);
}

} // namespace cmd
