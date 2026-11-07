#include "afec_frame.h"
#include <string.h>

namespace afec {

namespace {
inline uint8_t par8(uint8_t v) { v ^= v >> 4; v ^= v >> 2; v ^= v >> 1; return v & 1; }

uint8_t h84enc(uint8_t n) {
    uint8_t d1 = n & 1, d2 = (n >> 1) & 1, d3 = (n >> 2) & 1, d4 = (n >> 3) & 1;
    uint8_t p1 = d1 ^ d2 ^ d4, p2 = d1 ^ d3 ^ d4, p3 = d2 ^ d3 ^ d4;
    uint8_t b = p1 | (p2 << 1) | (d1 << 2) | (p3 << 3) | (d2 << 4) | (d3 << 5) | (d4 << 6);
    return b | (par8(b) << 7);
}
bool h84dec(uint8_t b, uint8_t* n) {
    uint8_t s1 = ((b >> 0) ^ (b >> 2) ^ (b >> 4) ^ (b >> 6)) & 1;
    uint8_t s2 = ((b >> 1) ^ (b >> 2) ^ (b >> 5) ^ (b >> 6)) & 1;
    uint8_t s3 = ((b >> 3) ^ (b >> 4) ^ (b >> 5) ^ (b >> 6)) & 1;
    uint8_t syn = s1 | (s2 << 1) | (s3 << 2);
    uint8_t odd = par8(b);
    if (syn) { if (!odd) return false; b ^= (uint8_t)(1 << (syn - 1)); }
    *n = ((b >> 2) & 1) | (((b >> 4) & 1) << 1) | (((b >> 5) & 1) << 2) | (((b >> 6) & 1) << 3);
    return true;
}
} // namespace

uint8_t parityBytesOf(uint8_t level) { return level == LEVEL_NONE ? 0 : levelInfo(level).parityBytes; }

uint8_t maxPayload(uint8_t level) {
    return AIR_MAX - OUTER_LEN - parityBytesOf(level) - INNER_OVERHEAD;
}

uint8_t fitLevel(uint8_t level, uint8_t plen) {
    if (level == LEVEL_NONE) return level;
    if (level > MAX_LEVEL) level = MAX_LEVEL;
    while (level > 0 && plen > maxPayload(level)) level--;
    return level;
}

bool encodeFrame(const Frame& f, uint8_t level, uint8_t* out, uint8_t* outLen) {
    if ((level > MAX_LEVEL && level != LEVEL_NONE) || f.plen > maxPayload(level)) return false;
    const uint8_t pb = parityBytesOf(level);
    const uint8_t L  = INNER_OVERHEAD + f.plen;

    uint8_t info = 0xA0 | level;
    out[0] = h84enc(info & 0x0F);
    out[1] = h84enc(info >> 4);

    uint8_t* in = out + OUTER_LEN;
    in[0] = f.src; in[1] = f.dst; in[2] = f.seq;
    in[3] = (uint8_t)((f.ttl & 0x0F) << 4) | (f.lhs & 0x0F);
    in[4] = (uint8_t)(((uint8_t)f.type & 0x03) << 6) | (f.flags & 0x3F);
    memcpy(in + INNER_HDR, f.payload, f.plen);
    uint16_t c = crc16(in, INNER_HDR + f.plen);
    in[L - 2] = c & 0xFF; in[L - 1] = c >> 8;

    if (level != LEVEL_NONE && !eccEncode(level, in, L, in + L)) return false;
    *outLen = OUTER_LEN + L + pb;
    return true;
}

DecodeStatus decodeFrame(const uint8_t* in, uint8_t len, Frame& f, DecodeInfo& info) {
    if (len < OUTER_LEN + INNER_OVERHEAD) return DecodeStatus::TOO_SHORT;
    uint8_t lo, hi;
    if (!h84dec(in[0], &lo) || !h84dec(in[1], &hi)) return DecodeStatus::BAD_HEADER;
    if (hi != 0x0A || (lo >= NUM_LEVELS && lo != LEVEL_NONE)) return DecodeStatus::BAD_HEADER;

    info.level = lo; info.corrected = 0; info.t = (lo == LEVEL_NONE) ? 0 : levelInfo(lo).t;
    const uint8_t pb = parityBytesOf(lo);
    if (len < OUTER_LEN + pb + INNER_OVERHEAD) return DecodeStatus::TOO_SHORT;
    const uint8_t L = len - OUTER_LEN - pb;

    uint8_t buf[AIR_MAX];
    memcpy(buf, in + OUTER_LEN, L);
    uint8_t corr = 0;
    if (lo != LEVEL_NONE && !eccDecode(lo, buf, L, in + OUTER_LEN + L, &corr)) { info.corrected = info.t; return DecodeStatus::ECC_FAIL; }
    info.corrected = corr;

    uint16_t c = crc16(buf, L - 2);
    if (buf[L - 2] != (c & 0xFF) || buf[L - 1] != (c >> 8)) { info.corrected = info.t; return DecodeStatus::CRC_FAIL; }

    f.src = buf[0]; f.dst = buf[1]; f.seq = buf[2];
    f.ttl = buf[3] >> 4; f.lhs = buf[3] & 0x0F;
    f.type = (MsgType)(buf[4] >> 6); f.flags = buf[4] & 0x3F;
    f.plen = L - INNER_OVERHEAD;
    memcpy(f.payload, buf + INNER_HDR, f.plen);
    return DecodeStatus::OK;
}

} // namespace afec
