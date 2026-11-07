#pragma once
// AFEC - frame layer (pure, hardware independent).
//
// On-air frame (what CC1101::send() gets, <= CC1101::MAX_PAYLOAD = 61 bytes):
//
//   [O0 O1]  [ inner (L bytes) .......................... ]  [ parity (PB bytes) ]
//    outer     src dst seq ttl|lhs type|flags  payload  crc16
//
//  * Outer header = 1 info byte (0xA0 | ecc level) split in two nibbles, each protected
//    by Hamming(8,4) SECDED. It sits OUTSIDE the main ECC (the decoder must know the level
//    before it can decode) - this is the "2-byte header outside ECC" framing rule.
//  * Inner part is systematic-ECC protected at the level named in the outer header.
//    L = frameLen - 2 - PB(level), so no length field is needed.
//  * CRC-16 inside the protected part catches decoder miscorrection ("aliasing").
#include <stdint.h>
#include "afec_ecc.h"

namespace afec {

constexpr uint8_t AIR_MAX       = 61;   // == CC1101::MAX_PAYLOAD
constexpr uint8_t OUTER_LEN     = 2;
constexpr uint8_t INNER_HDR     = 5;
constexpr uint8_t CRC_LEN       = 2;
constexpr uint8_t INNER_OVERHEAD= INNER_HDR + CRC_LEN;
constexpr uint8_t BROADCAST     = 0xFF;
constexpr uint8_t LEVEL_NONE    = 7;     // CRC-16 only, no parity (C1, C2, C3, 'no ECC' sweep point)
constexpr uint8_t PAYLOAD_MAX   = AIR_MAX - OUTER_LEN - 2 /*min parity*/ - INNER_OVERHEAD; // 50

enum class MsgType : uint8_t { DATA = 0, ACK = 1, NACK = 2 };
constexpr uint8_t FLAG_ACK_REQ = 0x01;

struct Frame {
    uint8_t src = 0, dst = 0, seq = 0;
    uint8_t ttl = 0;                 // 0..15
    uint8_t lhs = 0;                 // 0x0..0xF   Link Health Score (feedback / local view)
    MsgType type = MsgType::DATA;
    uint8_t flags = 0;               // 6 bits
    uint8_t plen = 0;
    uint8_t payload[PAYLOAD_MAX];
};

enum class DecodeStatus : uint8_t { OK, TOO_SHORT, BAD_HEADER, ECC_FAIL, CRC_FAIL };

struct DecodeInfo {
    uint8_t level = 0;       // level named by the outer header (valid unless BAD_HEADER/TOO_SHORT)
    uint8_t corrected = 0;   // symbols/bits repaired by the ECC (s)
    uint8_t t = 0;           // capacity of that level (t)
};

uint8_t parityBytesOf(uint8_t level);                     // 0 for LEVEL_NONE
uint8_t maxPayload(uint8_t level);                        // largest payload that fits at `level`
uint8_t fitLevel(uint8_t level, uint8_t payloadLen);      // highest level <= `level` that fits

// out must hold AIR_MAX bytes. false if the payload does not fit at that level.
bool encodeFrame(const Frame& f, uint8_t level, uint8_t* out, uint8_t* outLen);
DecodeStatus decodeFrame(const uint8_t* in, uint8_t len, Frame& f, DecodeInfo& info);

} // namespace afec
