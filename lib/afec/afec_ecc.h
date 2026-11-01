#pragma once
// AFEC - ECC layer: Hamming SECDED, BCH over GF(2^9), Reed-Solomon over GF(2^8).
// Systematic, shortened codes: [data bytes][parity bytes], parity size fixed per level.
#include <stdint.h>
#include <stddef.h>

namespace afec {

enum class Scheme : uint8_t { HAMMING, BCH, RS };

// Protection ladder, weakest -> strongest. Level index travels in the outer frame header.
//   0 Hamming SECDED  t=1 bit      parity 10 bit  -> 2 B
//   1 BCH(t=2)        t=2 bits     parity 18 bit  -> 3 B
//   2 BCH(t=5)        t=5 bits     parity 45 bit  -> 6 B
//   3 RS(t=2)         t=2 bytes    parity  4 B
//   4 RS(t=4)         t=4 bytes    parity  8 B
//   5 RS(t=8)         t=8 bytes    parity 16 B
constexpr uint8_t NUM_LEVELS = 6;
constexpr uint8_t MAX_LEVEL  = NUM_LEVELS - 1;

struct LevelInfo {
    Scheme  scheme;
    uint8_t t;            // correction capacity (bits for HAMMING/BCH, bytes for RS)
    uint8_t parityBytes;  // bytes appended after the data
};

const LevelInfo& levelInfo(uint8_t level);          // clamps to MAX_LEVEL
const char*      levelName(uint8_t level);

// Build the GF tables (idempotent, called lazily too).
void eccInit();

// parity must hold levelInfo(level).parityBytes bytes.
// Limits: Hamming len<=62, BCH len<=58, RS len+parity<=255.
bool eccEncode(uint8_t level, const uint8_t* data, uint8_t len, uint8_t* parity);

// Corrects `data` in place. true = codeword decoded (errors fixed, count in *corrected).
// false = uncorrectable (data left untouched).
bool eccDecode(uint8_t level, uint8_t* data, uint8_t len, const uint8_t* parity, uint8_t* corrected);

uint16_t crc16(const uint8_t* d, size_t n);         // CRC-16/CCITT-FALSE

} // namespace afec
