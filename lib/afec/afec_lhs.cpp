#include "afec_lhs.h"

namespace afec {

void Lhs::begin(const LhsParams& p) {
    _p = p;
    for (int h = 0; h < 8; h++)
        for (int s = 0; s < 8; s++)
            for (int l = 0; l < 8; l++) {
                uint16_t num = 60 * h + 30 * s + 10 * l;          // 0..700  (= 100 * 7 * score)
                _lut[(h << 6) | (s << 3) | l] = (uint8_t)((num * 15 + 350) / 700);
            }
}

uint8_t Lhs::score(uint8_t corrected, uint8_t t, int16_t rssi, uint8_t lqi, bool failed) const {
    uint8_t h = 0;
    if (!failed) {
        uint16_t sq = t ? (uint16_t)((corrected * 7 + t / 2) / t) : 7;   // stress s/t -> 0..7
        if (sq > 7) sq = 7;
        h = (uint8_t)(7 - sq);                                           // health = 1 - stress
    }
    int16_t r = rssi;
    if (r < _p.rssiMinDbm) r = _p.rssiMinDbm;
    if (r > _p.rssiMaxDbm) r = _p.rssiMaxDbm;
    uint8_t sn = (uint8_t)(((int32_t)(r - _p.rssiMinDbm) * 7) / (_p.rssiMaxDbm - _p.rssiMinDbm));
    uint8_t l  = lqi > _p.lqiWorst ? _p.lqiWorst : lqi;
    uint8_t li = (uint8_t)(7 - (l * 7 + _p.lqiWorst / 2) / _p.lqiWorst);

    uint8_t v = _lut[(h << 6) | (sn << 3) | li];
    if (failed) { if (v > 4) v = 4; if (v < 1) v = 1; }
    else {
        if (v < 5) v = 5;
        if (corrected > 0 && v > 14) v = 14;      // 0xF only for an error-free decode
        if (corrected == 0 && v >= 14) v = 15;    // Table 1: 0xF = perfect link (s = 0)
    }
    return v;
}

Action Lhs::decide(uint8_t s) {
    if (s == 0xF)  return Action::DOWNGRADE;
    if (s >= 0xA)  return Action::HOLD;
    if (s >= 0x5)  return Action::UPGRADE;
    if (s >= 0x1)  return Action::NACK_UPGRADE;
    return Action::MAX_PROTECT;
}

} // namespace afec
