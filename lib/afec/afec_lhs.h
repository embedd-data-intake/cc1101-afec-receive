#pragma once
// AFEC - Link Health Score (LHS), Eq.(1)/(2) of the paper, evaluated with integer LUT lookup.
//   Score = 0.6*Health_ECC + 0.3*SNR_norm + 0.1*LQI_norm,   Health = 1 - s/t
// Each input is quantised to 3 bits; the 8x8x8 LUT (512 B) is filled once with integer
// arithmetic, then every score is O(1): no float, no division on the hot path except the
// two quantisers.
#include <stdint.h>

namespace afec {

struct LhsParams {
    int16_t rssiMinDbm = -110;   // ~ CC1101 sensitivity floor      -> SNR_norm = 0
    int16_t rssiMaxDbm = -50;    // strong, comfortable link        -> SNR_norm = 1
    uint8_t lqiWorst   = 80;     // CC1101 LQI (0..127, lower=better) treated as LQI_norm = 0
};

// Table 1 actions
enum class Action : uint8_t { DOWNGRADE, HOLD, UPGRADE, NACK_UPGRADE, MAX_PROTECT };

class Lhs {
public:
    void begin(const LhsParams& p = LhsParams());

    // corrected/t : s and t of the decoder (ignored if decodeFailed)
    // Returns 0x1..0xF.  0x0 is never produced here: it is reserved for "ACK never came".
    //  - decoder failure -> clamped to 0x1..0x4  (NACK zone)
    //  - decoded OK      -> clamped to 0x5..0xF
    //  - 0xF only when s == 0 ("perfect link"): an error-free decode scoring >= 0xE is promoted to 0xF
    uint8_t score(uint8_t corrected, uint8_t t, int16_t rssiDbm, uint8_t lqi, bool decodeFailed) const;

    static Action decide(uint8_t score);

private:
    LhsParams _p;
    uint8_t   _lut[512];
};

} // namespace afec
