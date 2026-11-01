#include "afec_ecc.h"
#include <string.h>

namespace afec {

namespace {

// ---------------------------------------------------------------- Galois fields
template <int M>
struct GF {
    static const int N = (1 << M) - 1;
    uint16_t exp[2 * N];
    uint16_t log[N + 1];
    bool     ready = false;

    void init(uint16_t poly) {
        if (ready) return;
        uint16_t x = 1;
        for (int i = 0; i < N; i++) {
            exp[i] = x; log[x] = (uint16_t)i;
            x <<= 1;
            if (x & (1 << M)) x ^= poly;
        }
        for (int i = N; i < 2 * N; i++) exp[i] = exp[i - N];
        log[0] = 0;
        ready = true;
    }
    uint16_t mul(uint16_t a, uint16_t b) const { return (a && b) ? exp[log[a] + log[b]] : 0; }
    uint16_t div(uint16_t a, uint16_t b) const { return a ? exp[log[a] + N - log[b]] : 0; } // b != 0
    uint16_t pw(int e) const { e %= N; if (e < 0) e += N; return exp[e]; }       // alpha^e
};

GF<9> g9;   // x^9 + x^4 + 1
GF<8> g8;   // x^8 + x^4 + x^3 + x^2 + 1

constexpr int MAXS = 16;   // max syndromes (RS t=8)

// Berlekamp-Massey. S[0..n-1] -> locator C[0..n], returns L (degree).
template <class F>
int berlekampMassey(const F& f, const uint16_t* S, int n, uint16_t* C) {
    uint16_t B[MAXS + 2], T[MAXS + 2];
    for (int i = 0; i <= n; i++) { C[i] = 0; B[i] = 0; }
    C[0] = B[0] = 1;
    int L = 0, m = 1; uint16_t b = 1;
    for (int k = 0; k < n; k++) {
        uint16_t d = S[k];
        for (int i = 1; i <= L; i++) d ^= f.mul(C[i], S[k - i]);
        if (!d) { m++; continue; }
        uint16_t coef = f.div(d, b);
        if (2 * L <= k) {
            memcpy(T, C, sizeof(uint16_t) * (n + 1));
            for (int i = 0; i + m <= n; i++) C[i + m] ^= f.mul(coef, B[i]);
            memcpy(B, T, sizeof(uint16_t) * (n + 1));
            L = k + 1 - L; b = d; m = 1;
        } else {
            for (int i = 0; i + m <= n; i++) C[i + m] ^= f.mul(coef, B[i]);
            m++;
        }
    }
    return L;
}

inline uint8_t bitAt(const uint8_t* p, int i) { return (p[i >> 3] >> (i & 7)) & 1; }
inline uint8_t par8(uint8_t v) { v ^= v >> 4; v ^= v >> 2; v ^= v >> 1; return v & 1; }

// ---------------------------------------------------------------- Hamming SECDED
// Data bit i gets a distinct 9-bit column (non power of two, >=3). Parity bit j has column 1<<j.
// Parity byte 0 = p0..p7, byte 1 bit0 = p8, bit1 = overall parity.
inline uint16_t nextCol(uint16_t c) { do { c++; } while ((c & (c - 1)) == 0); return c; }

bool hamEncode(const uint8_t* d, uint8_t len, uint8_t* par) {
    if (len > 62) return false;
    uint16_t syn = 0; uint8_t ov = 0, col = 0; uint16_t c = 2;
    (void)col;
    for (int i = 0; i < 8 * len; i++) {
        c = nextCol(c);
        if (bitAt(d, i)) { syn ^= c; ov ^= 1; }
    }
    uint8_t pc = par8(syn & 0xFF) ^ (uint8_t)((syn >> 8) & 1);
    par[0] = syn & 0xFF;
    par[1] = ((syn >> 8) & 1) | ((ov ^ pc) << 1);
    return true;
}

bool hamDecode(uint8_t* d, uint8_t len, const uint8_t* par, uint8_t* corr) {
    if (len > 62) return false;
    uint16_t syn = 0; uint8_t ov = 0; uint16_t c = 2;
    for (int i = 0; i < 8 * len; i++) {
        c = nextCol(c);
        if (bitAt(d, i)) { syn ^= c; ov ^= 1; }
    }
    uint16_t p9 = par[0] | ((par[1] & 1) << 8);
    syn ^= p9;
    ov  ^= par8(par[0]) ^ (par[1] & 1) ^ ((par[1] >> 1) & 1);   // overall parity of whole word
    *corr = 0;
    if (!syn && !ov) return true;
    if (!ov) return false;                     // syndrome != 0, parity even -> double error
    *corr = 1;
    if (!syn) return true;                     // the overall-parity bit itself flipped
    if ((syn & (syn - 1)) == 0) return true;   // a parity bit flipped, data intact
    c = 2;
    for (int i = 0; i < 8 * len; i++) {
        c = nextCol(c);
        if (c == syn) { d[i >> 3] ^= (uint8_t)(1 << (i & 7)); return true; }
    }
    *corr = 0;
    return false;                              // syndrome points into the shortened-away region
}

// ---------------------------------------------------------------- BCH over GF(2^9), n=511
// Generator = product of minimal polynomials of alpha^1..alpha^2t.
int bchGen(uint8_t t, uint16_t* g /*>=48*/) {
    g9.init(0x211);
    memset(g, 0, 48 * sizeof(uint16_t));
    g[0] = 1; int deg = 0;
    uint8_t seen[64]; memset(seen, 0, sizeof seen);
    for (int i = 1; i <= 2 * t; i++) {
        if (seen[i >> 3] & (1 << (i & 7))) continue;
        int e = i;
        do {
            seen[e >> 3] |= (uint8_t)(1 << (e & 7));
            uint16_t a = g9.pw(e);
            for (int j = deg + 1; j >= 1; j--) g[j] = g[j - 1] ^ g9.mul(g[j], a);
            g[0] = g9.mul(g[0], a);
            deg++;
            e = (e * 2) % 511;
        } while (e != i);
    }
    return deg;
}

bool bchEncode(uint8_t t, const uint8_t* d, uint8_t len, uint8_t* par, uint8_t pbytes) {
    if (len > 58) return false;
    uint16_t g[48]; int p = bchGen(t, g);
    if (p > 45) return false;
    uint8_t rem[48]; memset(rem, 0, sizeof rem);
    for (int j = 8 * len - 1; j >= 0; j--) {
        uint8_t fb = bitAt(d, j) ^ rem[p - 1];
        for (int i = p - 1; i > 0; i--) rem[i] = rem[i - 1];
        rem[0] = 0;
        if (fb) for (int i = 0; i < p; i++) rem[i] ^= (uint8_t)g[i];
    }
    memset(par, 0, pbytes);
    for (int i = 0; i < p; i++) if (rem[i]) par[i >> 3] |= (uint8_t)(1 << (i & 7));
    return true;
}

bool bchDecode(uint8_t t, uint8_t* d, uint8_t len, const uint8_t* par, uint8_t* corr) {
    if (len > 58) return false;
    uint16_t g[48]; int p = bchGen(t, g);
    int n = p + 8 * len;
    uint16_t S[MAXS] = {0};
    for (int i = 0; i < n; i++) {
        uint8_t bit = (i < p) ? bitAt(par, i) : bitAt(d, i - p);
        if (!bit) continue;
        for (int j = 1; j <= 2 * t; j++) S[j - 1] ^= g9.exp[(i * j) % 511];
    }
    bool any = false;
    for (int j = 0; j < 2 * t; j++) any |= (S[j] != 0);
    *corr = 0;
    if (!any) return true;

    uint16_t C[MAXS + 2];
    int L = berlekampMassey(g9, S, 2 * t, C);
    if (L > t) return false;
    int pos[8]; int cnt = 0;
    for (int i = 0; i < n && cnt <= L; i++) {
        uint16_t sum = 0;
        for (int j = 0; j <= L; j++) sum ^= g9.mul(C[j], g9.pw(-(i * j)));
        if (!sum) { if (cnt < 8) pos[cnt] = i; cnt++; }
    }
    if (cnt != L) return false;
    for (int k = 0; k < cnt; k++)
        if (pos[k] >= p) d[(pos[k] - p) >> 3] ^= (uint8_t)(1 << ((pos[k] - p) & 7));
    *corr = (uint8_t)L;
    return true;
}

// ---------------------------------------------------------------- Reed-Solomon over GF(2^8), fcr = 0
void rsGen(int np, uint16_t* g /*np+1, low-first, monic*/) {
    g8.init(0x11D);
    memset(g, 0, (np + 1) * sizeof(uint16_t));
    g[0] = 1; int deg = 0;
    for (int i = 0; i < np; i++) {
        uint16_t a = g8.pw(i);
        for (int j = deg + 1; j >= 1; j--) g[j] = g[j - 1] ^ g8.mul(g[j], a);
        g[0] = g8.mul(g[0], a);
        deg++;
    }
}

bool rsEncode(uint8_t t, const uint8_t* d, uint8_t len, uint8_t* par) {
    int np = 2 * t;
    if (len + np > 255) return false;
    uint16_t g[17]; rsGen(np, g);
    memset(par, 0, np);
    for (int i = 0; i < len; i++) {
        uint8_t fb = d[i] ^ par[0];
        for (int j = 0; j < np - 1; j++) par[j] = par[j + 1] ^ (uint8_t)g8.mul(fb, g[np - 1 - j]);
        par[np - 1] = (uint8_t)g8.mul(fb, g[0]);
    }
    return true;
}

bool rsSyndromes(const uint8_t* cw, int n, int np, uint16_t* S) {
    bool any = false;
    for (int j = 0; j < np; j++) {
        uint16_t a = g8.pw(j), s = 0;
        for (int k = 0; k < n; k++) s = g8.mul(s, a) ^ cw[k];
        S[j] = s; any |= (s != 0);
    }
    return any;
}

bool rsDecode(uint8_t t, uint8_t* d, uint8_t len, const uint8_t* par, uint8_t* corr) {
    int np = 2 * t, n = len + np;
    if (n > 255) return false;
    g8.init(0x11D);
    uint8_t cw[255]; memcpy(cw, d, len); memcpy(cw + len, par, np);
    uint16_t S[MAXS];
    *corr = 0;
    if (!rsSyndromes(cw, n, np, S)) return true;

    uint16_t C[MAXS + 2];
    int L = berlekampMassey(g8, S, np, C);
    if (L > t) return false;
    int pos[8]; int cnt = 0;
    for (int k = 0; k < n && cnt <= L; k++) {
        int e = n - 1 - k; uint16_t sum = 0;
        for (int l = 0; l <= L; l++) sum ^= g8.mul(C[l], g8.pw(-(e * l)));
        if (!sum) { if (cnt < 8) pos[cnt] = k; cnt++; }
    }
    if (cnt != L) return false;

    uint16_t Om[MAXS];                                   // Omega = S(x)*Lambda(x) mod x^np
    for (int i = 0; i < np; i++) {
        uint16_t v = 0;
        for (int j = 0; j <= i; j++) if (i - j <= L) v ^= g8.mul(S[j], C[i - j]);
        Om[i] = v;
    }
    for (int q = 0; q < cnt; q++) {
        int e = n - 1 - pos[q];
        uint16_t om = 0, den = 0;
        for (int i = 0; i < np; i++) om ^= g8.mul(Om[i], g8.pw(-(e * i)));
        for (int l = 1; l <= L; l += 2) den ^= g8.mul(C[l], g8.pw(-(e * (l - 1))));
        if (!den) return false;
        // Forney with fcr=0:  e_k = X_k * Omega(X_k^-1) / Lambda'(X_k^-1)
        cw[pos[q]] ^= (uint8_t)g8.mul(g8.pw(e), g8.div(om, den));
    }
    if (rsSyndromes(cw, n, np, S)) return false;         // belt and braces
    memcpy(d, cw, len);
    *corr = (uint8_t)L;
    return true;
}

const LevelInfo kLevels[NUM_LEVELS] = {
    { Scheme::HAMMING, 1, 2  },
    { Scheme::BCH,     2, 3  },
    { Scheme::BCH,     5, 6  },
    { Scheme::RS,      2, 4  },
    { Scheme::RS,      4, 8  },
    { Scheme::RS,      8, 16 },
};
const char* const kNames[NUM_LEVELS] = {
    "Hamming-SECDED", "BCH t=2", "BCH t=5", "RS t=2", "RS t=4", "RS t=8"
};

} // namespace

const LevelInfo& levelInfo(uint8_t level) { return kLevels[level > MAX_LEVEL ? MAX_LEVEL : level]; }
const char* levelName(uint8_t level)      { return kNames[level > MAX_LEVEL ? MAX_LEVEL : level]; }

void eccInit() { g9.init(0x211); g8.init(0x11D); }

bool eccEncode(uint8_t level, const uint8_t* data, uint8_t len, uint8_t* parity) {
    eccInit();
    const LevelInfo& li = levelInfo(level);
    switch (li.scheme) {
        case Scheme::HAMMING: return hamEncode(data, len, parity);
        case Scheme::BCH:     return bchEncode(li.t, data, len, parity, li.parityBytes);
        case Scheme::RS:      return rsEncode(li.t, data, len, parity);
    }
    return false;
}

bool eccDecode(uint8_t level, uint8_t* data, uint8_t len, const uint8_t* parity, uint8_t* corrected) {
    eccInit();
    uint8_t dummy; if (!corrected) corrected = &dummy;
    const LevelInfo& li = levelInfo(level);
    switch (li.scheme) {
        case Scheme::HAMMING: return hamDecode(data, len, parity, corrected);
        case Scheme::BCH:     return bchDecode(li.t, data, len, parity, corrected);
        case Scheme::RS:      return rsDecode(li.t, data, len, parity, corrected);
    }
    return false;
}

uint16_t crc16(const uint8_t* d, size_t n) {
    uint16_t c = 0xFFFF;
    while (n--) {
        c ^= (uint16_t)(*d++) << 8;
        for (int i = 0; i < 8; i++) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1);
    }
    return c;
}

} // namespace afec
