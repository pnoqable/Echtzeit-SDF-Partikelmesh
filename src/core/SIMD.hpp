#pragma once

// Kleine 4-Lane-Float-Abstraktion fuer die Kraftberechnung (src/simulation/
// ParticleSystem.cpp). Ziel: den paarweisen Abstossungs-Kern auf vier
// Nachbarn gleichzeitig rechnen.
//
// Pfade:
//   - aarch64 (Apple Silizium, ARM64-Windows): NEON (vsqrt/vdiv native)
//   - x86-64 (MacIntel, Windows): SSE2 (portabel, wo immer verfuegbar)
//   - sonst: skalarer Fallback mit identischer Semantik
//
// Nur Haupt-Thread-Nutzung im Kraft-Loop; keine Atomics noetig.

#include <cmath>
#include <cstring>

#if defined(__aarch64__)
#include <arm_neon.h>
#define SDFF4_NEON 1
#elif defined(__SSE2__) || defined(_M_SSE)
#include <immintrin.h>
#define SDFF4_SSE 1
#endif

namespace simd {

struct F4 {
#if defined(SDFF4_NEON)
    float32x4_t v;
#elif defined(SDFF4_SSE)
    __m128 v;
#else
    float c[4];
#endif

    static F4 set1(float s) {
        F4 r;
#if defined(SDFF4_NEON)
        r.v = vdupq_n_f32(s);
#elif defined(SDFF4_SSE)
        r.v = _mm_set1_ps(s);
#else
        r.c[0] = r.c[1] = r.c[2] = r.c[3] = s;
#endif
        return r;
    }

    static F4 load4(const float* p) {
        F4 r;
#if defined(SDFF4_NEON)
        r.v = vld1q_f32(p);
#elif defined(SDFF4_SSE)
        r.v = _mm_loadu_ps(p);
#else
        std::memcpy(r.c, p, 4 * sizeof(float));
#endif
        return r;
    }

    static F4 zero() { return set1(0.0f); }

    F4 add(const F4& b) const {
        F4 r;
#if defined(SDFF4_NEON)
        r.v = vaddq_f32(v, b.v);
#elif defined(SDFF4_SSE)
        r.v = _mm_add_ps(v, b.v);
#else
        for (int i = 0; i < 4; ++i) r.c[i] = c[i] + b.c[i];
#endif
        return r;
    }

    F4 sub(const F4& b) const {
        F4 r;
#if defined(SDFF4_NEON)
        r.v = vsubq_f32(v, b.v);
#elif defined(SDFF4_SSE)
        r.v = _mm_sub_ps(v, b.v);
#else
        for (int i = 0; i < 4; ++i) r.c[i] = c[i] - b.c[i];
#endif
        return r;
    }

    F4 mul(const F4& b) const {
        F4 r;
#if defined(SDFF4_NEON)
        r.v = vmulq_f32(v, b.v);
#elif defined(SDFF4_SSE)
        r.v = _mm_mul_ps(v, b.v);
#else
        for (int i = 0; i < 4; ++i) r.c[i] = c[i] * b.c[i];
#endif
        return r;
    }

    F4 neg() const {
        F4 r;
#if defined(SDFF4_NEON)
        r.v = vnegq_f32(v);
#elif defined(SDFF4_SSE)
        r.v = _mm_sub_ps(_mm_setzero_ps(), v);
#else
        for (int i = 0; i < 4; ++i) r.c[i] = -c[i];
#endif
        return r;
    }

    static F4 sqrtv(const F4& a) {
        F4 r;
#if defined(SDFF4_NEON)
        r.v = vsqrtq_f32(a.v);
#elif defined(SDFF4_SSE)
        r.v = _mm_sqrt_ps(a.v);
#else
        for (int i = 0; i < 4; ++i) r.c[i] = std::sqrt(a.c[i]);
#endif
        return r;
    }

    static F4 divv(const F4& a, const F4& b) {
        F4 r;
#if defined(SDFF4_NEON)
        r.v = vdivq_f32(a.v, b.v);
#elif defined(SDFF4_SSE)
        r.v = _mm_div_ps(a.v, b.v);
#else
        for (int i = 0; i < 4; ++i) r.c[i] = a.c[i] / b.c[i];
#endif
        return r;
    }

    // Lanes mit all-ones (bit-gesetzter Maske) markieren.
    static F4 ge(const F4& a, const F4& b) {
        F4 r;
#if defined(SDFF4_NEON)
        r.v = vreinterpretq_f32_u32(vcgeq_f32(a.v, b.v));
#elif defined(SDFF4_SSE)
        r.v = _mm_cmpge_ps(a.v, b.v);
#else
        for (int i = 0; i < 4; ++i)
            r.c[i] = a.c[i] >= b.c[i] ? 1.0f : 0.0f;
#endif
        return r;
    }

    static F4 lt(const F4& a, const F4& b) {
        F4 r;
#if defined(SDFF4_NEON)
        r.v = vreinterpretq_f32_u32(vcltq_f32(a.v, b.v));
#elif defined(SDFF4_SSE)
        r.v = _mm_cmplt_ps(a.v, b.v);
#else
        for (int i = 0; i < 4; ++i)
            r.c[i] = a.c[i] < b.c[i] ? 1.0f : 0.0f;
#endif
        return r;
    }

    static F4 andMask(const F4& a, const F4& b) {
        F4 r;
#if defined(SDFF4_NEON)
        r.v = vreinterpretq_f32_u32(
            vandq_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
#elif defined(SDFF4_SSE)
        r.v = _mm_and_ps(a.v, b.v);
#else
        for (int i = 0; i < 4; ++i)
            r.c[i] = (a.c[i] > 0.5f && b.c[i] > 0.5f) ? 1.0f : 0.0f;
#endif
        return r;
    }

    // Lane uebernimmt diesen Wert, wo `mask` gesetzt ist, sonst `other`.
    F4 select(const F4& mask, const F4& other) const {
        F4 r;
#if defined(SDFF4_NEON)
        r.v = vbslq_f32(mask.v, v, other.v);
#elif defined(SDFF4_SSE)
        r.v = _mm_or_ps(_mm_and_ps(mask.v, v),
                        _mm_andnot_ps(mask.v, other.v));
#else
        for (int i = 0; i < 4; ++i)
            r.c[i] = mask.c[i] > 0.5f ? c[i] : other.c[i];
#endif
        return r;
    }

    float hsum() const {
#if defined(SDFF4_NEON)
        return vaddvq_f32(v);
#elif defined(SDFF4_SSE)
        float q[4];
        _mm_storeu_ps(q, v);
        return q[0] + q[1] + q[2] + q[3];
#else
        return c[0] + c[1] + c[2] + c[3];
#endif
    }
};

inline F4 operator+(const F4& a, const F4& b) { return a.add(b); }
inline F4 operator-(const F4& a, const F4& b) { return a.sub(b); }
inline F4 operator*(const F4& a, const F4& b) { return a.mul(b); }

} // namespace simd

#undef SDFF4_NEON
#undef SDFF4_SSE