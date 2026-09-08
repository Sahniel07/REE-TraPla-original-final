#ifndef POLY_TRIG_SIMPLE_H
#define POLY_TRIG_SIMPLE_H

#include <cmath>

/*
 * Common native-format sin/cos kernel for IEEE-like T and posit T.
 *
 * Algorithm: bounded Cody-Waite-style pi/2 reduction + fdlibm-style
 * reduced-range polynomials. All numerical arithmetic after constants are
 * converted is performed in T. No quire and no double sin/cos/tan call.
 *
 * Intended sincos input range: [-2.03, 3.71] radians, giving k in {-1,0,1,2}.
 *
 * Derived from the Sun Freely Distributable Math Library (fdlibm), 1993:
 *   s1..s6 (sine coefficients)    https://www.netlib.org/fdlibm/k_sin.c
 *   c1..c6 (cosine coefficients)  https://www.netlib.org/fdlibm/k_cos.c
 *   reduce/evaluate/quadrant flow https://www.netlib.org/fdlibm/s_sin.c
 *
 * cos_small() uses the single form 1 - z/2 + z*z*q. __kernel_cos carries an
 * additional compensating term for |x| >= 0.3 that exists to recover the last
 * bits in binary64; it is omitted here because this kernel is instantiated at
 * many precisions.
 *
 * The fdlibm licence requires its notice to be preserved in redistributions,
 * so it is reproduced below and applies to those coefficients and to the
 * kernel structure taken from that library.
 */

/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunSoft, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */


namespace poly {

constexpr double PI_2        = 1.5707963267948966192313216916398;
constexpr double TWO_OVER_PI = 0.63661977236758134307553505349006;

template <typename T>
int significand_bits() {
    for (int p = 1; p < 64; ++p) {
        if (double(T(1.0 + std::ldexp(1.0, -p))) == 1.0) return p;
    }
    return 64;
}

inline double truncated_pi_over_2(int bits) {
    return std::ldexp(std::floor(std::ldexp(PI_2, bits - 1)), -(bits - 1));
}

template <typename T>
struct Constants {
    T two_over_pi, pio2_hi, pio2_lo;
    T s1, s2, s3, s4, s5, s6;
    T c1, c2, c3, c4, c5, c6;

    Constants() {
        two_over_pi = T(TWO_OVER_PI);

        /* pi/2 = pio2_hi + pio2_lo; the split reduces cancellation. */
        const double hi = truncated_pi_over_2(significand_bits<T>() - 2);
        pio2_hi = T(hi);
        pio2_lo = T(PI_2 - hi);

        /* fdlibm sine coefficients. */
        s1 = T(-1.66666666666666324348e-01);
        s2 = T( 8.33333333332248946124e-03);
        s3 = T(-1.98412698298579493134e-04);
        s4 = T( 2.75573137070700676789e-06);
        s5 = T(-2.50507602534068634195e-08);
        s6 = T( 1.58969099521155010221e-10);

        /* fdlibm cosine coefficients. */
        c1 = T( 4.16666666666666019037e-02);
        c2 = T(-1.38888888888741095749e-03);
        c3 = T( 2.48015872894767294178e-05);
        c4 = T(-2.75573143513906633035e-07);
        c5 = T( 2.08757232129817482790e-09);
        c6 = T(-1.13596475577881948265e-11);
    }
};

template <typename T>
const Constants<T>& constants() {
    static const Constants<T> v;
    return v;
}

/* Polynomial kernels for r in approximately [-pi/4, pi/4]. */
template <typename T>
T sin_small(T r) {
    const Constants<T>& a = constants<T>();
    const T z = T(r * r);
    const T p = T(a.s1 + z * T(a.s2 + z * T(a.s3 + z * T(a.s4 + z * T(a.s5 + z * a.s6)))));
    return T(r + r * z * p);
}

template <typename T>
T cos_small(T r) {
    const Constants<T>& a = constants<T>();
    const T z = T(r * r);
    const T p = T(a.c1 + z * T(a.c2 + z * T(a.c3 + z * T(a.c4 + z * T(a.c5 + z * a.c6)))));
    return T(T(1.0) - T(0.5) * z + z * z * p);
}

/* Returns r = x - k*pi/2, with native-T range-reduction arithmetic. */
template <typename T>
T reduce(T x, long& k) {
    const Constants<T>& a = constants<T>();
    const T quarter_turns = T(x * a.two_over_pi); // native T multiplication
    k = std::lround(double(quarter_turns));       // only converts T to small integer
    const T kt = T(double(k));
    return T(T(x - kt * a.pio2_hi) - kt * a.pio2_lo);
}

template <typename T>
void sincos(T x, T& s, T& c) {
    long k;
    const T r = reduce(x, k);
    const T sr = sin_small(r);
    const T cr = cos_small(r);

    switch (((k % 4) + 4) % 4) {
        case 0:  s = sr;    c = cr;    break;
        case 1:  s = cr;    c = -sr;   break;
        case 2:  s = -sr;   c = -cr;   break;
        default: s = -cr;   c = sr;    break;
    }
}

template <typename T> T sin(T x) { T s, c; sincos(x, s, c); return s; }
template <typename T> T cos(T x) { T s, c; sincos(x, s, c); return c; }

/* Use only when cos(x) is safely nonzero; never across tan poles. */
template <typename T> T tan(T x) { T s, c; sincos(x, s, c); return T(s / c); }

} // namespace poly


#endif // POLY_TRIG_SIMPLE_H
