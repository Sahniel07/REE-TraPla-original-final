#ifndef TRIG_H
#define TRIG_H

#include <cmath>
#include "poly_trig.h"

/*
 * Trigonometric path selector for integrate().
 *
 *   USE_POLY 1   fdlibm-style minimax polynomial, evaluated entirely in the
 *                working format after a Cody-Waite range reduction
 *   USE_POLY 0   convert the argument to fp64, evaluate with the C library,
 *                round the result back into the working format
 *
 * Set by the build (-DTRIG_PATH=poly|upscale). The fallback matches the CMake
 * default, so both routes select the same path.
 */
#ifndef USE_POLY
#define USE_POLY 1
#endif

/* TRIG_SINCOS returns both functions from a single range reduction. */
#if USE_POLY
  #define TRIG_COS(x) poly::cos(x)
  #define TRIG_SIN(x) poly::sin(x)
  #define TRIG_TAN(x) poly::tan(x)
  #define TRIG_SINCOS(a, s, c) poly::sincos((a), (s), (c))
#else
  #define TRIG_COS(x) cos(x)
  #define TRIG_SIN(x) sin(x)
  #define TRIG_TAN(x) tan(x)
  #define TRIG_SINCOS(a, s, c) do { (s) = sin(a); (c) = cos(a); } while (0)
#endif

#endif  // TRIG_H
