/**
 * @file math/util.h
 *
 * @brief A common math library for use in both floating and fixed point.
 */

/*
 * Copyright (c) 2022 Google Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef INCLUDE_MATH_UTIL_H_
#define INCLUDE_MATH_UTIL_H_

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_FPU
#define PRIf	    "f"
#define PRIf_ARG(n) (n)
#else
#define PRIf		    "d.%06llu"
#define __FP_RAW_DECIMAL(n) ((int16_t)((n) & ((1 << FP_BITS) - 1)))
#define __FP_DECIMAL(n)	    ((n < 0) ? (~__FP_RAW_DECIMAL(n) + 1) : __FP_RAW_DECIMAL(n))
#define PRIf_ARG(n)                                                                                \
	(FP_TO_INT(abs(n)) * (n < 0 ? -1 : 1)),                                                    \
		((__FP_DECIMAL(n) * UINT64_C(1000000)) + ((1 << FP_BITS) / 2)) / (1 << FP_BITS)
#endif

#ifdef CONFIG_FPU
typedef float fp_t;
typedef float fp_inter_t;
/* Conversion to/from fixed-point */
#define INT_TO_FP(x)   ((float)(x))
#define FP_TO_INT(x)   ((int32_t)(x))
/* Float to fixed-point, only for compile-time constants and unit tests */
#define FLOAT_TO_FP(x) ((float)(x))
/* Fixed-point to float, for unit tests */
#define FP_TO_FLOAT(x) ((float)(x))

#define FLT_MAX (3.4028234664e+38)
#define FLT_MIN (1.1754943508e-38)

#else
/* Fixed-point type */
typedef int32_t fp_t;

/* Type used during fp operation */
typedef int64_t fp_inter_t;

/* Number of bits left of decimal point for fixed-point */
#define FP_BITS	       16

/* Conversion to/from fixed-point */
#define INT_TO_FP(x)   ((fp_t)(x) << FP_BITS)
#define FP_TO_INT(x)   ((int32_t)((x) >> FP_BITS))
/* Float to fixed-point, only for compile-time constants and unit tests */
#define FLOAT_TO_FP(x) ((fp_t)((x) * (float)(1 << FP_BITS)))
/* Fixed-point to float, for unit tests */
#define FP_TO_FLOAT(x) ((float)(x) / (float)(1 << FP_BITS))

#define FLT_MAX INT32_MAX
#define FLT_MIN INT32_MIN

#endif

/**
 * @brief Perform a floating/fixed point multiplication
 *
 * @param a The left side of the multiplication
 * @param b The right side of the multiplication
 * @return (a * b)
 */
#ifdef CONFIG_FPU
static inline fp_t fp_mul(fp_t a, fp_t b) { return a * b; }
#else
static inline fp_t fp_mul(fp_t a, fp_t b) { return (fp_t)(((fp_inter_t)a * b) >> FP_BITS); }
#endif

/**
 * @brief Perform a floating/fixed point division
 *
 * @param a The left side of the division
 * @param b The right side of the division
 * @return (a / b)
 */
#ifdef CONFIG_FPU
static inline fp_t fp_div(fp_t a, fp_t b) { return a / b; }
#else
static inline fp_t fp_div(fp_t a, fp_t b)
{
	return (fp_t)(((fp_inter_t)a << FP_BITS) / b);
}
#endif

#ifdef __cplusplus
};
#endif

#endif /* INCLUDE_MATH_UTIL_H_ */
