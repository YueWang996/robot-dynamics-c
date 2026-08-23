/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file rd_config.h
 * @brief Robot Dynamics Library Configuration
 */

#ifndef RD_CONFIG_H
#define RD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Platform Detection
 * ============================================================================ */

/* Detect ARM Cortex-M */
#if defined(__ARM_ARCH) || defined(__arm__) || defined(__ARM_ARCH_7M__) || \
    defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_8M_MAIN__)
    #define RD_PLATFORM_ARM  1
#else
    #define RD_PLATFORM_ARM  0
#endif

/* Detect Cortex-M4/M7 with FPU (for CMSIS-DSP optimizations) */
#if defined(__ARM_ARCH_7EM__) || defined(__ARM_ARCH_8M_MAIN__)
    #define RD_PLATFORM_CORTEX_M4_PLUS  1
#else
    #define RD_PLATFORM_CORTEX_M4_PLUS  0
#endif

/* ============================================================================
 * User Configuration (can be overridden via compiler flags)
 * ============================================================================ */

/* Use single precision (float) instead of double (Default: 1) */
#ifndef RD_USE_SINGLE_PRECISION
    #define RD_USE_SINGLE_PRECISION  1
#endif

/* Polynomial sin/cos instead of libm's: 169 cycles per (sin, cos) pair against
 * libm's 521, measured inside rd_update_kinematics on an STM32G474 at 170 MHz,
 * at the same accuracy -- 6.6e-08 worst case against double precision over
 * [-pi, pi], where libm is 5.9e-08. Both pass the Pinocchio comparison at the
 * same tolerances. Set to 0 for libm's. */
#ifndef RD_FAST_TRIG
    #define RD_FAST_TRIG  1
#endif

/* A board's own math accelerator.
 *
 * Define RD_MATH_BACKEND to a header of yours -- in quotes or angle brackets,
 * the way #include wants it -- and rd_math.h includes it before it defines
 * anything. The header may define any of:
 *
 *   RD_SINCOS(x, sp, cp)   sine and cosine of x, into *(sp) and *(cp)
 *   RD_SQRT(x)             square root of x, as an expression
 *
 * Whatever it leaves undefined keeps the portable implementation, so a backend
 * may cover one operation and ignore the other.
 *
 * These are macros and not function pointers deliberately. rd_sincos() is
 * called once per revolute joint from inside rd_update_kinematics()'s loop and
 * is 45% of that function on Go2; an indirect call there costs more than most
 * accelerators save, and it would also stop the compiler from keeping the
 * caller's values in registers across it.
 *
 *   #define RD_MATH_BACKEND "my_cordic.h"
 *
 * Nothing of the sort ships with the library, deliberately -- a backend is
 * yours to write and yours to own, and this is only the seam it hangs on.
 * examples/backends/ in the repository has a worked one for the STM32G4
 * CORDIC to read.
 */

/* Compile the articulated-body algorithm (Default: 1).
 *
 * ABA is the only algorithm here that carries per-node state of its own -- an
 * articulated inertia, a velocity-product acceleration, and the U/D/u triple
 * its outward pass needs -- so it is what sizes rd_state_t. With it, a link
 * costs 70 floats of workspace; without it, 45. On a 40-link model in a
 * float32 build that is 11,264 bytes against 7,264.
 *
 * Set to 0 in a build whose forward dynamics comes from
 * rd_forward_dynamics(RD_FD_CRBA), which is the faster of the two on most
 * models a microcontroller runs, or in one that only needs inverse dynamics.
 * rd_aba() and rd_aba_ext() are then not declared, and rd_forward_dynamics()
 * answers RD_ERR_INVALID_INDEX to RD_FD_ABA. */
#ifndef RD_ENABLE_ABA
    #define RD_ENABLE_ABA  1
#endif

/* Use ARM CMSIS-DSP library for matrix operations (Default: 0) */
#ifndef RD_USE_CMSIS_DSP
    #define RD_USE_CMSIS_DSP  0
#endif

/* Use static memory allocation (no malloc/free) (Default: 0) */
#ifndef RD_USE_STATIC_ALLOC
    #define RD_USE_STATIC_ALLOC  0
#endif

/* Force CORDIC disabled */
#define RD_USE_CORDIC  0

/* Maximum number of links (for static allocation) */
#ifndef RD_MAX_LINKS
    #define RD_MAX_LINKS  16
#endif

/* Maximum number of joints (for static allocation) */
#ifndef RD_MAX_JOINTS
    #define RD_MAX_JOINTS  12
#endif

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

#if RD_USE_CMSIS_DSP && RD_PLATFORM_ARM
    #include "arm_math.h"
    typedef float32_t rd_real_t;
    #define RD_REAL_IS_FLOAT  1
#elif RD_USE_SINGLE_PRECISION
    typedef float rd_real_t;
    #define RD_REAL_IS_FLOAT  1
#else
    typedef double rd_real_t;
    #define RD_REAL_IS_FLOAT  0
#endif

/* Integer types */
#include <stdint.h>
typedef int32_t  rd_int_t;
typedef uint32_t rd_uint_t;
typedef int16_t  rd_idx_t;   /* Index type for arrays */

/* ============================================================================
 * Math Constants
 * ============================================================================ */

#if RD_REAL_IS_FLOAT
    #define RD_PI          3.14159265358979f
    #define RD_2PI         6.28318530717959f
    #define RD_HALF_PI     1.57079632679490f
    #define RD_INV_PI      0.31830988618379f
    #define RD_GRAVITY     9.81f
    #define RD_EPS         1e-6f
    #define RD_REAL(x)     (x##f)
#else
    #define RD_PI          3.14159265358979323846
    #define RD_2PI         6.28318530717958647693
    #define RD_HALF_PI     1.57079632679489661923
    #define RD_INV_PI      0.31830988618379067154
    #define RD_GRAVITY     9.81
    #define RD_EPS         1e-12
    #define RD_REAL(x)     (x)
#endif

/* ============================================================================
 * Memory Management
 * ============================================================================ */

#if RD_USE_STATIC_ALLOC
    #define RD_MALLOC(size)       NULL
    #define RD_FREE(ptr)          ((void)0)
    #define RD_CALLOC(n, size)    NULL
#else
    #include <stdlib.h>
    #define RD_MALLOC(size)       malloc(size)
    #define RD_FREE(ptr)          free(ptr)
    #define RD_CALLOC(n, size)    calloc(n, size)
#endif

/* ============================================================================
 * Compiler Hints
 * ============================================================================ */

#if defined(__GNUC__) || defined(__clang__)
    #define RD_INLINE       inline __attribute__((always_inline))
    #define RD_NOINLINE     __attribute__((noinline))
    #define RD_HOT          __attribute__((hot))
    #define RD_PURE         __attribute__((const))
    #define RD_RESTRICT     __restrict__
#else
    #define RD_INLINE       inline
    #define RD_NOINLINE
    #define RD_HOT
    #define RD_PURE
    #define RD_RESTRICT
#endif

/* Alignment for SIMD/DMA (4-byte for Cortex-M4) */
#if RD_PLATFORM_ARM
    #define RD_ALIGN(n)     __attribute__((aligned(n)))
    #define RD_ALIGN4       __attribute__((aligned(4)))
#else
    #define RD_ALIGN(n)
    #define RD_ALIGN4
#endif

/* ============================================================================
 * Error Codes
 * ============================================================================ */

typedef enum {
    RD_OK                =  0,
    RD_ERR_NULL_PTR      = -1,
    RD_ERR_INVALID_INDEX = -2,
    RD_ERR_INVALID_FRAME = -3,
    RD_ERR_ALLOC_FAILED  = -4,
    RD_ERR_SINGULAR      = -5,
    RD_ERR_INVALID_SIZE  = -6
} rd_status_t;

/* ============================================================================
 * Debug Support
 * ============================================================================ */

#ifndef RD_DEBUG
    #define RD_DEBUG  0
#endif

#if RD_DEBUG
    #include <stdio.h>
    #define RD_ASSERT(cond, msg)  do { if (!(cond)) { printf("RD Assert: %s\n", msg); } } while(0)
    #define RD_LOG(fmt, ...)      printf("[RD] " fmt "\n", ##__VA_ARGS__)
#else
    #define RD_ASSERT(cond, msg)  ((void)0)
    #define RD_LOG(fmt, ...)      ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* RD_CONFIG_H */