/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file rd_config.h
 * @brief Build options, scalar types, and status codes.
 *
 * Everything here can be set from the compiler command line, and the defaults
 * are what a Cortex-M4F control loop wants. Two of them change struct layout --
 * RD_USE_SINGLE_PRECISION and RD_ENABLE_ABA -- so every translation unit in a
 * program has to be built with the same values.
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

/** float for rd_real_t rather than double (Default: 1).
 *
 * The FPU on a Cortex-M4F or an M33 is single precision and nothing else, so a
 * double build does its arithmetic in software on exactly the parts this
 * library was written for. Single precision agrees with Pinocchio to 1.9e-06
 * across the test models, which is finer than the encoder and the inertia
 * numbers feeding it. Double reaches 5.5e-15 and belongs in a host build that
 * is checking something. */
#ifndef RD_USE_SINGLE_PRECISION
    #define RD_USE_SINGLE_PRECISION  1
#endif

/** Polynomial sin/cos instead of libm's: 169 cycles per (sin, cos) pair against
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

/** Compile the articulated-body algorithm (Default: 1).
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

/** Take sqrt from ARM's CMSIS-DSP rather than libm (Default: 0).
 *
 * CMSIS-DSP has to be on the include path, and the CMake option that turns
 * this on wants RD_CMSIS_DSP_INCLUDE_DIR set alongside it. RD_MATH_BACKEND
 * reaches the same place without a vendor dependency and is the seam to prefer
 * for new work. */
#ifndef RD_USE_CMSIS_DSP
    #define RD_USE_CMSIS_DSP  0
#endif

/** Make RD_MALLOC and RD_CALLOC return NULL, for a build with no heap (Default: 0).
 *
 * The control loop allocates nothing either way -- every algorithm works out of
 * the buffer handed to rd_state_init(). rd_chain_build() is the single
 * exception, once at startup, and with this on it fails instead. So the switch
 * is honest about a heapless build and does not yet produce one. */
#ifndef RD_USE_STATIC_ALLOC
    #define RD_USE_STATIC_ALLOC  0
#endif

/** Links a model may hold, which is the length of rd_model_t::links (Default: 16).
 *
 * rd_model_t carries that array inside itself, so this is the size of every
 * model object in the program whether the robot fills it or not. Set it to
 * the link count of the URDF you converted: Go2 needs 31, the G1 humanoid 40. */
#ifndef RD_MAX_LINKS
    #define RD_MAX_LINKS  16
#endif

/** Actuated joints a model may hold (Default: 12).
 *
 * Sizes the joint-space arrays a caller declares -- tau, qdd and the rest are
 * conventionally 6 + RD_MAX_JOINTS so that one declaration covers a floating
 * base as well. Fixed links do not count against it; Go2 has 31 links and 12
 * joints. */
#ifndef RD_MAX_JOINTS
    #define RD_MAX_JOINTS  12
#endif

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

/** The scalar type every array in the library is made of.
 *
 * float or double, chosen by RD_USE_SINGLE_PRECISION. Write literals with
 * RD_REAL() so they carry the right suffix in both builds. */
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
typedef int32_t  rd_int_t;   /**< Counts and sizes: nv, n_nodes, row counts. */
typedef uint32_t rd_uint_t;  /**< Unsigned counts, in the model description. */
/** A link or frame index.
 *
 * Signed and 16 bits, so it is half the width of a pointer on the parts this
 * runs on and negative values are free to mean something: -1 is "no parent" in
 * rd_link_t::parent_idx and RD_ANCHOR_WORLD in a constraint. */
typedef int16_t  rd_idx_t;

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

/** What every function in the library returns. RD_OK is zero, errors negative. */
typedef enum {
    RD_OK                =  0,  /**< Success. */
    RD_ERR_NULL_PTR      = -1,  /**< A required pointer argument was NULL. */
    RD_ERR_INVALID_INDEX = -2,  /**< A frame, joint or method index is out of range. */
    RD_ERR_INVALID_FRAME = -3,  /**< ref_frame is not an rd_frame_t value. */
    RD_ERR_ALLOC_FAILED  = -4,  /**< rd_chain_build() could not allocate. */
    RD_ERR_SINGULAR      = -5,  /**< Cholesky met a non-positive pivot: the mass
                                 *   matrix or the constraint set is degenerate.
                                 *   Duplicated constraint rows are the usual
                                 *   cause; so is an armature of zero on a joint
                                 *   whose link has no inertia. */
    RD_ERR_INVALID_SIZE  = -6   /**< A buffer is smaller than the sizing helper
                                 *   asked for, or n_nodes is not positive. */
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
