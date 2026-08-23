/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file rd_cordic_stm32g4.h
 * @brief RD_SINCOS on the STM32G4's CORDIC coprocessor.
 *
 * Not part of the library. RobotDynamics ships one file and no vendor headers;
 * a backend is the caller's to own, and this is an example of writing one.
 * Copy it into your project if you want it:
 *
 *     #define RD_MATH_BACKEND "rd_cordic_stm32g4.h"
 *     ...
 *     rd_cordic_init();            // after the clock tree is up
 *     rd_chain_build(&model, &chain);
 *
 * Deliberately no vendor headers of its own either. The three registers it
 * needs are written out below, so it drops next to the single-header
 * distribution and compiles against nothing but the toolchain. A backend is a
 * header that defines RD_SINCOS and/or RD_SQRT, and that is the whole
 * contract.
 *
 * Parts: STM32G4 and STM32H7 carry this CORDIC. The register layout is the
 * same on both; only RD_CORDIC_BASE and the clock enable differ, and both are
 * overridable below.
 *
 * What it buys, measured on an STM32G474 at 170 MHz against the polynomial
 * this replaces:
 *
 *                        (sin, cos) pair    worst error over [-pi, pi]
 *     RD_FAST_TRIG            83 cyc              8.2e-08
 *     this backend            66 cyc              1.7e-06
 *
 * In situ that is 2 to 3% off rd_update_kinematics and 3 to 7% off
 * rd_fk_frame, and nothing anywhere else. **It costs about four and a half of
 * float32's twenty-four bits** -- the CORDIC's own floor, not the reduction's:
 * raising RD_CORDIC_PRECISION past 6 does not move the sine error at all and
 * makes the cosine worse. Take this backend when 1.7e-06 on a joint angle is
 * comfortably below your encoder and your model error, which for a servo robot
 * it is, and leave it alone when it is not. `make TRIG=1` in benchmark/stm32g4
 * prints the table above for whatever backend is compiled in.
 *
 * The gain is small because the pair is already cheap and this stays
 * synchronous: write the angle, stall on the result. The CORDIC is built to be
 * used the other way -- issue the next angle while the last result is being
 * consumed -- and a backend that owned the whole per-link loop could do that.
 * The RD_SINCOS contract is one angle in, two numbers out, and cannot.
 */

#ifndef RD_CORDIC_STM32G4_H
#define RD_CORDIC_STM32G4_H

#include <stdint.h>
#include <math.h>

/* Included from rd_math.h, so rd_config.h has already been read. The CORDIC is
 * a 32-bit fixed-point unit and has nothing to offer a double build. */
#if defined(RD_REAL_IS_FLOAT) && !RD_REAL_IS_FLOAT
#error "rd_cordic_stm32g4.h needs RD_USE_SINGLE_PRECISION=1"
#endif

/* --- The part ------------------------------------------------------------ */

#ifndef RD_CORDIC_BASE
#define RD_CORDIC_BASE      0x40020C00UL   /* AHB1, STM32G4 and STM32H7 */
#endif
#ifndef RD_CORDIC_RCC_ENR
#define RD_CORDIC_RCC_ENR   0x40021048UL   /* RCC_AHB1ENR on STM32G4 */
#endif
#ifndef RD_CORDIC_RCC_BIT
#define RD_CORDIC_RCC_BIT   (1UL << 3)     /* CORDICEN */
#endif

/* 4 x PRECISION iterations, and about PRECISION cycles. Six is the reset
 * default and already below a float32 ULP for sine and cosine. */
#ifndef RD_CORDIC_PRECISION
#define RD_CORDIC_PRECISION 6u
#endif

#define RD_CORDIC_CSR_   (*(volatile uint32_t*)(RD_CORDIC_BASE + 0x00UL))
#define RD_CORDIC_WDATA_ (*(volatile uint32_t*)(RD_CORDIC_BASE + 0x04UL))
#define RD_CORDIC_RDATA_ (*(volatile uint32_t*)(RD_CORDIC_BASE + 0x08UL))

/* FUNC = 0 (cosine, which also yields sine), SCALE = 0, two 32-bit results.
 * NARGS stays 0: only the angle is written per call, and the modulus keeps
 * whatever rd_cordic_init() left in it. */
#define RD_CORDIC_CSR_RUN_ \
    ((RD_CORDIC_PRECISION << 4) | (1UL << 19))
#define RD_CORDIC_CSR_INIT_ \
    (RD_CORDIC_CSR_RUN_ | (1UL << 20))     /* + NARGS: write the modulus too */

/**
 * @brief Turn the CORDIC on and load its unit modulus. Call once, after the
 *        clock tree is configured and before any dynamics call.
 */
static inline void rd_cordic_init(void) {
    *(volatile uint32_t*)RD_CORDIC_RCC_ENR |= RD_CORDIC_RCC_BIT;
    (void)*(volatile uint32_t*)RD_CORDIC_RCC_ENR;   /* RCC write barrier */

    /* One two-argument operation, purely to leave modulus = 1.0 in ARG2 where
     * every later single-argument call will read it. */
    RD_CORDIC_CSR_   = RD_CORDIC_CSR_INIT_;
    RD_CORDIC_WDATA_ = 0x00000000UL;                /* angle 0 */
    RD_CORDIC_WDATA_ = 0x7FFFFFFFUL;                /* modulus +1 in q1.31 */
    (void)RD_CORDIC_RDATA_;
    (void)RD_CORDIC_RDATA_;

    RD_CORDIC_CSR_ = RD_CORDIC_CSR_RUN_;
}

/**
 * @brief Sine and cosine of x radians.
 *
 * The CORDIC takes its angle in half-turns as q1.31, so the float argument is
 * scaled by 1/pi and folded onto [-1, 1) first -- which is the whole of the
 * range reduction, since the hardware is exact at the wrap. Reading RDATA
 * before the result is ready stalls the bus by design, so there is no polling
 * loop here and none is missing.
 */
static inline void rd_cordic_sincos(float x, float* s, float* c) {
    /* The same cut-off the polynomial backend uses, and for the same reason:
     * past it a float32 angle no longer carries enough bits below the radian
     * for any reduction to recover. Keeping the two identical here means
     * swapping one for the other changes speed and nothing else. */
    if (fabsf(x) > 4096.0f) { *s = sinf(x); *c = cosf(x); return; }

    /* Cody-Waite onto [-pi/2, pi/2], not a plain x/pi fold. Dividing first and
     * folding after leaves the residual carrying only the low bits of a number
     * as large as x/pi -- at x = 200 that is an ulp of 3.8e-06, and the answer
     * came out 1.4e-05 wrong. Subtracting k*pi in two pieces keeps the bits.
     * The half-turn the CORDIC wants is then a residual, always small. */
    const float MAGIC = 12582912.0f;                 /* 2^23 + 2^22 */
    const float k = (x * 0.318309886f + MAGIC) - MAGIC;   /* round(x/pi) */
    float r = x - k * 3.1415925026e+00f;             /* pi, high part */
    r       = r - k * 1.5099580253e-07f;             /* pi, low part  */

    /* r/pi in q1.31. r is at most pi/2, so this is at most 2^30 and the
     * conversion cannot overflow. */
    RD_CORDIC_WDATA_ = (uint32_t)(int32_t)(r * 683565248.0f);

    const int32_t rc = (int32_t)RD_CORDIC_RDATA_;    /* cos, q1.31 */
    const int32_t rs = (int32_t)RD_CORDIC_RDATA_;    /* sin, q1.31 */

    /* k odd means the residual came from the far half-turn. */
    const float sign = ((int32_t)k & 1) ? -(1.0f / 2147483648.0f)
                                        :  (1.0f / 2147483648.0f);
    *c = (float)rc * sign;
    *s = (float)rs * sign;
}

/* The contract: whichever of these a backend defines, rd_math.h uses. */
#define RD_SINCOS(x, sp, cp)  rd_cordic_sincos((x), (sp), (cp))

#endif /* RD_CORDIC_STM32G4_H */
