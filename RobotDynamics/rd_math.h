/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file rd_math.h
 * @brief Robot Dynamics Math Utilities
 * * Performance Optimized:
 * - Scalar: Inline wrappers for FPU/DSP.
 * - 3x3/4x4: Hand-unrolled loops.
 * - Spatial: Sparse algebra for O(n) recursions.
 * - Inertia: 6x6 matrix ops for CRBA.
 */

#ifndef RD_MATH_H
#define RD_MATH_H

#include "rd_config.h"
#include <math.h>
#include <string.h>

/* A board's own accelerator, ahead of everything that might defer to it.
 * See RD_MATH_BACKEND in rd_config.h. */
#ifdef RD_MATH_BACKEND
#include RD_MATH_BACKEND
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 1. Basic Scalar Math
 * ============================================================================ */

/* Dispatch on the configured precision. Using the float variants under
 * RD_USE_SINGLE_PRECISION=0 would silently round every transcendental down to
 * float and cap the library's accuracy at ~1e-7 regardless of rd_real_t. */
#if defined(RD_PLATFORM_ARM) && defined(__DSP_PRESENT) && (RD_USE_CMSIS_DSP)
    #include "arm_math.h"
    static RD_INLINE rd_real_t rd_sin(rd_real_t x) { return arm_sin_f32(x); }
    static RD_INLINE rd_real_t rd_cos(rd_real_t x) { return arm_cos_f32(x); }
    static RD_INLINE rd_real_t rd_sqrt_sw(rd_real_t x) { float r; arm_sqrt_f32(x, &r); return r; }
#elif RD_REAL_IS_FLOAT
    static RD_INLINE rd_real_t rd_sin(rd_real_t x) { return sinf(x); }
    static RD_INLINE rd_real_t rd_cos(rd_real_t x) { return cosf(x); }
    static RD_INLINE rd_real_t rd_sqrt_sw(rd_real_t x) { return sqrtf(x); }
#else
    static RD_INLINE rd_real_t rd_sin(rd_real_t x) { return sin(x); }
    static RD_INLINE rd_real_t rd_cos(rd_real_t x) { return cos(x); }
    static RD_INLINE rd_real_t rd_sqrt_sw(rd_real_t x) { return sqrt(x); }
#endif

/* A backend's RD_SQRT wins; otherwise whichever the chain above picked. On a
 * core with an FPU there is nothing here to accelerate -- sqrtf is one VSQRT
 * instruction -- but on one without, this is a soft-float call. */
#ifndef RD_SQRT
    #define RD_SQRT(x)  rd_sqrt_sw(x)
#endif
static RD_INLINE rd_real_t rd_sqrt(rd_real_t x) { return RD_SQRT(x); }

#if RD_REAL_IS_FLOAT
    static RD_INLINE rd_real_t rd_fabs(rd_real_t x) { return fabsf(x); }
    static RD_INLINE rd_real_t rd_acos(rd_real_t x) { return acosf(x); }
    static RD_INLINE rd_real_t rd_asin(rd_real_t x) { return asinf(x); }
    static RD_INLINE rd_real_t rd_atan2(rd_real_t y, rd_real_t x) { return atan2f(y, x); }
#else
    static RD_INLINE rd_real_t rd_fabs(rd_real_t x) { return fabs(x); }
    static RD_INLINE rd_real_t rd_acos(rd_real_t x) { return acos(x); }
    static RD_INLINE rd_real_t rd_asin(rd_real_t x) { return asin(x); }
    static RD_INLINE rd_real_t rd_atan2(rd_real_t y, rd_real_t x) { return atan2(y, x); }
#endif

/* One angle, both functions. Cycles for a (sin, cos) pair measured in situ on
 * an STM32G474 at 170 MHz -- inside rd_update_kinematics, not in a loop that
 * keeps the coefficients hot -- and worst-case absolute error against
 * double-precision libm over [-pi, pi]:
 *
 *     sinf + cosf     521 cyc   5.9e-08
 *     RD_FAST_TRIG    169 cyc   6.6e-08    default
 *
 * 76 of those 169 are the flash: the polynomial's ten coefficients are
 * literal-pool loads, and the G474's ART data cache is 128 bytes. From CCM
 * SRAM the pair costs 93. Worth 45% of rd_update_kinematics on Go2.
 *
 * Two alternatives are worse and should not be reached for: newlib's sincosf
 * is 313.4 cyc because it is `bl sinf; bl cosf` plus stack shuffling rather
 * than a fused routine, and CMSIS-DSP's arm_sin/cos_f32 is 105.5 cyc at
 * 1.9e-05, interpolating linearly between 512 table entries and giving up
 * accuracy float32 can resolve.
 *
 * A part with a trigonometric accelerator can displace this entirely: see
 * RD_MATH_BACKEND in rd_config.h, and backends/rd_cordic_stm32g4.h for one
 * that trades four bits for 20% of the pair.
 *
 * RD_FAST_TRIG is on by default; set it to 0 for libm's results. A backend's
 * RD_SINCOS displaces both. */
#if defined(RD_SINCOS)
static RD_INLINE void rd_sincos(rd_real_t x, rd_real_t* s, rd_real_t* c) {
    RD_SINCOS(x, s, c);
}
#elif RD_FAST_TRIG && RD_REAL_IS_FLOAT
/* Cody-Waite reduction onto [-pi/4, pi/4] plus a quadrant, then Taylor series
 * carried far enough that the truncation error sits under a float32 ULP. Joint
 * angles are bounded, so none of libm's Payne-Hanek machinery for huge
 * arguments is needed -- but a continuous joint can integrate without limit, so
 * anything outside the range the two-term pi/2 split can hold goes to libm. */
static RD_INLINE void rd_sincos(rd_real_t x, rd_real_t* s, rd_real_t* c) {
    if (rd_fabs(x) > 4096.0f) { *s = rd_sin(x); *c = rd_cos(x); return; }

    const float MAGIC = 12582912.0f;                /* 2^23 + 2^22 */
    float fn = (x * 0.636619772f + MAGIC) - MAGIC;  /* n = round(x * 2/pi) */
    int   n  = (int)fn;
    float r  = x - fn * 1.5707962513e+00f;          /* pi/2, high part */
    r        =   r - fn * 7.5497894159e-08f;        /* pi/2, low part  */

    float r2 = r * r;
    float sr = r * (1.0f + r2 * (-1.6666667e-01f + r2 * ( 8.3333337e-03f
                     + r2 * (-1.9841270e-04f + r2 * ( 2.7557319e-06f)))));
    float cr = 1.0f + r2 * (-5.0000000e-01f + r2 * ( 4.1666668e-02f
                     + r2 * (-1.3888889e-03f + r2 * ( 2.4801587e-05f))));

    switch (n & 3) {
        case 0:  *s =  sr; *c =  cr; break;
        case 1:  *s =  cr; *c = -sr; break;
        case 2:  *s = -sr; *c = -cr; break;
        default: *s = -cr; *c =  sr; break;
    }
}
#else
static RD_INLINE void rd_sincos(rd_real_t x, rd_real_t* s, rd_real_t* c) {
    /* Not sincosf: see the table above -- newlib's is slower than both calls. */
    *s = rd_sin(x);
    *c = rd_cos(x);
}
#endif

/*
 * Clear an array of rd_real_t.
 *
 * Deliberately not memset. The output buffers the algorithms clear -- a mass
 * matrix, a Jacobian, a torque vector -- have a size that is only known at run
 * time, so the compiler cannot inline the clear and has to call whatever libc
 * is linked. newlib-nano's memset, which is what --specs=nano.specs selects and
 * what STM32CubeIDE ships by default, is a byte-at-a-time loop: four
 * instructions per byte, so Go2's 18x18 mass matrix costs five thousand of
 * them. A typed loop is one store per element and does not depend on which
 * libc the caller happens to link.
 */
static RD_INLINE void rd_zero(rd_real_t* RD_RESTRICT p, rd_int_t n) {
    /* The zero arrives through a volatile read so the compiler cannot prove
     * the loop stores a constant byte pattern. Without that,
     * -ftree-loop-distribute-patterns recognises the idiom and turns this
     * straight back into the memset call it exists to avoid. */
    volatile rd_real_t vz = RD_REAL(0.0);
    const rd_real_t z = vz;
    for (rd_int_t i = 0; i < n; ++i) p[i] = z;
}

/* ============================================================================
 * 2. 3x3 Matrix Operations (Rotation & Vector)
 * ============================================================================ */

/* c = A * v */
static RD_INLINE void rd_mat3_vec(const rd_real_t* RD_RESTRICT A, 
                                  const rd_real_t* RD_RESTRICT v, 
                                  rd_real_t* RD_RESTRICT c) {
    rd_real_t v0 = v[0], v1 = v[1], v2 = v[2];
    c[0] = A[0]*v0 + A[1]*v1 + A[2]*v2;
    c[1] = A[3]*v0 + A[4]*v1 + A[5]*v2;
    c[2] = A[6]*v0 + A[7]*v1 + A[8]*v2;
}

/* C = A * B */
static RD_INLINE void rd_mat3_mul(const rd_real_t* RD_RESTRICT A, 
                                  const rd_real_t* RD_RESTRICT B, 
                                  rd_real_t* RD_RESTRICT C) {
    C[0] = A[0]*B[0] + A[1]*B[3] + A[2]*B[6];
    C[1] = A[0]*B[1] + A[1]*B[4] + A[2]*B[7];
    C[2] = A[0]*B[2] + A[1]*B[5] + A[2]*B[8];
    
    C[3] = A[3]*B[0] + A[4]*B[3] + A[5]*B[6];
    C[4] = A[3]*B[1] + A[4]*B[4] + A[5]*B[7];
    C[5] = A[3]*B[2] + A[4]*B[5] + A[5]*B[8];
    
    C[6] = A[6]*B[0] + A[7]*B[3] + A[8]*B[6];
    C[7] = A[6]*B[1] + A[7]*B[4] + A[8]*B[7];
    C[8] = A[6]*B[2] + A[7]*B[5] + A[8]*B[8];
}

/* S = skew(v) */
static RD_INLINE void rd_skew3(const rd_real_t v[3], rd_real_t S[9]) {
    S[0] = 0.0f;  S[1] = -v[2]; S[2] = v[1];
    S[3] = v[2];  S[4] = 0.0f;  S[5] = -v[0];
    S[6] = -v[1]; S[7] = v[0];  S[8] = 0.0f;
}

/* ============================================================================
 * 3. 4x4 Homogeneous Operations (Kinematics)
 * ============================================================================ */

/* Written out rather than memset plus four ones: sixteen floats is over
 * whatever threshold the compiler inlines a fixed-size clear at, so it emitted
 * a call -- and newlib-nano's memset is a byte-at-a-time loop. See rd_zero(). */
static RD_INLINE void rd_mat4_identity(rd_real_t T[16]) {
    const rd_real_t Z = RD_REAL(0.0), O = RD_REAL(1.0);
    T[0]  = O; T[1]  = Z; T[2]  = Z; T[3]  = Z;
    T[4]  = Z; T[5]  = O; T[6]  = Z; T[7]  = Z;
    T[8]  = Z; T[9]  = Z; T[10] = O; T[11] = Z;
    T[12] = Z; T[13] = Z; T[14] = Z; T[15] = O;
}

/* C = A * B */
static RD_INLINE void rd_mat4_mul(const rd_real_t* RD_RESTRICT A, 
                                  const rd_real_t* RD_RESTRICT B, 
                                  rd_real_t* RD_RESTRICT C) {
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            C[r + c*4] = A[r + 0*4]*B[0 + c*4] +
                         A[r + 1*4]*B[1 + c*4] +
                         A[r + 2*4]*B[2 + c*4] +
                         A[r + 3*4]*B[3 + c*4];
        }
    }
}

/* T = Translate(t) */
static RD_INLINE void rd_mat4_translate(const rd_real_t t[3], rd_real_t T[16]) {
    rd_mat4_identity(T);
    T[12] = t[0]; T[13] = t[1]; T[14] = t[2];
}

/*
 * C = A * B for SE(3) operands only.
 *
 * Every transform this library composes is a rigid motion, so the bottom row
 * is always [0 0 0 1] and a quarter of rd_mat4_mul's work goes into
 * rediscovering that. This does the rotation block and the translation
 * directly: 36 multiplies instead of 64.
 */
static RD_INLINE void rd_mat4_mul_se3(const rd_real_t* RD_RESTRICT A,
                                      const rd_real_t* RD_RESTRICT B,
                                      rd_real_t* RD_RESTRICT C) {
    for (int c = 0; c < 3; ++c) {
        const rd_real_t b0 = B[c*4 + 0], b1 = B[c*4 + 1], b2 = B[c*4 + 2];
        C[c*4 + 0] = A[0]*b0 + A[4]*b1 + A[8]*b2;
        C[c*4 + 1] = A[1]*b0 + A[5]*b1 + A[9]*b2;
        C[c*4 + 2] = A[2]*b0 + A[6]*b1 + A[10]*b2;
        C[c*4 + 3] = RD_REAL(0.0);
    }
    {
        const rd_real_t t0 = B[12], t1 = B[13], t2 = B[14];
        C[12] = A[0]*t0 + A[4]*t1 + A[8]*t2  + A[12];
        C[13] = A[1]*t0 + A[5]*t1 + A[9]*t2  + A[13];
        C[14] = A[2]*t0 + A[6]*t1 + A[10]*t2 + A[14];
        C[15] = RD_REAL(1.0);
    }
}

/*
 * C = A * (the joint's own motion transform), for a 1-DOF joint.
 *
 * The joint transform is never general. rd_axis_t admits only +-X, +-Y and
 * +-Z, so a revolute joint's matrix has one column equal to a basis vector
 * and the other two spanned by cos and sin, and a prismatic joint's is the
 * identity with one column's worth of translation. Building that 4x4 and
 * running the general compose spends 36 multiplies and 20 stores rediscovering
 * it. Composing directly is 12 multiplies and no joint matrix at all: the
 * rotation carries one column of A through untouched and mixes the other two,
 * and the translation is a copy.
 *
 * s_axis and s_sign are the motion subspace the chain already stores -- 3+k
 * for a revolute joint about axis k, k for a prismatic one along it.
 */
/*
 * The two shapes a 1-DOF joint's transform can take, with the axis a
 * compile-time constant. K is the axis, I and J the other two in cyclic order.
 *
 * The constant matters more than it looks: vldr and vstr take an immediate
 * offset and nothing else, so a run-time axis means six live base registers,
 * one per column touched. In a loop that already has the chain and state
 * pointers live there are none to spare, and GCC spends seventeen integer
 * instructions computing those addresses and spilling them, then reloads each
 * one before its float. With the axis constant every access is an immediate
 * offset from A or C.
 */
#define RD_JOINT_ROT_(K, I, J)                                              \
    do {                                                                    \
        C[(K)*4+0] = A[(K)*4+0];                                            \
        C[(K)*4+1] = A[(K)*4+1];                                            \
        C[(K)*4+2] = A[(K)*4+2];                                            \
        for (int r_ = 0; r_ < 3; ++r_) {                                    \
            const rd_real_t u_ = A[(I)*4+r_], w_ = A[(J)*4+r_];             \
            C[(I)*4+r_] = c*u_ + s*w_;                                      \
            C[(J)*4+r_] = c*w_ - s*u_;                                      \
        }                                                                   \
    } while (0)

#define RD_JOINT_TRANS_(K)                                                  \
    do {                                                                    \
        for (int m_ = 0; m_ < 3; ++m_) {                                    \
            C[m_*4+0] = A[m_*4+0];                                          \
            C[m_*4+1] = A[m_*4+1];                                          \
            C[m_*4+2] = A[m_*4+2];                                          \
        }                                                                   \
        t0 += d*A[(K)*4+0]; t1 += d*A[(K)*4+1]; t2 += d*A[(K)*4+2];         \
    } while (0)

/*
 * C = A * (the joint's own motion transform), for a 1-DOF joint.
 *
 * The joint transform is never general. rd_axis_t admits only +-X, +-Y and
 * +-Z, so a revolute joint's matrix carries one column of its parent's offset
 * through untouched and mixes the other two by cos and sin, and a prismatic
 * joint's is the identity with one column's worth of translation. Building
 * that 4x4 and running the general compose spends 36 multiplies and 20 stores
 * rediscovering it. Composing directly is 12 multiplies and no joint matrix.
 *
 * s_axis and s_sign are the motion subspace the chain already stores -- 3+k
 * for a revolute joint about axis k, k for a prismatic one along it.
 */
static RD_INLINE void rd_mat4_mul_joint(const rd_real_t* RD_RESTRICT A,
                                        rd_int_t s_axis, rd_real_t s_sign,
                                        rd_real_t q,
                                        rd_real_t* RD_RESTRICT C) {
    rd_real_t t0 = A[12], t1 = A[13], t2 = A[14];

    if (s_axis >= 3) {
        rd_real_t s, c;
        rd_sincos(q, &s, &c);
        s *= s_sign;
        switch (s_axis) {
            case 3:  RD_JOINT_ROT_(0, 1, 2); break;
            case 4:  RD_JOINT_ROT_(1, 2, 0); break;
            default: RD_JOINT_ROT_(2, 0, 1); break;
        }
    } else {
        /* Prismatic, and the do-nothing joints, whose sign is zero: the
         * rotation is the identity and only the translation moves. */
        const rd_real_t d = s_sign * q;
        switch (s_axis) {
            case 0:  RD_JOINT_TRANS_(0); break;
            case 1:  RD_JOINT_TRANS_(1); break;
            default: RD_JOINT_TRANS_(2); break;
        }
    }

    C[3] = C[7] = C[11] = RD_REAL(0.0);
    C[12] = t0; C[13] = t1; C[14] = t2; C[15] = RD_REAL(1.0);
}

/*
 * Only the two columns a revolute joint mixes, for a link whose T_dyn holds
 * exactly A * Rk(q).
 *
 * The other ten floats of that transform are constants: the axis column and
 * the translation come straight from the joint offset, and the bottom row is
 * the bottom row. rd_update_kinematics() writes them once when it first sees
 * the chain, so the per-tick store drops from sixteen floats to six and the
 * six loads that fed them go away too.
 */
#define RD_JOINT_COLS_(I, J)                                                \
    do {                                                                    \
        for (int r_ = 0; r_ < 3; ++r_) {                                    \
            const rd_real_t u_ = A[(I)*4+r_], w_ = A[(J)*4+r_];             \
            C[(I)*4+r_] = c*u_ + s*w_;                                      \
            C[(J)*4+r_] = c*w_ - s*u_;                                      \
        }                                                                   \
    } while (0)

static RD_INLINE void rd_mat4_joint_cols(const rd_real_t* RD_RESTRICT A,
                                         rd_int_t s_axis,
                                         rd_real_t s, rd_real_t c,
                                         rd_real_t* RD_RESTRICT C) {
    switch (s_axis) {
        case 3:  RD_JOINT_COLS_(1, 2); break;
        case 4:  RD_JOINT_COLS_(2, 0); break;
        default: RD_JOINT_COLS_(0, 1); break;
    }
}

/** The constant ten of that transform: everything rd_mat4_joint_cols() leaves
 *  alone. Written once per (chain, state) pairing. */
static RD_INLINE void rd_mat4_joint_fixed(const rd_real_t* RD_RESTRICT A,
                                          rd_int_t s_axis,
                                          rd_real_t* RD_RESTRICT C) {
    const rd_int_t k = s_axis - 3;
    C[k*4+0] = A[k*4+0]; C[k*4+1] = A[k*4+1]; C[k*4+2] = A[k*4+2];
    C[3] = C[7] = C[11] = RD_REAL(0.0);
    C[12] = A[12]; C[13] = A[13]; C[14] = A[14]; C[15] = RD_REAL(1.0);
}

#undef RD_JOINT_COLS_
#undef RD_JOINT_ROT_
#undef RD_JOINT_TRANS_

/* Ti = inv(T) for SE3 */
static RD_INLINE void rd_mat4_inv(const rd_real_t* RD_RESTRICT T,
                                  rd_real_t* RD_RESTRICT Ti) {
    rd_real_t R[9] = {T[0],T[4],T[8], T[1],T[5],T[9], T[2],T[6],T[10]};
    rd_real_t p[3] = {T[12], T[13], T[14]};
    
    rd_real_t ti[3]; // -R^T * p
    ti[0] = -(R[0]*p[0] + R[3]*p[1] + R[6]*p[2]);
    ti[1] = -(R[1]*p[0] + R[4]*p[1] + R[7]*p[2]);
    ti[2] = -(R[2]*p[0] + R[5]*p[1] + R[8]*p[2]);
    
    Ti[0]=R[0]; Ti[1]=R[1]; Ti[2]=R[2]; Ti[3]=0.0f;
    Ti[4]=R[3]; Ti[5]=R[4]; Ti[6]=R[5]; Ti[7]=0.0f;
    Ti[8]=R[6]; Ti[9]=R[7]; Ti[10]=R[8]; Ti[11]=0.0f;
    Ti[12]=ti[0]; Ti[13]=ti[1]; Ti[14]=ti[2]; Ti[15]=1.0f;
}

/* ============================================================================
 * 4. 6x6 Matrix Operations (REQUIRED FOR INERTIA / CRBA)
 * Note: Inertia matrices are 6x6 dense, so these ops are essential.
 * ============================================================================ */

static RD_INLINE void rd_mat6_vec(const rd_real_t* RD_RESTRICT A, 
                                  const rd_real_t* RD_RESTRICT v, 
                                  rd_real_t* RD_RESTRICT c) {
    for (int r = 0; r < 6; ++r) {
        const rd_real_t* row = &A[r*6];
        c[r] = row[0]*v[0] + row[1]*v[1] + row[2]*v[2] +
               row[3]*v[3] + row[4]*v[4] + row[5]*v[5];
    }
}

static RD_INLINE void rd_mat6_mul(const rd_real_t* RD_RESTRICT A, 
                                  const rd_real_t* RD_RESTRICT B, 
                                  rd_real_t* RD_RESTRICT C) {
    for (int r = 0; r < 6; ++r) {
        const rd_real_t* Arow = &A[r*6];
        for (int c = 0; c < 6; ++c) {
            rd_real_t sum = 0.0f;
            sum += Arow[0] * B[0*6+c];
            sum += Arow[1] * B[1*6+c];
            sum += Arow[2] * B[2*6+c];
            sum += Arow[3] * B[3*6+c];
            sum += Arow[4] * B[4*6+c];
            sum += Arow[5] * B[5*6+c];
            C[r*6+c] = sum;
        }
    }
}

static RD_INLINE void rd_mat6_add(const rd_real_t* A, const rd_real_t* B, rd_real_t* C) {
    for (int i = 0; i < 36; ++i) C[i] = A[i] + B[i];
}

/*
 * out = Rk(c,s) * C * Rk(c,s)^T, for a rotation about coordinate axis k.
 *
 * Such a rotation leaves row k and column k of C alone and mixes the other two
 * by the same 2x2, once on the rows and once on the columns: 24 multiplies
 * instead of 45, and the rotation is two numbers rather than nine. K is the
 * axis, I and J the other two -- all three compile-time constants, because a
 * run-time axis here would need a base register per row touched and cost more
 * than the multiplies it saves. That mistake was measured once already.
 */
#define RD_AXCONG_(K, I, J)                                                 \
    do {                                                                    \
        rd_real_t D_[9];                                                    \
        for (int b_ = 0; b_ < 3; ++b_) {                                    \
            const rd_real_t ci_ = C[(I)*3+b_], cj_ = C[(J)*3+b_];           \
            D_[(K)*3+b_] = C[(K)*3+b_];                                     \
            D_[(I)*3+b_] = c*ci_ - s*cj_;                                   \
            D_[(J)*3+b_] = s*ci_ + c*cj_;                                   \
        }                                                                   \
        for (int a_ = 0; a_ < 3; ++a_) {                                    \
            const rd_real_t di_ = D_[a_*3+(I)], dj_ = D_[a_*3+(J)];         \
            out[a_*3+(K)] = D_[a_*3+(K)];                                   \
            out[a_*3+(I)] = c*di_ - s*dj_;                                  \
            out[a_*3+(J)] = s*di_ + c*dj_;                                  \
        }                                                                   \
    } while (0)

static RD_INLINE void rd_axis_congruence3(rd_int_t k, rd_real_t c, rd_real_t s,
                                          const rd_real_t* RD_RESTRICT C,
                                          rd_real_t* RD_RESTRICT out) {
    switch (k) {
        case 0:  RD_AXCONG_(0, 1, 2); break;
        case 1:  RD_AXCONG_(1, 2, 0); break;
        default: RD_AXCONG_(2, 0, 1); break;
    }
}

#undef RD_AXCONG_

/*
 * The same rotation applied to a symmetric matrix, in and out packed as
 * {xx, yy, zz, xy, xz, yz}. Fifteen multiplies and six stores, against 24 and
 * nine for the general form -- worth having separately because the two callers
 * that carry an inertia never want the three entries symmetry already fixes,
 * and building the full 3x3 to throw half of it away is most of the cost.
 */
#define RD_SYMIDX_(A, B) ((A) == (B) ? (A) : ((A) + (B) == 1 ? 3 :            \
                                             ((A) + (B) == 2 ? 4 : 5)))
#define RD_AXCONGS_(K, I, J)                                                  \
    do {                                                                      \
        const rd_real_t ki_ = C[RD_SYMIDX_(K,I)], kj_ = C[RD_SYMIDX_(K,J)];   \
        const rd_real_t ii_ = C[RD_SYMIDX_(I,I)], jj_ = C[RD_SYMIDX_(J,J)];   \
        const rd_real_t ij_ = C[RD_SYMIDX_(I,J)];                             \
        const rd_real_t cc_ = c*c, ss_ = s*s, cs_ = c*s, cs2_ = cs_ + cs_;    \
        out[RD_SYMIDX_(K,K)] = C[RD_SYMIDX_(K,K)];                            \
        out[RD_SYMIDX_(K,I)] = c*ki_ - s*kj_;                                 \
        out[RD_SYMIDX_(K,J)] = s*ki_ + c*kj_;                                 \
        out[RD_SYMIDX_(I,I)] = cc_*ii_ - cs2_*ij_ + ss_*jj_;                  \
        out[RD_SYMIDX_(J,J)] = ss_*ii_ + cs2_*ij_ + cc_*jj_;                  \
        out[RD_SYMIDX_(I,J)] = cs_*(ii_ - jj_) + (cc_ - ss_)*ij_;             \
    } while (0)

static RD_INLINE void rd_axis_congruence3_sym(rd_int_t k, rd_real_t c, rd_real_t s,
                                              const rd_real_t* RD_RESTRICT C,
                                              rd_real_t* RD_RESTRICT out) {
    switch (k) {
        case 0:  RD_AXCONGS_(0, 1, 2); break;
        case 1:  RD_AXCONGS_(1, 2, 0); break;
        default: RD_AXCONGS_(2, 0, 1); break;
    }
}

#undef RD_AXCONGS_
#undef RD_SYMIDX_

/* ============================================================================
 * Compact spatial inertia
 *
 * A rigid body's spatial inertia is determined by ten numbers, not thirty-six:
 *
 *   I = [[ m*1,    -m[c]x ]      packed as { m, cx,cy,cz, Jxx,Jyy,Jzz,Jxy,Jxz,Jyz }
 *        [ m[c]x,   J     ]]     with J = Ic - m[c]x[c]x
 *
 * Multiplying by it in that form costs 27 multiplies and ten loads instead of
 * 36 and 36. On an in-order core running this code out of SRAM the load count
 * is what dominates, so this is the cheaper operation by a wide margin.
 * ============================================================================ */

#define RD_INERTIA_COMPACT_LEN 10

/* out = I * v, with v and out ordered [linear, angular]. */
/*
 * Rigid-body inertia in ten numbers: {m, h, J}, where h = m*c is the first
 * moment and J the second moment about the frame origin.
 *
 *   I = [[ m*1, -[h]x ], [ [h]x, J ]]
 *
 * CRBA's composite inertia is a sum of rigid-body inertias carried into a
 * common frame, and both summing and transforming preserve rigid-body form, so
 * the composite never needs the general 6x6 -- ten numbers describe it exactly.
 * That is 26 floats per node of traffic saved and a congruence of about 80
 * multiplies instead of 216.
 *
 *   m' = m
 *   h' = R h + m p
 *   J' = R J R^T + (2 p.Rh + m|p|^2) I - (Rh)p^T - p(Rh)^T - m p p^T
 *
 * T is the child expressed in the parent, so unlike the 6x6 congruence this
 * needs no inverse of it.
 *
 * axis_k >= 0 says T's rotation is about coordinate axis k and nothing else --
 * what T_dyn holds whenever a revolute joint's origin carries no rotation.
 * Then R J R^T costs 24 multiplies instead of 45, R h costs 4 instead of 9,
 * and R itself is two numbers off T rather than nine.
 */
static RD_INLINE void rd_rbi_congruence_accum(const rd_real_t* RD_RESTRICT T,
                                              rd_int_t axis_k,
                                              const rd_real_t* RD_RESTRICT in,
                                              rd_real_t* RD_RESTRICT accum) {
    const rd_real_t m = in[0];
    const rd_real_t q[3] = { T[12], T[13], T[14] };

    rd_real_t g[3];    /* R h */
    rd_real_t Jc[6];   /* R J R^T, packed xx yy zz xy xz yz */

    if (axis_k >= 0) {
        /*
         * One switch serves both products. The congruence's own switch folds
         * away because it is reached with a literal, so every subscript here
         * is a compile-time constant -- which is the whole point, and that
         * includes reading c and s out of T. The packing of in[4..9] is the
         * packing rd_axis_congruence3_sym wants, so J goes in and Jc comes out
         * with no 3x3 in between.
         */
        #define RD_RBI_AX_(K, P, Q)                                             \
            do {                                                                \
                const rd_real_t c = T[(P)*4+(P)], sn = T[(P)*4+(Q)];            \
                g[(K)] = in[1+(K)];                                             \
                g[(P)] = c*in[1+(P)] - sn*in[1+(Q)];                            \
                g[(Q)] = sn*in[1+(P)] + c*in[1+(Q)];                            \
                rd_axis_congruence3_sym((K), c, sn, in + 4, Jc);                \
            } while (0)
        switch (axis_k) {
            case 0:  RD_RBI_AX_(0, 1, 2); break;
            case 1:  RD_RBI_AX_(1, 2, 0); break;
            default: RD_RBI_AX_(2, 0, 1); break;
        }
        #undef RD_RBI_AX_
    } else {
        const rd_real_t R[9] = { T[0], T[4], T[8],
                                 T[1], T[5], T[9],
                                 T[2], T[6], T[10] };
        g[0] = R[0]*in[1] + R[1]*in[2] + R[2]*in[3];
        g[1] = R[3]*in[1] + R[4]*in[2] + R[5]*in[3];
        g[2] = R[6]*in[1] + R[7]*in[2] + R[8]*in[3];

        const rd_real_t Jin[9] = { in[4], in[7], in[8],
                                   in[7], in[5], in[9],
                                   in[8], in[9], in[6] };
        rd_real_t RJ[9];
        rd_mat3_mul(R, Jin, RJ);
        /* Only the six unique entries of the symmetric result are formed. */
        #define RD_RBI_J(a,b) (RJ[(a)*3+0]*R[(b)*3+0] + RJ[(a)*3+1]*R[(b)*3+1] \
                             + RJ[(a)*3+2]*R[(b)*3+2])
        Jc[0] = RD_RBI_J(0,0); Jc[1] = RD_RBI_J(1,1); Jc[2] = RD_RBI_J(2,2);
        Jc[3] = RD_RBI_J(0,1); Jc[4] = RD_RBI_J(0,2); Jc[5] = RD_RBI_J(1,2);
        #undef RD_RBI_J
    }

    accum[0] += m;
    accum[1] += g[0] + m*q[0];
    accum[2] += g[1] + m*q[1];
    accum[3] += g[2] + m*q[2];

    const rd_real_t d = RD_REAL(2.0)*(q[0]*g[0] + q[1]*g[1] + q[2]*g[2])
                      + m*(q[0]*q[0] + q[1]*q[1] + q[2]*q[2]);

    accum[4] += Jc[0] + d - RD_REAL(2.0)*g[0]*q[0] - m*q[0]*q[0];
    accum[5] += Jc[1] + d - RD_REAL(2.0)*g[1]*q[1] - m*q[1]*q[1];
    accum[6] += Jc[2] + d - RD_REAL(2.0)*g[2]*q[2] - m*q[2]*q[2];
    accum[7] += Jc[3] - g[0]*q[1] - q[0]*g[1] - m*q[0]*q[1];
    accum[8] += Jc[4] - g[0]*q[2] - q[0]*g[2] - m*q[0]*q[2];
    accum[9] += Jc[5] - g[1]*q[2] - q[1]*g[2] - m*q[1]*q[2];
}

/** Expand the ten-number form into a 6x6 block at an arbitrary row stride, so
 *  the mass matrix's base block can be written where it belongs rather than
 *  into a 36-float temporary and copied. Every entry is written, zeros
 *  included, so the destination need not be cleared first. */
static RD_INLINE void rd_rbi_to_mat6_stride(const rd_real_t* RD_RESTRICT in,
                                            rd_real_t* RD_RESTRICT M, rd_int_t s) {
    const rd_real_t m = in[0], hx = in[1], hy = in[2], hz = in[3];
    const rd_real_t hs[9] = { RD_REAL(0.0), -hz, hy,
                              hz, RD_REAL(0.0), -hx,
                              -hy, hx, RD_REAL(0.0) };
    for (rd_int_t r = 0; r < 3; ++r) {
        for (rd_int_t c = 0; c < 3; ++c) {
            M[r*s + c]         = (r == c) ? m : RD_REAL(0.0);
            M[r*s + (c+3)]     = -hs[r*3 + c];
            M[(r+3)*s + c]     =  hs[r*3 + c];
        }
    }
    M[3*s+3] = in[4]; M[4*s+4] = in[5]; M[5*s+5] = in[6];
    M[3*s+4] = M[4*s+3] = in[7];
    M[3*s+5] = M[5*s+3] = in[8];
    M[4*s+5] = M[5*s+4] = in[9];
}

static RD_INLINE void rd_spatial_inertia_mul(const rd_real_t* RD_RESTRICT ic,
                                             const rd_real_t* RD_RESTRICT v,
                                             rd_real_t* RD_RESTRICT out) {
    const rd_real_t m  = ic[0];
    const rd_real_t hx = ic[1], hy = ic[2], hz = ic[3];
    const rd_real_t vx = v[0], vy = v[1], vz = v[2];
    const rd_real_t wx = v[3], wy = v[4], wz = v[5];

    /* linear = m*v - h x w */
    out[0] = m*vx - (hy*wz - hz*wy);
    out[1] = m*vy - (hz*wx - hx*wz);
    out[2] = m*vz - (hx*wy - hy*wx);

    /* angular = h x v + J w */
    const rd_real_t Jxx = ic[4], Jyy = ic[5], Jzz = ic[6];
    const rd_real_t Jxy = ic[7], Jxz = ic[8], Jyz = ic[9];
    out[3] = (hy*vz - hz*vy) + Jxx*wx + Jxy*wy + Jxz*wz;
    out[4] = (hz*vx - hx*vz) + Jxy*wx + Jyy*wy + Jyz*wz;
    out[5] = (hx*vy - hy*vx) + Jxz*wx + Jyz*wy + Jzz*wz;
}

static RD_INLINE void rd_mat6_transpose(const rd_real_t* RD_RESTRICT A, 
                                        rd_real_t* RD_RESTRICT At) {
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            At[c*6+r] = A[r*6+c];
        }
    }
}

/**
 * @brief Adjoint matrix Ad(T) (6x6) - Required for Inertia Transformation
 * Ad(T) = [[R, [p]x R], [0, R]]
 */
static RD_INLINE void rd_spatial_adjoint(const rd_real_t T[16], rd_real_t Ad[36]) {
    rd_real_t R[9] = {T[0],T[4],T[8], T[1],T[5],T[9], T[2],T[6],T[10]};
    rd_real_t p[3] = {T[12], T[13], T[14]};
    rd_real_t px[9], pxR[9];
    
    rd_skew3(p, px);
    rd_mat3_mul(px, R, pxR);
    
    memset(Ad, 0, 36 * sizeof(rd_real_t));
    for(int r=0; r<3; r++) {
        for(int c=0; c<3; c++) {
            Ad[r*6 + c] = R[r*3+c];           // TL
            Ad[(r+3)*6 + (c+3)] = R[r*3+c];   // BR
            Ad[r*6 + (c+3)] = pxR[r*3+c];     // TR
        }
    }
}

static RD_INLINE void rd_mat3_mul_skew(const rd_real_t A[9], const rd_real_t p[3],
                                         rd_real_t out[9]) {
    for (int i = 0; i < 3; ++i) {
        const rd_real_t a0 = A[i*3+0], a1 = A[i*3+1], a2 = A[i*3+2];
        out[i*3+0] =  a1*p[2] - a2*p[1];
        out[i*3+1] = -a0*p[2] + a2*p[0];
        out[i*3+2] =  a0*p[1] - a1*p[0];
    }
}

/* out = [p]x * A, row-major 3x3. 18 mults. */
static RD_INLINE void rd_skew_mul_mat3(const rd_real_t p[3], const rd_real_t A[9],
                                         rd_real_t out[9]) {
    for (int j = 0; j < 3; ++j) {
        const rd_real_t a0 = A[0*3+j], a1 = A[1*3+j], a2 = A[2*3+j];
        out[0*3+j] = -p[2]*a1 + p[1]*a2;
        out[1*3+j] =  p[2]*a0 - p[0]*a2;
        out[2*3+j] = -p[1]*a0 + p[0]*a1;
    }
}

/* out = R^T * C * R, row-major 3x3. */
static RD_INLINE void rd_congruence3(const rd_real_t R[9], const rd_real_t C[9],
                                       rd_real_t out[9]) {
    rd_real_t CR[9];
    rd_mat3_mul(C, R, CR);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out[i*3+j] = R[0*3+i]*CR[0*3+j]
                       + R[1*3+i]*CR[1*3+j]
                       + R[2*3+i]*CR[2*3+j];
        }
    }
}

/* dst[3x3 block at stride 6] += R^T * C * R, without materialising the result. */
static RD_INLINE void rd_congruence3_accum(const rd_real_t R[9], const rd_real_t C[9],
                                             rd_real_t* dst) {
    rd_real_t CR[9];
    rd_mat3_mul(C, R, CR);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            dst[i*6+j] += R[0*3+i]*CR[0*3+j]
                        + R[1*3+i]*CR[1*3+j]
                        + R[2*3+i]*CR[2*3+j];
        }
    }
}

/*
 * I_accum += X^T * I_in * X, with X = Ad(T).
 *
 * Callers pass state->Ti, the pose of the parent expressed in the child, which
 * makes X the motion transform from the parent frame into the child's. That is
 * the direction the composite/articulated inertia congruence needs.
 *
 * This is the hot spot of both rd_crba() and rd_aba(), so it is written against
 * the structure rather than as two dense 6x6 products. Writing
 * I = [[A11, A12], [A21, A22]] and X = [[R, [p]x R], [0, R]], the congruence is
 *
 *   TL = R^T A11 R
 *   TR = R^T (A11 P + A12) R                       P = [p]x
 *   BL = TR^T
 *   BR = R^T (A22 + A21 P + (A21 P)^T - P A11 P) R
 *
 * which needs 216 multiplies instead of 432, skips X's 3x3 zero block entirely,
 * gets the lower-left triangle for free from symmetry, and never materialises a
 * 6x6 temporary.
 */
/* ---------------------------------------------------------------------------
 * Articulated-body inertia, packed.
 *
 * ABA's articulated inertia stops being a rigid body after the rank-1 downdate,
 * so unlike CRBA's composite it needs a general 6x6 -- but it stays symmetric
 * throughout, so it needs 21 numbers and not 36. The blocks are stored the way
 * every operation on them wants them:
 *
 *   [0..5]    A11, symmetric: 00 01 02 11 12 22
 *   [6..14]   A12, full 3x3 row-major
 *   [15..20]  A22, symmetric: 00 01 02 11 12 22
 *
 * A21 is A12 transposed and is never stored. On a memory-bound core that is
 * 15 fewer floats read and written per node per pass.
 * ------------------------------------------------------------------------ */
#define RD_ABI_LEN 21

/** Seed from a rigid-body inertia in the ten-number form. */
static RD_INLINE void rd_abi_from_rbi(const rd_real_t* RD_RESTRICT ic,
                                      rd_real_t* RD_RESTRICT out) {
    const rd_real_t m = ic[0], hx = ic[1], hy = ic[2], hz = ic[3];
    out[0] = m;  out[1] = RD_REAL(0.0); out[2] = RD_REAL(0.0);
    out[3] = m;  out[4] = RD_REAL(0.0); out[5] = m;
    /* A12 = -[h]x */
    out[6]  = RD_REAL(0.0); out[7]  =  hz;           out[8]  = -hy;
    out[9]  = -hz;          out[10] = RD_REAL(0.0);  out[11] =  hx;
    out[12] =  hy;          out[13] = -hx;           out[14] = RD_REAL(0.0);
    out[15] = ic[4]; out[16] = ic[7]; out[17] = ic[8];
    out[18] = ic[5]; out[19] = ic[9]; out[20] = ic[6];
}

/** out = I * v, spatial vectors ordered [linear, angular]. */
static RD_INLINE void rd_abi_mul(const rd_real_t* RD_RESTRICT I,
                                 const rd_real_t* RD_RESTRICT v,
                                 rd_real_t* RD_RESTRICT out) {
    const rd_real_t a = v[0], b = v[1], c = v[2];
    const rd_real_t d = v[3], e = v[4], f = v[5];
    out[0] = I[0]*a + I[1]*b + I[2]*c + I[6]*d  + I[7]*e  + I[8]*f;
    out[1] = I[1]*a + I[3]*b + I[4]*c + I[9]*d  + I[10]*e + I[11]*f;
    out[2] = I[2]*a + I[4]*b + I[5]*c + I[12]*d + I[13]*e + I[14]*f;
    out[3] = I[6]*a + I[9]*b + I[12]*c + I[15]*d + I[16]*e + I[17]*f;
    out[4] = I[7]*a + I[10]*b + I[13]*c + I[16]*d + I[18]*e + I[19]*f;
    out[5] = I[8]*a + I[11]*b + I[14]*c + I[17]*d + I[19]*e + I[20]*f;
}

/** acc += I * v. The bias force is always accumulated, never assigned, so the
 *  six-float result never has to exist. */
static RD_INLINE void rd_abi_mul_accum(const rd_real_t* RD_RESTRICT I,
                                       const rd_real_t* RD_RESTRICT v,
                                       rd_real_t* RD_RESTRICT acc) {
    const rd_real_t a = v[0], b = v[1], c = v[2];
    const rd_real_t d = v[3], e = v[4], f = v[5];
    acc[0] += I[0]*a + I[1]*b + I[2]*c  + I[6]*d  + I[7]*e  + I[8]*f;
    acc[1] += I[1]*a + I[3]*b + I[4]*c  + I[9]*d  + I[10]*e + I[11]*f;
    acc[2] += I[2]*a + I[4]*b + I[5]*c  + I[12]*d + I[13]*e + I[14]*f;
    acc[3] += I[6]*a + I[9]*b + I[12]*c + I[15]*d + I[16]*e + I[17]*f;
    acc[4] += I[7]*a + I[10]*b + I[13]*c + I[16]*d + I[18]*e + I[19]*f;
    acc[5] += I[8]*a + I[11]*b + I[14]*c + I[17]*d + I[19]*e + I[20]*f;
}

/** I -= (U U^T) * inv_d, the articulated-body rank-1 downdate. */
static RD_INLINE void rd_abi_rank1_sub(rd_real_t* RD_RESTRICT I,
                                       const rd_real_t* RD_RESTRICT U,
                                       rd_real_t inv_d) {
    const rd_real_t u0 = U[0]*inv_d, u1 = U[1]*inv_d, u2 = U[2]*inv_d;
    const rd_real_t u3 = U[3]*inv_d, u4 = U[4]*inv_d, u5 = U[5]*inv_d;
    I[0] -= u0*U[0]; I[1] -= u0*U[1]; I[2] -= u0*U[2];
    I[3] -= u1*U[1]; I[4] -= u1*U[2]; I[5] -= u2*U[2];
    I[6]  -= u0*U[3]; I[7]  -= u0*U[4]; I[8]  -= u0*U[5];
    I[9]  -= u1*U[3]; I[10] -= u1*U[4]; I[11] -= u1*U[5];
    I[12] -= u2*U[3]; I[13] -= u2*U[4]; I[14] -= u2*U[5];
    I[15] -= u3*U[3]; I[16] -= u3*U[4]; I[17] -= u3*U[5];
    I[18] -= u4*U[4]; I[19] -= u4*U[5]; I[20] -= u5*U[5];
}

/** Expand to a full row-major 6x6, for the floating base's 6x6 solve. Written
 *  out rather than looped over index tables: the subscripts are constants, and
 *  a loop turns each of them into an integer load. */
static RD_INLINE void rd_abi_to_mat6(const rd_real_t* RD_RESTRICT I,
                                     rd_real_t* RD_RESTRICT M) {
    M[0] =I[0]; M[1] =I[1];  M[2] =I[2];  M[3] =I[6];  M[4] =I[7];  M[5] =I[8];
    M[6] =I[1]; M[7] =I[3];  M[8] =I[4];  M[9] =I[9];  M[10]=I[10]; M[11]=I[11];
    M[12]=I[2]; M[13]=I[4];  M[14]=I[5];  M[15]=I[12]; M[16]=I[13]; M[17]=I[14];
    M[18]=I[6]; M[19]=I[9];  M[20]=I[12]; M[21]=I[15]; M[22]=I[16]; M[23]=I[17];
    M[24]=I[7]; M[25]=I[10]; M[26]=I[13]; M[27]=I[16]; M[28]=I[18]; M[29]=I[19];
    M[30]=I[8]; M[31]=I[11]; M[32]=I[14]; M[33]=I[17]; M[34]=I[19]; M[35]=I[20];
}

/** dst += the six unique entries of R^T C R, for symmetric C. */
static RD_INLINE void rd_congruence3_sym_accum(const rd_real_t R[9],
                                               const rd_real_t C[9],
                                               rd_real_t* RD_RESTRICT dst) {
    rd_real_t CR[9];
    rd_mat3_mul(C, R, CR);
    static const int oi[6] = {0,0,0,1,1,2};
    static const int oj[6] = {0,1,2,1,2,2};
    for (int k = 0; k < 6; ++k) {
        const int i = oi[k], j = oj[k];
        dst[k] += R[0*3+i]*CR[0*3+j] + R[1*3+i]*CR[1*3+j] + R[2*3+i]*CR[2*3+j];
    }
}

/*
 * Column extraction.
 *
 * rd_axis_t can only hold +/-X, +/-Y or +/-Z, and the link frame is the
 * joint's child frame, so the motion subspace is always a unit spatial axis:
 * exactly one component, equal to +/-1. Every I*S product in RNEA, CRBA and ABA
 * is therefore one column of an inertia, not a matrix-vector product. No test
 * is needed -- the model type cannot express anything else.
 */
/* Top three from the symmetric A11 or from A12, bottom three from A12's row
 * or from the symmetric A22 -- written out per axis so every subscript is a
 * constant, rather than read out of an index table at run time. */
#define RD_ABI_COL_(T0, T1, T2, B0, B1, B2)                                 \
    do {                                                                    \
        out[0] = sgn * I[T0]; out[1] = sgn * I[T1]; out[2] = sgn * I[T2];   \
        out[3] = sgn * I[B0]; out[4] = sgn * I[B1]; out[5] = sgn * I[B2];   \
    } while (0)

static RD_INLINE void rd_abi_col(const rd_real_t* RD_RESTRICT I, int k,
                                 rd_real_t sgn, rd_real_t* RD_RESTRICT out) {
    switch (k) {
        case 0:  RD_ABI_COL_( 0,  1,  2,  6,  7,  8); break;
        case 1:  RD_ABI_COL_( 1,  3,  4,  9, 10, 11); break;
        case 2:  RD_ABI_COL_( 2,  4,  5, 12, 13, 14); break;
        case 3:  RD_ABI_COL_( 6,  9, 12, 15, 16, 17); break;
        case 4:  RD_ABI_COL_( 7, 10, 13, 16, 18, 19); break;
        default: RD_ABI_COL_( 8, 11, 14, 17, 19, 20); break;
    }
}

#undef RD_ABI_COL_

/** The same, for a rigid-body inertia in the ten-number {m, h, J} form. */
static RD_INLINE void rd_rbi_col(const rd_real_t* RD_RESTRICT ic, int k,
                                 rd_real_t sgn, rd_real_t* RD_RESTRICT out) {
    const rd_real_t m = ic[0], hx = ic[1], hy = ic[2], hz = ic[3];
    if (k < 3) {
        /* top = m e_k, bottom = [h]x e_k */
        out[0] = (k == 0) ? sgn*m : RD_REAL(0.0);
        out[1] = (k == 1) ? sgn*m : RD_REAL(0.0);
        out[2] = (k == 2) ? sgn*m : RD_REAL(0.0);
        if (k == 0)      { out[3] = RD_REAL(0.0); out[4] =  sgn*hz; out[5] = -sgn*hy; }
        else if (k == 1) { out[3] = -sgn*hz; out[4] = RD_REAL(0.0); out[5] =  sgn*hx; }
        else             { out[3] =  sgn*hy; out[4] = -sgn*hx; out[5] = RD_REAL(0.0); }
    } else {
        const int j = k - 3;
        /* top = -[h]x e_j, bottom = J e_j */
        if (j == 0)      { out[0] = RD_REAL(0.0); out[1] = -sgn*hz; out[2] =  sgn*hy;
                           out[3] = sgn*ic[4]; out[4] = sgn*ic[7]; out[5] = sgn*ic[8]; }
        else if (j == 1) { out[0] =  sgn*hz; out[1] = RD_REAL(0.0); out[2] = -sgn*hx;
                           out[3] = sgn*ic[7]; out[4] = sgn*ic[5]; out[5] = sgn*ic[9]; }
        else             { out[0] = -sgn*hy; out[1] =  sgn*hx; out[2] = RD_REAL(0.0);
                           out[3] = sgn*ic[8]; out[4] = sgn*ic[9]; out[5] = sgn*ic[6]; }
    }
}

/* out = R * C * R^T, row-major 3x3, for a general rotation. */
static RD_INLINE void rd_congruence3_rt(const rd_real_t R[9], const rd_real_t C[9],
                                        rd_real_t out[9]) {
    rd_real_t RC[9];
    rd_mat3_mul(R, C, RC);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out[i*3+j] = RC[i*3+0]*R[j*3+0] + RC[i*3+1]*R[j*3+1] + RC[i*3+2]*R[j*3+2];
        }
    }
}

/* out = A^T * [q]x, row-major 3x3. 18 mults, no transpose materialised. */
static RD_INLINE void rd_mat3T_mul_skew(const rd_real_t A[9], const rd_real_t q[3],
                                        rd_real_t out[9]) {
    for (int i = 0; i < 3; ++i) {
        const rd_real_t a0 = A[0*3+i], a1 = A[1*3+i], a2 = A[2*3+i];
        out[i*3+0] =  a1*q[2] - a2*q[1];
        out[i*3+1] = -a0*q[2] + a2*q[0];
        out[i*3+2] =  a0*q[1] - a1*q[0];
    }
}

/*
 * accum += Ad(inv(T))^T I Ad(inv(T)), all in packed form -- the articulated
 * inertia of a child, added to its parent's, given the child's pose in the
 * parent.
 *
 * Rotation first, then translation. Conjugating by the rotation turns the
 * inverse's translation, -R^T t, into plain -t: rotate the three blocks first
 * and the skew matrix the translation contributes is the transform's own
 * translation negated, with nothing left to compute. Three congruences and
 * four skew products, and A21 comes from A12 transposed so the lower-left is
 * never formed.
 *
 * axis_k >= 0 says the rotation is about coordinate axis k and nothing else --
 * what T_dyn holds whenever a revolute joint's origin carries no rotation,
 * which is the ordinary case in a URDF and true of every joint on a quadruped.
 * Then the three congruences cost 24 multiplies each instead of 45.
 */
static RD_INLINE void rd_abi_congruence_accum(const rd_real_t* RD_RESTRICT T,
                                              rd_int_t axis_k,
                                              const rd_real_t* RD_RESTRICT I_in,
                                              rd_real_t* RD_RESTRICT accum) {
    rd_real_t TL[9], G[9], H[9];
    {
        rd_real_t A[9];
        A[0]=I_in[0]; A[1]=I_in[1]; A[2]=I_in[2];
        A[3]=I_in[1]; A[4]=I_in[3]; A[5]=I_in[4];
        A[6]=I_in[2]; A[7]=I_in[4]; A[8]=I_in[5];
        rd_real_t B[9];
        B[0]=I_in[15]; B[1]=I_in[16]; B[2]=I_in[17];
        B[3]=I_in[16]; B[4]=I_in[18]; B[5]=I_in[19];
        B[6]=I_in[17]; B[7]=I_in[19]; B[8]=I_in[20];

        if (axis_k >= 0) {
            /* c and s sit at compile-time offsets in T once the axis is a
             * literal: for a rotation about K they are its (P,P) and (Q,P)
             * entries. Reaching them through a computed index instead costs
             * six integer instructions per node. */
            #define RD_ABI_AX_(K, P, Q)                                     \
                do {                                                        \
                    const rd_real_t c = T[(P)*4+(P)], sn = T[(P)*4+(Q)];    \
                    rd_axis_congruence3((K), c, sn, A, TL);                 \
                    rd_axis_congruence3((K), c, sn, I_in + 6, G);           \
                    rd_axis_congruence3((K), c, sn, B, H);                  \
                } while (0)
            switch (axis_k) {
                case 0:  RD_ABI_AX_(0, 1, 2); break;
                case 1:  RD_ABI_AX_(1, 2, 0); break;
                default: RD_ABI_AX_(2, 0, 1); break;
            }
            #undef RD_ABI_AX_
        } else {
            const rd_real_t R[9] = { T[0], T[4], T[8],
                                     T[1], T[5], T[9],
                                     T[2], T[6], T[10] };
            rd_congruence3_rt(R, A, TL);
            rd_congruence3_rt(R, I_in + 6, G);
            rd_congruence3_rt(R, B, H);
        }
    }

    accum[0] += TL[0]; accum[1] += TL[1]; accum[2] += TL[2];
    accum[3] += TL[4]; accum[4] += TL[5]; accum[5] += TL[8];

    const rd_real_t q[3] = { -T[12], -T[13], -T[14] };

    {   /* TR = TL Q + G,  Q = [q]x */
        rd_real_t TLQ[9];
        rd_mat3_mul_skew(TL, q, TLQ);
        for (int i = 0; i < 9; ++i) accum[6 + i] += TLQ[i] + G[i];
    }
    {   /* BR = H + G^T Q + (G^T Q)^T - Q TL Q */
        rd_real_t K[9], QT[9], QTQ[9];
        rd_mat3T_mul_skew(G, q, K);
        rd_skew_mul_mat3(q, TL, QT);
        rd_mat3_mul_skew(QT, q, QTQ);
        accum[15] += H[0] + K[0] + K[0] - QTQ[0];
        accum[16] += H[1] + K[1] + K[3] - QTQ[1];
        accum[17] += H[2] + K[2] + K[6] - QTQ[2];
        accum[18] += H[4] + K[4] + K[4] - QTQ[4];
        accum[19] += H[5] + K[5] + K[7] - QTQ[5];
        accum[20] += H[8] + K[8] + K[8] - QTQ[8];
    }
}

static RD_INLINE void rd_spatial_inertia_congruence(const rd_real_t* RD_RESTRICT T,
                                              const rd_real_t* RD_RESTRICT I_in,
                                              rd_real_t* RD_RESTRICT I_accum) {
    /* R row-major, p, out of the column-major T */
    const rd_real_t R[9] = { T[0], T[4], T[8],
                             T[1], T[5], T[9],
                             T[2], T[6], T[10] };
    const rd_real_t p[3] = { T[12], T[13], T[14] };

    /*
     * Only three 3x3 temporaries live at once. The obvious version -- pull
     * A11/A12/A21/A22 out, build C12/C22, then materialise S11/S12/S22 --
     * needs twelve, which is more than the M33's 32 single-precision registers
     * can hold, so it spills to stack and reloads. On a core where this code is
     * memory-bound rather than multiply-bound, that costs more than the
     * multiplies saved.
     */
    rd_real_t A11[9], C[9], S12[9];

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) A11[i*3+j] = I_in[i*6 + j];
    }

    /* TL += R^T A11 R */
    rd_congruence3_accum(R, A11, I_accum);

    /* C = A11 P + A12,   then TR += R^T C R  and  BL += (that)^T */
    rd_mat3_mul_skew(A11, p, C);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) C[i*3+j] += I_in[i*6 + 3 + j];
    }
    rd_congruence3(R, C, S12);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            I_accum[i*6 + 3 + j] += S12[i*3+j];
            I_accum[(i+3)*6 + j] += S12[j*3+i];        /* BL = TR^T */
        }
    }

    /* C = A22 + A21 P + (A21 P)^T - P A11 P,  then BR += R^T C R */
    {
        rd_real_t t[9];
        rd_skew_mul_mat3(p, A11, t);                 /* P A11   */
        rd_mat3_mul_skew(t, p, C);                   /* P A11 P */
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                t[i*3+j] = I_in[(i+3)*6 + j];          /* A21     */
            }
        }
        rd_real_t A21P[9];
        rd_mat3_mul_skew(t, p, A21P);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                C[i*3+j] = I_in[(i+3)*6 + 3 + j] + A21P[i*3+j] + A21P[j*3+i]
                         - C[i*3+j];
            }
        }
    }
    rd_congruence3_accum(R, C, I_accum + 3*6 + 3);
}

/*
 * Velocity of a link relative to its parent, i.e. S * qd.
 *
 * This is v_i - Ad(Ti) v_parent, but rd_update_kinematics() already formed it
 * on the way to v_i, so the joint velocity it used is cached rather than the
 * transform being run a second time. RNEA, ABA and rd_spatial_acceleration()
 * each want this once per link.
 */

/* ============================================================================
 * 5. Sparse Spatial Algebra (High-Performance RNEA/Jacobian)
 * ============================================================================ */

/**
 * @brief Sparse Motion Transform: v_A = Ad(T_AB) * v_B
 */
static RD_INLINE void rd_spatial_transform_motion(const rd_real_t* RD_RESTRICT T,
                                                  const rd_real_t* RD_RESTRICT v_in,
                                                  rd_real_t* RD_RESTRICT v_out) {
    /* R = T(0..2, 0..2) [row-major], p = T(0..2, 3) */
    rd_real_t wx = v_in[3], wy = v_in[4], wz = v_in[5];
    rd_real_t vx = v_in[0], vy = v_in[1], vz = v_in[2];

    /* w_out = R * w_in */
    rd_real_t w_rot_x = T[0]*wx + T[4]*wy + T[8]*wz;
    rd_real_t w_rot_y = T[1]*wx + T[5]*wy + T[9]*wz;
    rd_real_t w_rot_z = T[2]*wx + T[6]*wy + T[10]*wz;
    v_out[3] = w_rot_x; v_out[4] = w_rot_y; v_out[5] = w_rot_z;

    /* v_out = R*v_in + p x w_out */
    rd_real_t v_rot_x = T[0]*vx + T[4]*vy + T[8]*vz;
    rd_real_t v_rot_y = T[1]*vx + T[5]*vy + T[9]*vz;
    rd_real_t v_rot_z = T[2]*vx + T[6]*vy + T[10]*vz;

    rd_real_t px = T[12], py = T[13], pz = T[14];
    v_out[0] = v_rot_x + (py*w_rot_z - pz*w_rot_y);
    v_out[1] = v_rot_y + (pz*w_rot_x - px*w_rot_z);
    v_out[2] = v_rot_z + (px*w_rot_y - py*w_rot_x);
}

/*
 * out = Ad(T) applied to a unit spatial axis.
 *
 * A geometric Jacobian's columns are exactly this: the joint's own motion
 * subspace, mapped into the reference frame. S is a unit spatial axis, so the
 * rotated half is one column of T read straight out and the general 24
 * multiplies collapse to nine -- or three, for a prismatic joint, whose column
 * has no angular part to cross the translation with.
 */
static RD_INLINE void rd_spatial_transform_motion_axis(const rd_real_t* RD_RESTRICT T,
                                                       rd_int_t s_axis, rd_real_t sgn,
                                                       rd_real_t* RD_RESTRICT v_out) {
    const rd_int_t j = (s_axis >= 3) ? (s_axis - 3) : s_axis;
    const rd_real_t cx = sgn*T[4*j + 0], cy = sgn*T[4*j + 1], cz = sgn*T[4*j + 2];

    if (s_axis >= 3) {
        const rd_real_t px = T[12], py = T[13], pz = T[14];
        v_out[0] = py*cz - pz*cy;
        v_out[1] = pz*cx - px*cz;
        v_out[2] = px*cy - py*cx;
        v_out[3] = cx; v_out[4] = cy; v_out[5] = cz;
    } else {
        v_out[0] = cx; v_out[1] = cy; v_out[2] = cz;
        v_out[3] = v_out[4] = v_out[5] = RD_REAL(0.0);
    }
}

/* Motion transform by the inverse of T, without forming the inverse.
 *
 *   rd_spatial_transform_motion_inv(T, v, out) == rd_spatial_transform_motion(inv(T), v, out)
 *
 * Same operation count as the forward version, so a caller holding only the
 * child-in-parent transform does not have to store the other direction too.
 *
 * Specialising this one on an axis-aligned rotation, the way the inertia
 * congruences are, was measured and reverted: it saves ten multiplies out of
 * forty-five, and the second kernel takes enough registers from the general
 * one to cost a robot without axis-aligned joints 4% of its kinematics. */
static RD_INLINE void rd_spatial_transform_motion_inv(const rd_real_t* RD_RESTRICT T,
                                                      const rd_real_t* RD_RESTRICT v_in,
                                                      rd_real_t* RD_RESTRICT v_out) {
    rd_real_t wx = v_in[3], wy = v_in[4], wz = v_in[5];
    rd_real_t px = T[12], py = T[13], pz = T[14];

    /* v_in - p x w, before rotating back */
    rd_real_t lx = v_in[0] - (py*wz - pz*wy);
    rd_real_t ly = v_in[1] - (pz*wx - px*wz);
    rd_real_t lz = v_in[2] - (px*wy - py*wx);

    /* R^T applied to both halves */
    v_out[0] = T[0]*lx + T[1]*ly + T[2]*lz;
    v_out[1] = T[4]*lx + T[5]*ly + T[6]*lz;
    v_out[2] = T[8]*lx + T[9]*ly + T[10]*lz;
    v_out[3] = T[0]*wx + T[1]*wy + T[2]*wz;
    v_out[4] = T[4]*wx + T[5]*wy + T[6]*wz;
    v_out[5] = T[8]*wx + T[9]*wy + T[10]*wz;
}

/* The same, in place. CRBA's inner walk carries one force up the ancestry and
 * has no use for the previous value, so a separate destination is a six-float
 * temporary and a copy per step. */
static RD_INLINE void rd_spatial_transform_force_inplace(const rd_real_t* RD_RESTRICT T,
                                                         rd_real_t* RD_RESTRICT f) {
    rd_real_t fx = f[0], fy = f[1], fz = f[2];
    rd_real_t tx = f[3], ty = f[4], tz = f[5];

    rd_real_t f_rot_x = T[0]*fx + T[4]*fy + T[8]*fz;
    rd_real_t f_rot_y = T[1]*fx + T[5]*fy + T[9]*fz;
    rd_real_t f_rot_z = T[2]*fx + T[6]*fy + T[10]*fz;

    rd_real_t t_rot_x = T[0]*tx + T[4]*ty + T[8]*tz;
    rd_real_t t_rot_y = T[1]*tx + T[5]*ty + T[9]*tz;
    rd_real_t t_rot_z = T[2]*tx + T[6]*ty + T[10]*tz;

    rd_real_t px = T[12], py = T[13], pz = T[14];
    f[0] = f_rot_x; f[1] = f_rot_y; f[2] = f_rot_z;
    f[3] = t_rot_x + (py*f_rot_z - pz*f_rot_y);
    f[4] = t_rot_y + (pz*f_rot_x - px*f_rot_z);
    f[5] = t_rot_z + (px*f_rot_y - py*f_rot_x);
}

/* The same, accumulated. Every caller in the library adds the transformed
 * force into a running total, so the six-float temporary and the read-back it
 * costs are pure overhead. */
static RD_INLINE void rd_spatial_transform_force_accum(const rd_real_t* RD_RESTRICT T,
                                                       const rd_real_t* RD_RESTRICT f_in,
                                                       rd_real_t* RD_RESTRICT f_acc) {
    rd_real_t fx = f_in[0], fy = f_in[1], fz = f_in[2];
    rd_real_t tx = f_in[3], ty = f_in[4], tz = f_in[5];

    rd_real_t f_rot_x = T[0]*fx + T[4]*fy + T[8]*fz;
    rd_real_t f_rot_y = T[1]*fx + T[5]*fy + T[9]*fz;
    rd_real_t f_rot_z = T[2]*fx + T[6]*fy + T[10]*fz;

    rd_real_t t_rot_x = T[0]*tx + T[4]*ty + T[8]*tz;
    rd_real_t t_rot_y = T[1]*tx + T[5]*ty + T[9]*tz;
    rd_real_t t_rot_z = T[2]*tx + T[6]*ty + T[10]*tz;

    rd_real_t px = T[12], py = T[13], pz = T[14];
    f_acc[0] += f_rot_x; f_acc[1] += f_rot_y; f_acc[2] += f_rot_z;
    f_acc[3] += t_rot_x + (py*f_rot_z - pz*f_rot_y);
    f_acc[4] += t_rot_y + (pz*f_rot_x - px*f_rot_z);
    f_acc[5] += t_rot_z + (px*f_rot_y - py*f_rot_x);
}

static RD_INLINE void rd_spatial_transform_force(const rd_real_t* RD_RESTRICT T,
                                                 const rd_real_t* RD_RESTRICT f_in,
                                                 rd_real_t* RD_RESTRICT f_out) {
    rd_real_t fx = f_in[0], fy = f_in[1], fz = f_in[2];
    rd_real_t tx = f_in[3], ty = f_in[4], tz = f_in[5];

    /* f_out = R * f_in */
    rd_real_t f_rot_x = T[0]*fx + T[4]*fy + T[8]*fz;
    rd_real_t f_rot_y = T[1]*fx + T[5]*fy + T[9]*fz;
    rd_real_t f_rot_z = T[2]*fx + T[6]*fy + T[10]*fz;
    f_out[0] = f_rot_x; f_out[1] = f_rot_y; f_out[2] = f_rot_z;

    /* tau_out = R*tau_in + p x f_out */
    rd_real_t t_rot_x = T[0]*tx + T[4]*ty + T[8]*tz;
    rd_real_t t_rot_y = T[1]*tx + T[5]*ty + T[9]*tz;
    rd_real_t t_rot_z = T[2]*tx + T[6]*ty + T[10]*tz;

    rd_real_t px = T[12], py = T[13], pz = T[14];
    f_out[3] = t_rot_x + (py*f_rot_z - pz*f_rot_y);
    f_out[4] = t_rot_y + (pz*f_rot_x - px*f_rot_z);
    f_out[5] = t_rot_z + (px*f_rot_y - py*f_rot_x);
}

/*
 * out = v x (val * e_k), the motion cross product against a unit spatial axis.
 *
 * The only motion vector either RNEA or ABA ever crosses v with is S*qd, and
 * S is a unit spatial axis -- one component, plus or minus one. Building the
 * six-vector and running the general cross costs a six-float temporary and 24
 * multiplies to rediscover that a cross product with a scaled basis vector
 * touches four components. This costs four multiplies and no temporary.
 */
/* J is the axis, I and L the other two in cyclic order. Spelled out per axis
 * so the subscripts are compile-time constants: vldr and vstr take an
 * immediate offset and nothing else, and a run-time axis here means ten
 * computed addresses off two bases. */
#define RD_CROSS_ROT_(J, I, L)                                              \
    do {                                                                    \
        out[(J)]     = RD_REAL(0.0);                                        \
        out[(I)]     =  val * v[(L)];                                       \
        out[(L)]     = -val * v[(I)];                                       \
        out[3 + (J)] = RD_REAL(0.0);                                        \
        out[3 + (I)] =  val * v[3 + (L)];                                   \
        out[3 + (L)] = -val * v[3 + (I)];                                   \
    } while (0)

#define RD_CROSS_LIN_(J, I, L)                                              \
    do {                                                                    \
        out[(J)] = RD_REAL(0.0);                                            \
        out[(I)] =  val * v[3 + (L)];                                       \
        out[(L)] = -val * v[3 + (I)];                                       \
        out[3] = out[4] = out[5] = RD_REAL(0.0);                            \
    } while (0)

static RD_INLINE void rd_spatial_cross_axis(const rd_real_t* RD_RESTRICT v,
                                            rd_int_t s_axis, rd_real_t val,
                                            rd_real_t* RD_RESTRICT out) {
    switch (s_axis) {
        /* Revolute: a x e_j leaves component j empty and swaps the other two,
         * in both halves. */
        case 3:  RD_CROSS_ROT_(0, 1, 2); break;
        case 4:  RD_CROSS_ROT_(1, 2, 0); break;
        case 5:  RD_CROSS_ROT_(2, 0, 1); break;
        /* Prismatic: only v's angular half acts, and the result is linear. */
        case 0:  RD_CROSS_LIN_(0, 1, 2); break;
        case 1:  RD_CROSS_LIN_(1, 2, 0); break;
        default: RD_CROSS_LIN_(2, 0, 1); break;
    }
}

#undef RD_CROSS_ROT_
#undef RD_CROSS_LIN_

/**
 * @brief Sparse Motion Cross Product: out = v1 x v2
 */
static RD_INLINE void rd_spatial_cross_motion(const rd_real_t v1[6], 
                                              const rd_real_t v2[6], 
                                              rd_real_t out[6]) {
    /* Lin */
    out[0] = (v1[4]*v2[2] - v1[5]*v2[1]) + (v1[1]*v2[5] - v1[2]*v2[4]);
    out[1] = (v1[5]*v2[0] - v1[3]*v2[2]) + (v1[2]*v2[3] - v1[0]*v2[5]);
    out[2] = (v1[3]*v2[1] - v1[4]*v2[0]) + (v1[0]*v2[4] - v1[1]*v2[3]);
    /* Ang */
    out[3] = v1[4]*v2[5] - v1[5]*v2[4];
    out[4] = v1[5]*v2[3] - v1[3]*v2[5];
    out[5] = v1[3]*v2[4] - v1[4]*v2[3];
}

/**
 * @brief Sparse Force Cross Product: out = v1 x* f2
 */
static RD_INLINE void rd_spatial_cross_force(const rd_real_t v1[6], 
                                             const rd_real_t f2[6], 
                                             rd_real_t out[6]) {
    /* Force */
    out[0] = v1[4]*f2[2] - v1[5]*f2[1];
    out[1] = v1[5]*f2[0] - v1[3]*f2[2];
    out[2] = v1[3]*f2[1] - v1[4]*f2[0];
    /* Torque */
    out[3] = (v1[4]*f2[5] - v1[5]*f2[4]) + (v1[1]*f2[2] - v1[2]*f2[1]);
    out[4] = (v1[5]*f2[3] - v1[3]*f2[5]) + (v1[2]*f2[0] - v1[0]*f2[2]);
    out[5] = (v1[3]*f2[4] - v1[4]*f2[3]) + (v1[0]*f2[1] - v1[1]*f2[0]);
}

/* ============================================================================
 * 6. Rotation Generators
 * ============================================================================ */

static RD_INLINE void rd_rot_axis_angle(const rd_real_t axis[3], rd_real_t angle, rd_real_t R[9]) {
    rd_real_t nx = axis[0], ny = axis[1], nz = axis[2];
    rd_real_t n_sq = nx*nx + ny*ny + nz*nz;
    if (n_sq < RD_EPS) {
        memset(R, 0, 9*sizeof(rd_real_t));
        R[0]=R[4]=R[8]=1.0f; return;
    }
    rd_real_t inv_n = 1.0f / rd_sqrt(n_sq);
    nx *= inv_n; ny *= inv_n; nz *= inv_n;
    
    rd_real_t s, c;
    rd_sincos(angle, &s, &c);
    rd_real_t C = 1.0f - c;
    
    rd_real_t xC=nx*C, yC=ny*C, zC=nz*C;
    rd_real_t xxC=nx*xC, yyC=ny*yC, zzC=nz*zC;
    rd_real_t xyC=nx*yC, yzC=ny*zC, zxC=nz*xC;
    rd_real_t xs=nx*s, ys=ny*s, zs=nz*s;
    
    R[0]=xxC+c;   R[1]=xyC-zs;  R[2]=zxC+ys;
    R[3]=xyC+zs;  R[4]=yyC+c;   R[5]=yzC-xs;
    R[6]=zxC-ys;  R[7]=yzC+xs;  R[8]=zzC+c;
}

static RD_INLINE void rd_rot_to_mat4(const rd_real_t R[9], const rd_real_t t[3], rd_real_t T[16]) {
    T[0]=R[0];  T[1]=R[3];  T[2]=R[6];  T[3]=0.0f;
    T[4]=R[1];  T[5]=R[4];  T[6]=R[7];  T[7]=0.0f;
    T[8]=R[2];  T[9]=R[5];  T[10]=R[8]; T[11]=0.0f;
    T[12]=t[0]; T[13]=t[1]; T[14]=t[2]; T[15]=1.0f;
}

static RD_INLINE void rd_rot_rpy(rd_real_t roll, rd_real_t pitch, rd_real_t yaw, rd_real_t R[9]) {
    rd_real_t sr, cr, sp, cp, sy, cy;
    rd_sincos(roll, &sr, &cr);
    rd_sincos(pitch, &sp, &cp);
    rd_sincos(yaw, &sy, &cy);
    R[0] = cy*cp;  R[1] = cy*sp*sr - sy*cr;  R[2] = cy*sp*cr + sy*sr;
    R[3] = sy*cp;  R[4] = sy*sp*sr + cy*cr;  R[5] = sy*sp*cr - cy*sr;
    R[6] = -sp;    R[7] = cp*sr;             R[8] = cp*cr;
}

static RD_INLINE void rd_rot_quat(const rd_real_t q[4], rd_real_t R[9]) {
    rd_real_t qw=q[0], qx=q[1], qy=q[2], qz=q[3];
    rd_real_t n = rd_sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    
    if (n < RD_EPS) {
        memset(R, 0, 9*sizeof(rd_real_t));
        R[0]=R[4]=R[8]=1.0f; return;
    }
    
    rd_real_t s = 2.0f / (n * n);
    rd_real_t xx = qx*qx*s, yy = qy*qy*s, zz = qz*qz*s;
    rd_real_t xy = qx*qy*s, xz = qx*qz*s, yz = qy*qz*s;
    rd_real_t wx = qw*qx*s, wy = qw*qy*s, wz = qw*qz*s;
    
    R[0] = 1.0f - yy - zz;
    R[1] = xy - wz;
    R[2] = xz + wy;
    R[3] = xy + wz;
    R[4] = 1.0f - xx - zz;
    R[5] = yz - wx;
    R[6] = xz - wy;
    R[7] = yz + wx;
    R[8] = 1.0f - xx - yy;
}

#ifdef __cplusplus
}
#endif

#endif /* RD_MATH_H */