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
    static RD_INLINE rd_real_t rd_sqrt(rd_real_t x) { float r; arm_sqrt_f32(x, &r); return r; }
#elif RD_REAL_IS_FLOAT
    static RD_INLINE rd_real_t rd_sin(rd_real_t x) { return sinf(x); }
    static RD_INLINE rd_real_t rd_cos(rd_real_t x) { return cosf(x); }
    static RD_INLINE rd_real_t rd_sqrt(rd_real_t x) { return sqrtf(x); }
#else
    static RD_INLINE rd_real_t rd_sin(rd_real_t x) { return sin(x); }
    static RD_INLINE rd_real_t rd_cos(rd_real_t x) { return cos(x); }
    static RD_INLINE rd_real_t rd_sqrt(rd_real_t x) { return sqrt(x); }
#endif

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

/* One angle, both functions. Cortex-M4F cycles for a (sin, cos) pair, and
 * worst-case absolute error against double-precision libm over [-pi, pi]:
 *
 *     sinf + cosf     273.6 cyc   5.9e-08    default
 *     RD_FAST_TRIG     57.3 cyc   6.6e-08
 *
 * Two alternatives are worse and should not be reached for: newlib's sincosf
 * is 313.4 cyc because it is `bl sinf; bl cosf` plus stack shuffling rather
 * than a fused routine, and CMSIS-DSP's arm_sin/cos_f32 is 105.5 cyc at
 * 1.9e-05, interpolating linearly between 512 table entries and giving up
 * accuracy float32 can resolve.
 *
 * RD_FAST_TRIG is off by default: a build option that changes results, however
 * slightly, should be opted into deliberately. */
#if RD_FAST_TRIG && RD_REAL_IS_FLOAT
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

static RD_INLINE void rd_mat4_identity(rd_real_t T[16]) {
    memset(T, 0, 16 * sizeof(rd_real_t));
    T[0] = T[5] = T[10] = T[15] = RD_REAL(1.0);
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
 * Rotation about a coordinate axis, written straight into a 4x4.
 *
 * rd_axis_t admits only +-X, +-Y and +-Z, so the general axis-angle path --
 * which normalises the axis with a sqrt and a divide and then evaluates
 * Rodrigues' formula -- is rederiving something already fixed by the type.
 * Falls through to the general form if the axis is not a unit coordinate
 * direction, so a hand-written model cannot be silently mis-rotated.
 */
static RD_INLINE int rd_mat4_axis_rotation(const rd_real_t axis[3], rd_real_t q,
                                           rd_real_t T[16]) {
    rd_real_t s, c;
    int which = -1, sign = 0;

    for (int i = 0; i < 3; ++i) {
        if (axis[i] == RD_REAL(1.0) || axis[i] == RD_REAL(-1.0)) {
            if (which >= 0) return 0;                  /* more than one axis set */
            which = i;
            sign = (axis[i] > RD_REAL(0.0)) ? 1 : -1;
        } else if (axis[i] != RD_REAL(0.0)) {
            return 0;                                  /* not axis-aligned */
        }
    }
    if (which < 0) return 0;

    rd_sincos(q, &s, &c);
    if (sign < 0) s = -s;

    rd_mat4_identity(T);
    switch (which) {
        case 0: T[5] = c;  T[6] = s;   T[9] = -s;  T[10] = c; break;  /* Rx */
        case 1: T[0] = c;  T[2] = -s;  T[8] = s;   T[10] = c; break;  /* Ry */
        default: T[0] = c; T[1] = s;   T[4] = -s;  T[5]  = c; break;  /* Rz */
    }
    return 1;
}

/* Ti = inv(T) for SE3 */
static RD_INLINE void rd_mat4_inv(const rd_real_t T[16], rd_real_t Ti[16]) {
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
static RD_INLINE void rd_spatial_inertia_mul(const rd_real_t* RD_RESTRICT ic,
                                             const rd_real_t* RD_RESTRICT v,
                                             rd_real_t* RD_RESTRICT out) {
    const rd_real_t m  = ic[0];
    const rd_real_t cx = ic[1], cy = ic[2], cz = ic[3];
    const rd_real_t vx = v[0], vy = v[1], vz = v[2];
    const rd_real_t wx = v[3], wy = v[4], wz = v[5];

    /* linear = m * (v - c x w) */
    out[0] = m * (vx - (cy*wz - cz*wy));
    out[1] = m * (vy - (cz*wx - cx*wz));
    out[2] = m * (vz - (cx*wy - cy*wx));

    /* angular = m * (c x v) + J w */
    const rd_real_t Jxx = ic[4], Jyy = ic[5], Jzz = ic[6];
    const rd_real_t Jxy = ic[7], Jxz = ic[8], Jyz = ic[9];
    out[3] = m * (cy*vz - cz*vy) + Jxx*wx + Jxy*wy + Jxz*wz;
    out[4] = m * (cz*vx - cx*vz) + Jxy*wx + Jyy*wy + Jyz*wz;
    out[5] = m * (cx*vy - cy*vx) + Jxz*wx + Jyz*wy + Jzz*wz;
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

/* ============================================================================
 * 5. Sparse Spatial Algebra (High-Performance RNEA/Jacobian)
 * ============================================================================ */

/**
 * @brief Sparse Motion Transform: v_A = Ad(T_AB) * v_B
 */
static RD_INLINE void rd_spatial_transform_motion(const rd_real_t T[16], 
                                                  const rd_real_t v_in[6], 
                                                  rd_real_t v_out[6]) {
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

/**
 * @brief Sparse Force Transform: f_A = Ad(T_BA)^T * f_B
 */
static RD_INLINE void rd_spatial_transform_force(const rd_real_t T[16], 
                                                 const rd_real_t f_in[6], 
                                                 rd_real_t f_out[6]) {
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