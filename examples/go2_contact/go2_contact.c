/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file go2_contact.c
 * @brief Contact dynamics for a quadruped, worked end to end on a Unitree Go2.
 *
 * A planted foot is not an external force you know -- it is a constraint whose
 * force you solve for. That is the difference between rd_rnea_ext() and
 * rd_constrained_dynamics(), and choosing wrongly is the usual way to get
 * plausible numbers that are wrong.
 *
 * The other thing this example is here to say: a constraint holds the *foot*,
 * not the robot. Plant all four feet on a robot with no joint torque and it
 * still falls -- the feet stay exactly where they are and the body comes down
 * between them, because nothing is holding the knees. Standing up is a torque
 * problem, and the contact only tells you what the ground gives back.
 *
 *   1. Four feet planted, limp: the body collapses and the ground barely pushes.
 *   2. Standing: pick the foot forces, ask inverse dynamics for the torques.
 *   3. Send those torques back through the forward solve; the body holds.
 *   4. A trot: the stance set changes every step and the chain does not.
 *   5. Normal force and the friction cone -- where this becomes contact dynamics.
 *   6. Closing the loop: the forces fed back through rd_rnea_ext() reproduce
 *      the torques we started from.
 *
 * Build (from this directory):  make && ./go2_contact
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "robot_dynamics.h"
#include "model_go2.h"

#define NV_MAX   24                       /* Go2 is 18: 6 base + 12 joints */
#define N_FEET    4
#define MU        0.6                      /* a rubber pad on concrete */

static rd_chain_t chain;
static rd_state_t state;
static rd_real_t  state_buf[RD_STATE_BUF_FLOATS(32)];

/* Work for the largest set we will ask for -- four point contacts. Size it for
 * the maximum once: a solver that reallocates when a foot lifts is a solver
 * that allocates inside the control loop. */
static rd_real_t work[4096];
static rd_real_t f_ext[6 * 40];

static const char* FOOT_NAME[N_FEET] = { "FL_foot", "FR_foot", "RL_foot", "RR_foot" };
static rd_idx_t    foot[N_FEET];
static rd_int_t    nv;

static void print_vec(const char* label, const rd_real_t* v, int n) {
    printf("  %-24s", label);
    for (int i = 0; i < n; ++i) printf(" %7.3f", (double)v[i]);
    printf("\n");
}

/*
 * rd_constrained_dynamics() solves an *equality* constrained system: it will
 * pull a foot down if that is what holding the constraint takes, and it will
 * ask for any tangential force it likes. The ground does neither. Those two
 * tests are what turn a constrained solve into contact, and the library leaves
 * them to the caller on purpose -- enforcing them is an iteration, not a
 * formula, and which iteration is a control decision.
 */
static int contact_is_physical(const rd_real_t f[3], const char* name) {
    const double fz = (double)f[2];
    const double ft = sqrt((double)f[0]*(double)f[0] + (double)f[1]*(double)f[1]);
    if (fz < 0.0) {
        printf("    %-8s normal %7.2f N   PULLING -- this foot should be released\n",
               name, fz);
        return 0;
    }
    if (ft > MU * fz) {
        printf("    %-8s normal %7.2f N  tangential %6.2f N   SLIPPING, needs mu >= %.2f\n",
               name, fz, ft, ft / fz);
        return 0;
    }
    printf("    %-8s normal %7.2f N  tangential %6.2f N   ok, mu used %.2f\n",
           name, fz, ft, fz > 1e-6 ? ft / fz : 0.0);
    return 1;
}

/* A world-frame force at a foot, written where rd_rnea_ext() reads it: per
 * node, in that link's own body frame, at the link origin -- which is exactly
 * the point the constraint acts at, so the two line up with no transfer. */
static void set_foot_force(rd_idx_t node, const rd_real_t f_world[3]) {
    rd_real_t T[16];
    rd_forward_kinematics(&chain, &state, node, T);
    rd_real_t* fe = &f_ext[node * 6];
    for (int r = 0; r < 3; ++r) {            /* R^T f, T column-major */
        fe[r] = T[r*4 + 0]*f_world[0] + T[r*4 + 1]*f_world[1] + T[r*4 + 2]*f_world[2];
    }
    fe[3] = fe[4] = fe[5] = 0;               /* a point force, no moment there */
}

int main(void) {
    if (rd_chain_build(go2_model_get(), &chain) != RD_OK) {
        printf("chain build failed\n"); return 1;
    }
    rd_state_init(&state, chain.n_nodes, state_buf, sizeof state_buf);
    nv = rd_chain_get_nv(&chain);

    for (int i = 0; i < N_FEET; ++i) {
        foot[i] = rd_chain_find_frame(&chain, FOOT_NAME[i]);
        if (foot[i] < 0) { printf("no frame %s\n", FOOT_NAME[i]); return 1; }
    }
    const rd_model_t* mdl = go2_model_get();
    double mass = 0;
    for (rd_uint_t i = 0; i < mdl->num_links; ++i) mass += (double)mdl->links[i].inertia.mass;
    const double weight = mass * 9.81;

    printf("Go2: %d links, %d joints, nv %d, %.2f kg (%.1f N)\n\n",
           (int)chain.n_nodes, (int)(nv - 6), (int)nv, mass, weight);

    /* A crouched stance with the base level. Real code reads these off the
     * encoders and the state estimator. */
    rd_real_t q_base[7] = { 0, 0, 0.32f, 1, 0, 0, 0 };
    rd_real_t q[12], qd[NV_MAX];
    for (int leg = 0; leg < 4; ++leg) {
        q[leg*3 + 0] = 0.0f;  q[leg*3 + 1] = 0.8f;  q[leg*3 + 2] = -1.5f;
    }
    memset(qd, 0, sizeof qd);
    rd_update_kinematics(&chain, &state, q_base, q, qd);

    rd_constraint_t stance[N_FEET];
    for (int i = 0; i < N_FEET; ++i) {
        stance[i].frame_a = foot[i];
        stance[i].frame_b = RD_ANCHOR_WORLD;   /* NOT RD_FRAME_WORLD; see the header */
        stance[i].type    = RD_CONSTRAINT_POINT;
    }
    /* POINT and not FULL. A foot is a point contact, and it is also the only
     * choice that stays solvable: four welds would be 24 constraints on an
     * 18-DOF robot and J M^-1 J^T would be singular. */

    rd_int_t need = rd_constrained_dynamics_work(&chain, stance, N_FEET);
    printf("workspace for four contacts: %d floats, %d bytes\n\n",
           (int)need, (int)(need * (int)sizeof(rd_real_t)));
    if (need > (rd_int_t)(sizeof work / sizeof work[0])) return 1;

    /* ------------------------------------------------------------------ */
    printf("1. Four feet planted, zero joint torque\n");
    /* ------------------------------------------------------------------ */
    rd_real_t tau_zero[NV_MAX] = {0};
    rd_real_t qdd[NV_MAX], lambda[3 * N_FEET];
    if (rd_constrained_dynamics(&chain, &state, tau_zero, NULL, stance, N_FEET,
                                work, qdd, lambda) != RD_OK) return 1;
    print_vec("base accel lin,ang", qdd, 6);
    rd_real_t got = 0;
    for (int i = 0; i < N_FEET; ++i) got += lambda[3*i + 2];
    printf("  ground supplies %.1f N of the %.1f N weight, and the base drops at\n"
           "  %.2f m/s^2. The feet have not moved -- the knees folded. A constraint\n"
           "  holds the foot; holding the robot up is the torque's job.\n\n",
           (double)got, weight, -(double)qdd[2]);

    /* ------------------------------------------------------------------ */
    printf("2. Standing: choose the foot forces, ask for the torques\n");
    /* ------------------------------------------------------------------ */
    /* Share the weight equally and ask inverse dynamics what torque holds the
     * robot still under those forces. This direction is the cheap one and it
     * is what a stance controller actually sends. */
    memset(f_ext, 0, sizeof f_ext);
    for (int i = 0; i < N_FEET; ++i) {
        rd_real_t up[3] = { 0, 0, (rd_real_t)(weight / N_FEET) };
        set_foot_force(foot[i], up);
    }
    rd_real_t tau_stand[NV_MAX];
    rd_rnea_ext(&chain, &state, NULL, NULL, f_ext, tau_stand);   /* qdd = 0 */

    /* The first six rows are the floating base, which has no actuator. What is
     * left there is the equilibrium this force split failed to reach: the
     * net wrench the ground would have to supply and does not. Driving it to
     * zero is force distribution -- a small QP in general, a control decision,
     * and deliberately not in this library. */
    print_vec("base residual lin,ang", tau_stand, 6);
    print_vec("hip/thigh/calf, FL", &tau_stand[6], 3);
    for (rd_int_t k = 0; k < 6; ++k) tau_stand[k] = 0;   /* no base actuator */
    printf("\n");

    /* ------------------------------------------------------------------ */
    printf("3. Those torques, back through the forward solve\n");
    /* ------------------------------------------------------------------ */
    if (rd_constrained_dynamics(&chain, &state, tau_stand, NULL, stance, N_FEET,
                                work, qdd, lambda) != RD_OK) return 1;
    print_vec("base accel lin,ang", qdd, 6);
    got = 0;
    for (int i = 0; i < N_FEET; ++i) got += lambda[3*i + 2];
    printf("  ground now supplies %.1f N of %.1f N, and the base holds to within\n"
           "  %.2f m/s^2. What is left is the base residual above, showing up as\n"
           "  the motion an equal split cannot prevent.\n\n",
           (double)got, weight, (double)fabs((double)qdd[2]));

    for (int i = 0; i < N_FEET; ++i) contact_is_physical(&lambda[3*i], FOOT_NAME[i]);
    printf("\n");

    /* ------------------------------------------------------------------ */
    printf("4. A trot: two feet down, and the chain never changes\n");
    /* ------------------------------------------------------------------ */
    /* Constraints are an argument, so a gait is an array that gets shorter and
     * longer. Nothing is rebuilt and nothing is allocated.
     *
     * The stance set and the torques have to move together, though. Sending
     * the four-foot torques with two feet down leaves half the weight
     * unsupported, which is a real failure mode and not a subtle one. */
    const int diag[2] = { 0, 3 };                 /* FL and RR in stance */
    rd_constraint_t pair[2] = { stance[diag[0]], stance[diag[1]] };
    rd_real_t qdd2[NV_MAX], lambda2[6];

    if (rd_constrained_dynamics(&chain, &state, tau_stand, NULL, pair, 2,
                                work, qdd2, lambda2) != RD_OK) return 1;
    printf("  four-foot torques, two feet down:  base z accel %6.2f m/s^2, "
           "ground %.1f N\n", (double)qdd2[2], (double)(lambda2[2] + lambda2[5]));

    /* Redistribute: the two feet still down take half the weight each. */
    memset(f_ext, 0, sizeof f_ext);
    for (int i = 0; i < 2; ++i) {
        rd_real_t up[3] = { 0, 0, (rd_real_t)(weight / 2.0) };
        set_foot_force(foot[diag[i]], up);
    }
    rd_real_t tau_trot[NV_MAX];
    rd_rnea_ext(&chain, &state, NULL, NULL, f_ext, tau_trot);
    for (rd_int_t k = 0; k < 6; ++k) tau_trot[k] = 0;

    if (rd_constrained_dynamics(&chain, &state, tau_trot, NULL, pair, 2,
                                work, qdd2, lambda2) != RD_OK) return 1;
    printf("  torques for this stance:           base z accel %6.2f m/s^2, "
           "ground %.1f N\n", (double)qdd2[2], (double)(lambda2[2] + lambda2[5]));
    for (int i = 0; i < 2; ++i) contact_is_physical(&lambda2[3*i], FOOT_NAME[diag[i]]);
    printf("\n");

    /* ------------------------------------------------------------------ */
    printf("5. Pushing sideways until the friction cone gives\n");
    /* ------------------------------------------------------------------ */
    /* Ask the four feet to shove the robot sideways as well as hold it up.
     * The normal force is fixed by the weight, so the tangential demand rises
     * with the acceleration asked for and the cone decides what is reachable.
     * This is the whole of "contact dynamics" as a control constraint. */
    for (double ax = 1.0; ax <= 8.0; ax += 2.0) {
        memset(f_ext, 0, sizeof f_ext);
        for (int i = 0; i < N_FEET; ++i) {
            rd_real_t push[3] = { (rd_real_t)(mass * ax / N_FEET), 0,
                                  (rd_real_t)(weight / N_FEET) };
            set_foot_force(foot[i], push);
        }
        rd_real_t tau_push[NV_MAX];
        rd_rnea_ext(&chain, &state, NULL, NULL, f_ext, tau_push);
        for (rd_int_t k = 0; k < 6; ++k) tau_push[k] = 0;
        if (rd_constrained_dynamics(&chain, &state, tau_push, NULL, stance,
                                    N_FEET, work, qdd, lambda) != RD_OK) return 1;
        const double fz = (double)lambda[2];
        const double ft = sqrt((double)lambda[0]*(double)lambda[0]
                             + (double)lambda[1]*(double)lambda[1]);
        printf("    ask %.0f m/s^2 sideways -> base x accel %5.2f, "
               "front-left needs mu %.2f  %s\n",
               ax, (double)qdd[0], fz > 1e-6 ? ft/fz : 99.0,
               (ft <= MU*fz) ? "" : "<-- slips");
    }
    printf("  Above about mu*g the feet cannot deliver it, whatever the torques\n"
           "  say. The library will still hand you the forces; deciding they are\n"
           "  unreachable is yours.\n\n");

    /* ------------------------------------------------------------------ */
    printf("6. Closing the loop\n");
    /* ------------------------------------------------------------------ */
    /* The forward solve said: with these torques and these feet planted, the
     * robot accelerates like qdd and the ground pushes with lambda. Inverse
     * dynamics has to agree. If a sign or a frame is wrong this is where it
     * shows, and it needs no reference implementation to say so. */
    /* Solve once more with the standing torques, since the sweep above has
     * been reusing qdd and lambda. */
    if (rd_constrained_dynamics(&chain, &state, tau_stand, NULL, stance, N_FEET,
                                work, qdd, lambda) != RD_OK) return 1;
    memset(f_ext, 0, sizeof f_ext);
    for (int i = 0; i < N_FEET; ++i) set_foot_force(foot[i], &lambda[3*i]);
    rd_real_t tau_check[NV_MAX];
    rd_rnea_ext(&chain, &state, qdd, NULL, f_ext, tau_check);

    rd_real_t worst = 0;
    for (rd_int_t k = 0; k < nv; ++k) {
        rd_real_t e = (rd_real_t)fabs((double)(tau_check[k] - tau_stand[k]));
        if (e > worst) worst = e;
    }
    printf("  max |rnea_ext(qdd, lambda) - tau| = %.3e   %s\n",
           (double)worst, worst < RD_REAL(1e-2) ? "consistent" : "SOMETHING IS WRONG");

    rd_chain_free(&chain);
    return worst < RD_REAL(1e-2) ? 0 : 1;
}
