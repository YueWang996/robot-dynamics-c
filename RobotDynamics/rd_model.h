/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file rd_model.h
 * @brief What a robot is, as data.
 *
 * A tree of links, each with an inertia and a joint to its parent, plus an
 * optional floating base. No pointers and no allocation: an rd_model_t is a
 * plain struct that tools/urdf2c.py emits as a const initialiser, so it lives
 * in flash and costs no RAM.
 *
 * Closed chains are described the way Pinocchio describes them -- the tree
 * here, and the loop as a constraint passed to the algorithms. See
 * rd_constraint_t.
 */

#ifndef RD_MODEL_H
#define RD_MODEL_H

#include "rd_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Joint Types
 * ============================================================================ */

/** What a link's joint to its parent can do.
 *
 * A fixed joint still describes a frame, and the library keeps it: feet, sensor
 * mounts and inertial frames are all fixed links, and rd_chain_build() folds
 * them into their nearest moving ancestor so the dynamics never walks them.
 * Go2 has 18 of them out of 31 links. */
typedef enum {
    RD_JOINT_FIXED = 0,      /**< Fixed (no motion) */
    RD_JOINT_REVOLUTE,       /**< Rotational joint */
    RD_JOINT_PRISMATIC,      /**< Linear joint. Implemented, and not yet checked
                              *   against Pinocchio: the reference robots are
                              *   revolute throughout. */
    RD_JOINT_FLOATING        /**< The 6-DOF base pose. Belongs to link 0 and
                              *   nowhere else; rd_model_t::use_floating_base is
                              *   what actually switches it on. */
} rd_joint_type_t;

/* ============================================================================
 * Axis Direction
 * ============================================================================ */

/** Which body axis a joint turns about or slides along.
 *
 * Axis-aligned only. A URDF axis of <1 0 0> is RD_AXIS_X and <0 0 -1> is
 * RD_AXIS_NEG_Z; anything at a slant has to be absorbed into the joint's rpy
 * offset, and tools/urdf2c.py refuses the model rather than rounding it. The
 * restriction is what lets rd_update_kinematics() write one sine and one cosine
 * into a transform instead of composing a general rotation, and it is worth a
 * large fraction of the library's speed. */
typedef enum {
    RD_AXIS_X = 0,
    RD_AXIS_Y,
    RD_AXIS_Z,
    RD_AXIS_NEG_X,
    RD_AXIS_NEG_Y,
    RD_AXIS_NEG_Z
} rd_axis_t;

/* ============================================================================
 * Reference Frame
 * ============================================================================ */

/** Which frame a Jacobian, velocity or acceleration is expressed in.
 *
 * RD_FRAME_WORLD is Pinocchio's ReferenceFrame.WORLD: the spatial vector taken
 * at the world origin, with the axes of the world. It is not
 * LOCAL_WORLD_ALIGNED. To get the world-aligned quantity at a point p, take the
 * WORLD one and subtract skew(p) times its angular half from the linear half.
 *
 * Passing one of these where a constraint wants a frame index is a trap worth
 * knowing about: see RD_ANCHOR_WORLD. */
typedef enum {
    RD_FRAME_WORLD = 0,   /**< At the world origin, in world axes. */
    RD_FRAME_LOCAL = 1    /**< In the link's own body frame. */
} rd_frame_t;

/* ============================================================================
 * 3D Vector
 * ============================================================================ */

/** A point or a direction in metres. */
typedef struct {
    rd_real_t x;
    rd_real_t y;
    rd_real_t z;
} rd_vec3_t;

/* ============================================================================
 * 3x3 Inertia Tensor (symmetric)
 * ============================================================================ */

/** A symmetric 3x3 rotational inertia, six numbers, in kg m^2. */
typedef struct {
    rd_real_t Ixx;
    rd_real_t Iyy;
    rd_real_t Izz;
    rd_real_t Ixy;
    rd_real_t Ixz;
    rd_real_t Iyz;
} rd_inertia3_t;

/* ============================================================================
 * Spatial Inertia (compact storage)
 * ============================================================================ */

/** A link's mass distribution, the way a URDF states it.
 *
 * Mass, centre of mass, and the inertia taken at that centre. rd_chain_build()
 * turns this into the ten-number spatial inertia the algorithms use, shifted to
 * the link's own origin, so nothing downstream repeats the parallel-axis
 * arithmetic per tick. */
typedef struct {
    rd_real_t mass;        /**< Mass (kg) */
    rd_vec3_t com;         /**< Center of mass in link frame (m) */
    rd_inertia3_t I_com;   /**< Inertia at COM (kg·m²) */
} rd_spatial_inertia_t;

/* ============================================================================
 * Joint Definition
 * ============================================================================ */

/** The joint between a link and its parent, plus what drives it.
 *
 * Limits, damping and friction are parsed and stored and no algorithm reads
 * them yet. armature is read, by CRBA and by both forward-dynamics methods. */
typedef struct {
    rd_joint_type_t type;  /**< Joint type */
    rd_axis_t axis;        /**< Rotation/translation axis */
    
    /* Joint limits */
    rd_real_t q_min;       /**< Lower limit (rad or m) */
    rd_real_t q_max;       /**< Upper limit */
    rd_real_t dq_max;      /**< Max velocity */
    rd_real_t tau_max;     /**< Max torque/force */
    
    /* Joint dynamics */
    rd_real_t damping;     /**< Viscous damping */
    rd_real_t friction;    /**< Coulomb friction */

    /**
     * Reflected rotor inertia, in the same units as a diagonal entry of M.
     * A geared actuator's rotor contributes n^2 * I_rotor at the joint, which
     * on a hobby servo or any high-ratio drive is routinely larger than the
     * link it turns. Zero leaves the model rigid-body only.
     *
     * URDF has no field for it, so tools/urdf2c.py emits zero and it is set
     * afterwards -- rd_chain_set_armature(), or the chain's array directly.
     */
    rd_real_t armature;
} rd_joint_t;

/* ============================================================================
 * Link Definition
 * ============================================================================ */

/** One link: where it sits, what it weighs, and how it attaches.
 *
 * Parents come before children in rd_model_t::links, which is what lets
 * rd_chain_build() order the tree in one pass. tools/urdf2c.py guarantees it. */
typedef struct {
    char name[16];                 /**< Link name, NUL-terminated, 15 characters
                                    *   of it. rd_chain_find_frame() matches on
                                    *   this. */
    
    /* Transform from parent joint to this link's joint */
    rd_vec3_t pos_parent;          /**< Position offset from parent (m) */
    rd_vec3_t rpy_parent;          /**< Orientation offset (roll, pitch, yaw) (rad) */
    
    /* Inertial parameters */
    rd_spatial_inertia_t inertia;
    
    /* Joint connecting to child */
    rd_joint_t joint;
    
    /* Tree structure */
    rd_idx_t parent_idx;           /**< Parent link index (-1 for base) */
} rd_link_t;

/* ============================================================================
 * Robot Model
 * ============================================================================ */

/** A whole robot, as a plain struct with no pointers in it.
 *
 * Generate one with tools/urdf2c.py and it comes out as a const initialiser
 * that lives in flash. It is a static description and is never written during
 * a control tick: rd_chain_build() reads it once into an rd_chain_t, and the
 * algorithms read that. */
typedef struct {
    char name[32];                 /**< Robot name */
    
    rd_uint_t num_links;           /**< Total number of links (including base) */
    rd_uint_t num_joints;          /**< Number of actuated joints */
    rd_uint_t use_floating_base;   /**< 1: floating base, 0: fixed */
    rd_uint_t total_dof;           /**< Total DOF = 6 (if floating) + num_joints */
    
    /* Link definitions (index 0 = base link) */
    rd_link_t links[RD_MAX_LINKS];
    
    /** Gravity in world axes, conventionally {0, 0, -9.81}.
     *
     * rd_chain_build() does not carry this across, and an algorithm handed a
     * NULL gravity uses {0, 0, -RD_GRAVITY} rather than reading it back. Pass
     * it explicitly if the model states something else -- rd_vec3_t is three
     * contiguous rd_real_t, so &model.gravity.x is the argument. */
    rd_vec3_t gravity;
    
} rd_model_t;

/* ============================================================================
 * Model API Functions
 * ============================================================================ */

/**
 * @brief Get total DOF of the model
 * @param model Robot model
 * @return Total DOF (6 + num_joints for floating base)
 */
RD_INLINE rd_uint_t rd_model_get_dof(const rd_model_t* model) {
    if (!model) return 0;
    return model->use_floating_base ? (6 + model->num_joints) : model->num_joints;
}

/**
 * @brief Get velocity DOF (nv)
 */
RD_INLINE rd_uint_t rd_model_get_nv(const rd_model_t* model) {
    return rd_model_get_dof(model);
}

/**
 * @brief Get configuration DOF (nq)
 * Note: For floating base, nq = 7 (pos + quat) + num_joints
 */
RD_INLINE rd_uint_t rd_model_get_nq(const rd_model_t* model) {
    if (!model) return 0;
    return model->use_floating_base ? (7 + model->num_joints) : model->num_joints;
}

#ifdef __cplusplus
}
#endif

#endif /* RD_MODEL_H */
