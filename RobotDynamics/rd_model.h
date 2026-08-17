/**
 * @file rd_model.h
 * @brief Robot Model Definition
 * 
 * Defines the kinematic and dynamic parameters of articulated robots.
 * Supports floating base and open kinematic chains.
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

typedef enum {
    RD_JOINT_FIXED = 0,      /**< Fixed (no motion) */
    RD_JOINT_REVOLUTE,       /**< Rotational joint */
    RD_JOINT_PRISMATIC,      /**< Linear joint */
    RD_JOINT_FLOATING        /**< 6-DOF floating (for base) */
} rd_joint_type_t;

/* ============================================================================
 * Axis Direction
 * ============================================================================ */

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

typedef enum {
    RD_FRAME_WORLD = 0,   /**< World/spatial frame */
    RD_FRAME_LOCAL = 1    /**< Body-fixed/local frame */
} rd_frame_t;

/* ============================================================================
 * 3D Vector
 * ============================================================================ */

typedef struct {
    rd_real_t x;
    rd_real_t y;
    rd_real_t z;
} rd_vec3_t;

/* ============================================================================
 * 3x3 Inertia Tensor (symmetric)
 * ============================================================================ */

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

typedef struct {
    rd_real_t mass;        /**< Mass (kg) */
    rd_vec3_t com;         /**< Center of mass in link frame (m) */
    rd_inertia3_t I_com;   /**< Inertia at COM (kg·m²) */
} rd_spatial_inertia_t;

/* ============================================================================
 * Joint Definition
 * ============================================================================ */

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
} rd_joint_t;

/* ============================================================================
 * Link Definition
 * ============================================================================ */

typedef struct {
    char name[16];                 /**< Link name */
    
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

typedef struct {
    char name[32];                 /**< Robot name */
    
    rd_uint_t num_links;           /**< Total number of links (including base) */
    rd_uint_t num_joints;          /**< Number of actuated joints */
    rd_uint_t use_floating_base;   /**< 1: floating base, 0: fixed */
    rd_uint_t total_dof;           /**< Total DOF = 6 (if floating) + num_joints */
    
    /* Link definitions (index 0 = base link) */
    rd_link_t links[RD_MAX_LINKS];
    
    /* Gravity vector in world frame */
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
