/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file rd_chain.h
 * @brief Kinematic Chain Data Structure
 * 
 * Pre-computed data structure for efficient kinematics and dynamics
 * computations. Built from rd_model_t.
 */

#ifndef RD_CHAIN_H
#define RD_CHAIN_H

#include "rd_config.h"
#include "rd_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Chain Data Structure
 * ============================================================================ */

/** Per-node control block for the dynamics traversals. See rd_chain_t::dyn. */
typedef struct {
    rd_idx_t  node;     /**< Node index */
    rd_idx_t  parent;   /**< parent_list[node] */
    rd_idx_t  danc;     /**< Nearest moving ancestor, or -1 for the root */
    rd_idx_t  jidx;     /**< Actuated joint index, or -1 */
    rd_idx_t  vidx;     /**< Index into qd/qdd/tau, or -1 for a node with no DOF */
    rd_idx_t  s_axis;   /**< Motion subspace component, 0..5 */
    rd_real_t s_sign;   /**< Its sign; zero for a node with no DOF */
} rd_dyn_node_t;

typedef struct {
    rd_int_t n_nodes;              /**< Number of links */
    rd_int_t n_joints;             /**< Number of actuated joints */
    rd_int_t has_floating_base;    /**< 1 if floating base */

    /* Topology */
    rd_idx_t* parent_list;         /**< Parent index for each node (size n_nodes) */
    rd_idx_t* topo_order;          /**< Topological order (size n_nodes) */

    /* Children structure */
    rd_int_t* children_count;      /**< Number of children per node */
    rd_idx_t* children_list;       /**< Flattened children list (n_nodes * n_nodes) */
    rd_int_t max_children;         /**< Maximum children any node has */

    /* Joint mapping */
    rd_idx_t* joint_idx;           /**< Actuated joint index per node (-1 if fixed) */
    rd_int_t* joint_type;          /**< Joint type per node */

    /* Joint axes (n_joints x 3) */
    rd_real_t* axes;               /**< Row-major, size n_joints*3 */

    /* Transforms (4x4, column-major). There is deliberately no separate link
     * offset: the link frame is the joint's child frame, so one would always be
     * the identity, and composing against it cost a 4x4 multiply per node per
     * tick. Reintroducing the distinction means changing this struct, not
     * multiplying by ones. */
    rd_real_t* T_joint_offset;     /**< Joint offset transforms (n_nodes * 16) */

    /* Frame names */
    char** frame_names;            /**< Frame name for each node */

    /* Parent path indices */
    rd_idx_t* parent_path;         /**< Flattened paths (n_nodes * n_nodes) */
    rd_int_t* parent_path_len;     /**< Path length for each node */

    /* Same inertias in the ten-number packed form, for I*v products.
     * See rd_spatial_inertia_mul(). Size n_nodes * RD_INERTIA_COMPACT_LEN. */
    rd_real_t* inertia_compact;

    /* ---- Dynamics tree ---------------------------------------------------
     * A fixed joint adds a frame but no degree of freedom, and on a real robot
     * most nodes are fixed: feet, sensor mounts and inertial frames. Walking
     * them in RNEA, CRBA and ABA costs 63-84% of what a real joint costs and
     * buys nothing, so those three traverse this reduced tree instead: only
     * nodes whose joint can move, plus the root.
     *
     * Each fixed link's spatial inertia is folded into its nearest moving
     * ancestor at build time, so `inertia_compact` holds *composite* bodies --
     * a moving node carries the mass of the fixed links hanging off it, and a
     * fixed node's own entry is zero. Total mass and the dynamics are
     * unchanged; the per-link split is not recoverable from here afterwards. Kinematics still walks every node, so every frame keeps its
     * own pose, velocity and Jacobian. */
    /* The motion subspace, which is always a unit spatial axis: rd_axis_t can
     * only hold +/-X, +/-Y or +/-Z and the link frame is the joint's child
     * frame. Two numbers instead of a six-vector, and every I*S product in the
     * dynamics becomes a column read. */
    rd_idx_t*  s_axis;             /**< Which of the six components, 0..5 */
    rd_real_t* s_sign;             /**< Its sign; zero for a node with no DOF */

    rd_int_t   n_dyn;              /**< Number of dynamics nodes */
    rd_idx_t*  dyn_order;          /**< n_dyn, topological */
    rd_idx_t*  dyn_parent;         /**< n_nodes, nearest moving ancestor or -1 */
    rd_idx_t*  dyn_child;          /**< n_nodes, CSR values */
    rd_int_t*  dyn_child_start;    /**< n_nodes+1, CSR offsets indexed by node */

    /* Everything the dynamics traversals need to know about a node, gathered
     * into one record. rd_update_kinematics, RNEA, CRBA and ABA all walk
     * dyn_order and then look the same handful of facts up in six separate
     * arrays: a base pointer out of this struct and an indexed load for each,
     * six pointers competing for the same registers as the transforms. One
     * sequential sixteen-byte record leaves a single pointer live and reads
     * in one burst. */
    rd_dyn_node_t* dyn;            /**< n_dyn, in dyn_order */
    rd_idx_t*      dyn_slot;       /**< n_nodes, node -> index into dyn, or -1 */

} rd_chain_t;

/** Does this node carry a degree of freedom (or is it the root)? */
static RD_INLINE int rd_chain_node_is_dynamic(const rd_chain_t* chain,
                                              rd_idx_t node) {
    if (chain->parent_list[node] == -1) return 1;
    if (chain->joint_idx[node] < 0) return 0;
    return chain->joint_type[node] == RD_JOINT_REVOLUTE ||
           chain->joint_type[node] == RD_JOINT_PRISMATIC;
}

/* ============================================================================
 * Chain API Functions
 * ============================================================================ */

/**
 * @brief Build chain from robot model
 * 
 * @param model Source robot model
 * @param chain Output chain (must be allocated)
 * @return RD_OK on success, error code otherwise
 */
rd_status_t rd_chain_build(const rd_model_t* model, rd_chain_t* chain);

/**
 * @brief Free chain resources
 * 
 * @param chain Chain to free
 */
void rd_chain_free(rd_chain_t* chain);

/**
 * @brief Find frame index by name
 * 
 * @param chain Chain to search
 * @param name Frame name
 * @return Frame index, or -1 if not found
 */
rd_idx_t rd_chain_find_frame(const rd_chain_t* chain, const char* name);

/**
 * @brief Get velocity DOF
 */
RD_INLINE rd_int_t rd_chain_get_nv(const rd_chain_t* chain) {
    if (!chain) return 0;
    return chain->has_floating_base ? (6 + chain->n_joints) : chain->n_joints;
}

#ifdef __cplusplus
}
#endif

#endif /* RD_CHAIN_H */
