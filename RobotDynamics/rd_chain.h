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
    rd_idx_t  vidx;     /**< Index into qd/qdd/tau, or -1 for a node with no DOF.
                         *   The index into q_joints is this minus the base's
                         *   six, which is why jidx is not stored: the record is
                         *   kept to sixteen bytes so that walking it is a shift
                         *   rather than a multiply. */
    rd_idx_t  s_axis;   /**< Motion subspace component, 0..5 */
    rd_idx_t  axis_rot; /**< Coordinate axis 0..2 when T_dyn's rotation is that
                         *   axis rotation and nothing else -- true whenever a
                         *   revolute joint's origin carries no rotation and its
                         *   parent is itself a dynamics node, which is the
                         *   ordinary case in a URDF. -1 when it is general. */
    rd_real_t s_sign;   /**< Its sign; zero for a node with no DOF */
} rd_dyn_node_t;

/** A model prepared for the algorithms to walk.
 *
 * rd_chain_build() turns an rd_model_t into one of these once at startup, and
 * it is the only allocation the library performs. Everything a per-tick
 * traversal would otherwise recompute is settled here: the topological order,
 * the parent paths, the spatial inertias shifted to their link origins, the
 * reduced tree that skips fixed links, and which joints are axis-aligned enough
 * for the fast kernels.
 *
 * Read-only during a control tick, and shareable between ticks and between
 * rd_state_t buffers. rd_chain_set_armature() is the one field meant to be
 * written after the build. Release it with rd_chain_free().
 *
 * The fields are documented for people reading the implementation. Nothing in
 * the public API requires touching them. */
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
    rd_int_t   n_axis_rot;        /**< How many of them have axis_rot >= 0.
                                   *   Zero lets rd_crba drop the axis-aligned
                                   *   kernel entirely, which is not merely dead
                                   *   code to branch over: keeping both kernels
                                   *   in one loop costs the general one
                                   *   registers. */
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


    /* Last on purpose. Everything above it is read by the hot traversals, and
     * moving their offsets was worth 4% of rd_crba on a part whose data cache
     * is 128 bytes -- measured, twice. A field nobody reads per node belongs
     * where it cannot disturb the ones that are. */
    rd_real_t* armature;           /**< nv, reflected rotor inertia per velocity
                                    *   index. Zero for a floating base's six.
                                    *   Writable: see rd_chain_set_armature. */
    rd_int_t   has_armature;       /**< Whether any of it is non-zero. Walking
                                    *   the mass matrix diagonal to add it costs
                                    *   3-5% of rd_crba, measured, so a model
                                    *   without rotor inertia skips it. */
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
 * @brief Prepare a model for the algorithms. Call once, at startup.
 *
 * Orders the tree, walks out the parent paths, shifts each link's inertia to
 * its own origin, folds the fixed links into their nearest moving ancestor,
 * and works out which joints qualify for the axis-aligned kernels. All of it
 * per model, none of it per tick.
 *
 * This is the only function in the library that allocates. It takes several
 * blocks from the heap and rd_chain_free() is what returns them, so a chain
 * that outlives the program is fine and a chain built per tick is not.
 *
 * @param chain Somewhere to write the result. Its own storage is the caller's;
 *              what is inside it belongs to this function.
 * @return RD_ERR_ALLOC_FAILED if the heap could not supply it,
 *         RD_ERR_INVALID_SIZE if the model exceeds RD_MAX_LINKS or
 *         RD_MAX_JOINTS.
 */
rd_status_t rd_chain_build(const rd_model_t* model, rd_chain_t* chain);

/**
 * @brief Set one joint's reflected rotor inertia after the chain is built.
 * @param vidx Velocity index, i.e. the joint's position in qd/qdd/tau.
 * @param value n^2 * I_rotor, in the units of a diagonal entry of M.
 *
 * URDF carries no armature, so a model generated from one starts at zero and
 * this is how it gets filled in. It is added to M's diagonal by rd_crba, to
 * tau by rd_rnea, and to the articulated inertia by rd_aba, which keeps the
 * three consistent with each other.
 */
rd_status_t rd_chain_set_armature(rd_chain_t* chain, rd_int_t vidx,
                                  rd_real_t value);

/**
 * @brief Release what rd_chain_build() allocated.
 *
 * Leaves the rd_chain_t itself alone, so a static one may be rebuilt. Any
 * rd_state_t used with this chain is stale afterwards and has to go back
 * through rd_update_kinematics() before it is read again.
 */
void rd_chain_free(rd_chain_t* chain);

/**
 * @brief Look a frame up by the name its link carries in the model.
 *
 * A linear scan, so resolve the frames a control loop cares about at startup
 * and keep the indices. Every frame in the model is here, fixed links
 * included: a foot is usually a fixed link, and asking for it by name is how
 * you get an index the Jacobian and the constraints will accept.
 *
 * @return The frame index, or -1 if no link carries that name.
 */
rd_idx_t rd_chain_find_frame(const rd_chain_t* chain, const char* name);

/**
 * @brief nv, the length of qd, qdd and tau, and the width of J and M.
 *
 * 6 + n_joints for a floating base, n_joints for a fixed one. nq differs from
 * it, because the base pose needs a quaternion: that is why q_base and
 * q_joints are separate arguments everywhere.
 */
RD_INLINE rd_int_t rd_chain_get_nv(const rd_chain_t* chain) {
    if (!chain) return 0;
    return chain->has_floating_base ? (6 + chain->n_joints) : chain->n_joints;
}

#ifdef __cplusplus
}
#endif

#endif /* RD_CHAIN_H */
