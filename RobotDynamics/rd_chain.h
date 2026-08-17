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

    /* Transforms (4x4, column-major) */
    rd_real_t* T_joint_offset;     /**< Joint offset transforms (n_nodes * 16) */
    rd_real_t* T_link_offset;      /**< Link offset transforms (n_nodes * 16) */

    /* Frame names */
    char** frame_names;            /**< Frame name for each node */

    /* Parent path indices */
    rd_idx_t* parent_path;         /**< Flattened paths (n_nodes * n_nodes) */
    rd_int_t* parent_path_len;     /**< Path length for each node */

    /* Spatial inertias (6x6, row-major) */
    rd_real_t* spatial_inertias;   /**< Size n_nodes * 36 */

    /* Same inertias in the ten-number packed form, for I*v products.
     * See rd_spatial_inertia_mul(). Size n_nodes * RD_INERTIA_COMPACT_LEN. */
    rd_real_t* inertia_compact;
    
} rd_chain_t;

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
