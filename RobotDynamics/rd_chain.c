/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file rd_chain.c
 * @brief Kinematic Chain Implementation
 */

#include "rd_chain.h"
#include "rd_math.h"
#include <string.h>

/* Convert axis enum to vector */
static void axis_to_vec(rd_axis_t a, rd_real_t v[3]) {
    v[0] = v[1] = v[2] = RD_REAL(0.0);
    switch (a) {
        case RD_AXIS_X:     v[0] =  RD_REAL(1.0); break;
        case RD_AXIS_Y:     v[1] =  RD_REAL(1.0); break;
        case RD_AXIS_Z:     v[2] =  RD_REAL(1.0); break;
        case RD_AXIS_NEG_X: v[0] = -RD_REAL(1.0); break;
        case RD_AXIS_NEG_Y: v[1] = -RD_REAL(1.0); break;
        case RD_AXIS_NEG_Z: v[2] = -RD_REAL(1.0); break;
        default: break;
    }
}

rd_status_t rd_chain_build(const rd_model_t* model, rd_chain_t* chain) {
    if (!model || !chain) return RD_ERR_NULL_PTR;
    
    rd_int_t n = (rd_int_t)model->num_links;
    rd_int_t nj = (rd_int_t)model->num_joints;
    
    chain->n_nodes = n;
    chain->n_joints = nj;
    chain->has_floating_base = (rd_int_t)model->use_floating_base;

    /* Allocate arrays */
    chain->parent_list = (rd_idx_t*)RD_MALLOC(sizeof(rd_idx_t) * n);
    chain->topo_order = (rd_idx_t*)RD_MALLOC(sizeof(rd_idx_t) * n);
    chain->children_count = (rd_int_t*)RD_CALLOC(n, sizeof(rd_int_t));
    chain->children_list = (rd_idx_t*)RD_MALLOC(sizeof(rd_idx_t) * n * n);
    chain->joint_idx = (rd_idx_t*)RD_MALLOC(sizeof(rd_idx_t) * n);
    chain->joint_type = (rd_int_t*)RD_MALLOC(sizeof(rd_int_t) * n);
    chain->axes = (rd_real_t*)RD_MALLOC(sizeof(rd_real_t) * nj * 3);
    chain->T_joint_offset = (rd_real_t*)RD_MALLOC(sizeof(rd_real_t) * n * 16);
    chain->frame_names = (char**)RD_MALLOC(sizeof(char*) * n);
    chain->parent_path = (rd_idx_t*)RD_MALLOC(sizeof(rd_idx_t) * n * n);
    chain->parent_path_len = (rd_int_t*)RD_MALLOC(sizeof(rd_int_t) * n);
    chain->s_axis = (rd_idx_t*)RD_CALLOC(n, sizeof(rd_idx_t));
    chain->s_sign = (rd_real_t*)RD_CALLOC(n, sizeof(rd_real_t));
    chain->dyn_order = (rd_idx_t*)RD_MALLOC(sizeof(rd_idx_t) * n);
    chain->dyn_parent = (rd_idx_t*)RD_MALLOC(sizeof(rd_idx_t) * n);
    chain->dyn_child = (rd_idx_t*)RD_MALLOC(sizeof(rd_idx_t) * n);
    chain->dyn_child_start = (rd_int_t*)RD_CALLOC(n + 1, sizeof(rd_int_t));
    chain->dyn = (rd_dyn_node_t*)RD_MALLOC(sizeof(rd_dyn_node_t) * n);
    chain->dyn_slot = (rd_idx_t*)RD_MALLOC(sizeof(rd_idx_t) * n);
    chain->inertia_compact = (rd_real_t*)RD_CALLOC(n * RD_INERTIA_COMPACT_LEN,
                                                   sizeof(rd_real_t));

    for (rd_int_t i = 0; i < n; ++i) {
        chain->frame_names[i] = (char*)RD_MALLOC(16);
    }

    /* Check allocations */
    if (!chain->parent_list || !chain->topo_order || !chain->children_count ||
        !chain->children_list || !chain->joint_idx || !chain->joint_type ||
        !chain->axes || !chain->T_joint_offset ||
        !chain->frame_names || !chain->parent_path || !chain->parent_path_len ||
        !chain->inertia_compact ||
        !chain->s_axis || !chain->s_sign || !chain->dyn_order || !chain->dyn_parent || !chain->dyn_child ||
        !chain->dyn_child_start || !chain->dyn || !chain->dyn_slot) {
        rd_chain_free(chain);
        return RD_ERR_ALLOC_FAILED;
    }

    /* Fill parent list, names, and joint data */
    rd_int_t joint_counter = 0;
    for (rd_int_t i = 0; i < n; ++i) {
        const rd_link_t* L = &model->links[i];
        chain->parent_list[i] = L->parent_idx;
        strncpy(chain->frame_names[i], L->name, 15);
        chain->frame_names[i][15] = '\0';
        chain->joint_type[i] = (rd_int_t)L->joint.type;
        
        if (L->joint.type == RD_JOINT_REVOLUTE || L->joint.type == RD_JOINT_PRISMATIC) {
            chain->joint_idx[i] = (rd_idx_t)joint_counter;
            rd_real_t v[3];
            axis_to_vec(L->joint.axis, v);
            chain->axes[joint_counter*3 + 0] = v[0];
            chain->axes[joint_counter*3 + 1] = v[1];
            chain->axes[joint_counter*3 + 2] = v[2];
            joint_counter++;
        } else {
            chain->joint_idx[i] = -1;
        }
        
        /* Build joint offset transform */
        rd_real_t R[9];
        rd_rot_rpy((rd_real_t)L->rpy_parent.x, 
                   (rd_real_t)L->rpy_parent.y, 
                   (rd_real_t)L->rpy_parent.z, R);
        rd_real_t t[3] = {
            (rd_real_t)L->pos_parent.x,
            (rd_real_t)L->pos_parent.y,
            (rd_real_t)L->pos_parent.z
        };
        rd_rot_to_mat4(R, t, &chain->T_joint_offset[i*16]);
        
        /* Link offset is identity */
    }

    /* Build children lists */
    for (rd_int_t i = 0; i < n; ++i) {
        rd_idx_t p = chain->parent_list[i];
        if (p >= 0 && p < n) {
            rd_int_t idx = chain->children_count[p];
            chain->children_list[p*n + idx] = (rd_idx_t)i;
            chain->children_count[p]++;
        }
    }
    
    chain->max_children = 0;
    for (rd_int_t i = 0; i < n; ++i) {
        if (chain->children_count[i] > chain->max_children) {
            chain->max_children = chain->children_count[i];
        }
    }

    /* Build parent path indices (root to each node) */
    for (rd_int_t i = 0; i < n; ++i) {
        rd_idx_t path[RD_MAX_LINKS];
        rd_int_t plen = 0;
        rd_idx_t cur = (rd_idx_t)i;
        
        while (cur >= 0 && plen < RD_MAX_LINKS) {
            path[plen++] = cur;
            cur = chain->parent_list[cur];
        }
        
        /* Reverse to get root->node order */
        for (rd_int_t j = 0; j < plen/2; ++j) {
            rd_idx_t tmp = path[j];
            path[j] = path[plen-1-j];
            path[plen-1-j] = tmp;
        }
        
        chain->parent_path_len[i] = plen;
        for (rd_int_t j = 0; j < plen; ++j) {
            chain->parent_path[i*n + j] = path[j];
        }
    }

    /* Per-link 6x6 spatial inertias. Only the fold below needs them, and only
     * the folded ten-number form survives, so this is a scratch allocation
     * rather than 36 floats a node kept for the life of the chain. */
    rd_real_t* SI = (rd_real_t*)RD_CALLOC((size_t)n * 36, sizeof(rd_real_t));
    if (!SI) { rd_chain_free(chain); return RD_ERR_ALLOC_FAILED; }

    for (rd_int_t i = 0; i < n; ++i) {
        const rd_link_t* L = &model->links[i];
        rd_real_t* Is = &SI[i*36];

        rd_real_t m = (rd_real_t)L->inertia.mass;
        rd_real_t cx = (rd_real_t)L->inertia.com.x;
        rd_real_t cy = (rd_real_t)L->inertia.com.y;
        rd_real_t cz = (rd_real_t)L->inertia.com.z;

        rd_real_t Ixx = (rd_real_t)L->inertia.I_com.Ixx;
        rd_real_t Iyy = (rd_real_t)L->inertia.I_com.Iyy;
        rd_real_t Izz = (rd_real_t)L->inertia.I_com.Izz;
        rd_real_t Ixy = (rd_real_t)L->inertia.I_com.Ixy;
        rd_real_t Ixz = (rd_real_t)L->inertia.I_com.Ixz;
        rd_real_t Iyz = (rd_real_t)L->inertia.I_com.Iyz;

        /* Skew matrix [c]x */
        rd_real_t cx_mat[9] = {
            RD_REAL(0.0), -cz, cy,
            cz, RD_REAL(0.0), -cx,
            -cy, cx, RD_REAL(0.0)
        };

        /* [c]x * [c]x */
        rd_real_t cx_sq[9];
        rd_mat3_mul(cx_mat, cx_mat, cx_sq);

        /* Initialize to zero */
        for (rd_int_t k = 0; k < 36; ++k) Is[k] = RD_REAL(0.0);

        /* Upper-left 3x3: m * I */
        Is[0*6+0] = m;
        Is[1*6+1] = m;
        Is[2*6+2] = m;

        /* Upper-right 3x3: -m * [c]x */
        for (rd_int_t r = 0; r < 3; ++r) {
            for (rd_int_t c = 0; c < 3; ++c) {
                Is[r*6 + (c+3)] = -m * cx_mat[r*3 + c];
            }
        }

        /* Lower-left 3x3: m * [c]x */
        for (rd_int_t r = 0; r < 3; ++r) {
            for (rd_int_t c = 0; c < 3; ++c) {
                Is[(r+3)*6 + c] = m * cx_mat[r*3 + c];
            }
        }

        /* Lower-right 3x3: Ic - m * [c]x * [c]x */
        rd_real_t Ic[9] = {
            Ixx, Ixy, Ixz,
            Ixy, Iyy, Iyz,
            Ixz, Iyz, Izz
        };
        for (rd_int_t r = 0; r < 3; ++r) {
            for (rd_int_t c = 0; c < 3; ++c) {
                Is[(r+3)*6 + (c+3)] = Ic[r*3 + c] - m * cx_sq[r*3 + c];
            }
        }

        /* Packed form: m, c, and the six unique entries of J. */
        rd_real_t* icp = &chain->inertia_compact[i * RD_INERTIA_COMPACT_LEN];
        icp[0] = m;
        icp[1] = cx; icp[2] = cy; icp[3] = cz;
        icp[4] = Is[3*6 + 3];   /* Jxx */
        icp[5] = Is[4*6 + 4];   /* Jyy */
        icp[6] = Is[5*6 + 5];   /* Jzz */
        icp[7] = Is[3*6 + 4];   /* Jxy */
        icp[8] = Is[3*6 + 5];   /* Jxz */
        icp[9] = Is[4*6 + 5];   /* Jyz */
    }

    /* Build topological order (simple BFS from roots) */
    rd_int_t count = 0;
    for (rd_int_t i = 0; i < n; ++i) {
        if (chain->parent_list[i] == -1) {
            chain->topo_order[count++] = (rd_idx_t)i;
            /* Add children recursively */
            for (rd_int_t j = 0; j < n; ++j) {
                if (chain->parent_list[j] == i) {
                    chain->topo_order[count++] = (rd_idx_t)j;
                    for (rd_int_t k = 0; k < n; ++k) {
                        if (chain->parent_list[k] == j) {
                            chain->topo_order[count++] = (rd_idx_t)k;
                        }
                    }
                }
            }
        }
    }
    
    /* Ensure all nodes are included */
    for (rd_int_t i = 0; i < n; ++i) {
        rd_int_t present = 0;
        for (rd_int_t j = 0; j < count; ++j) {
            if (chain->topo_order[j] == i) {
                present = 1;
                break;
            }
        }
        if (!present) {
            chain->topo_order[count++] = (rd_idx_t)i;
        }
    }

    /* Motion subspace as an axis and a sign. A revolute joint's twist is
     * angular, so it selects one of components 3..5; a prismatic joint's is
     * linear, selecting 0..2. */
    for (rd_int_t i = 0; i < n; ++i) {
        chain->s_axis[i] = 0;
        chain->s_sign[i] = RD_REAL(0.0);
        rd_idx_t j = chain->joint_idx[i];
        if (j < 0) continue;
        if (chain->joint_type[i] != RD_JOINT_REVOLUTE &&
            chain->joint_type[i] != RD_JOINT_PRISMATIC) continue;
        const rd_real_t* ax = &chain->axes[j*3];
        rd_int_t k = (ax[0] != RD_REAL(0.0)) ? 0 : ((ax[1] != RD_REAL(0.0)) ? 1 : 2);
        chain->s_axis[i] = (rd_idx_t)((chain->joint_type[i] == RD_JOINT_REVOLUTE)
                                      ? (3 + k) : k);
        chain->s_sign[i] = ax[k];
    }

    /* ---- Dynamics tree ---------------------------------------------------
     * Nearest moving ancestor, by walking up rather than relying on the order
     * topo_order happens to have produced. */
    for (rd_int_t i = 0; i < n; ++i) {
        rd_idx_t p = chain->parent_list[i];
        while (p != -1 && !rd_chain_node_is_dynamic(chain, p)) {
            p = chain->parent_list[p];
        }
        chain->dyn_parent[i] = p;
    }

    /* Dynamics nodes, keeping topo_order's ordering so parents still precede
     * children. */
    chain->n_dyn = 0;
    for (rd_int_t ti = 0; ti < n; ++ti) {
        rd_idx_t node = chain->topo_order[ti];
        if (rd_chain_node_is_dynamic(chain, node)) {
            chain->dyn_order[chain->n_dyn++] = node;
        }
    }

    /* The per-node control block the traversals read, gathered once. */
    for (rd_int_t i = 0; i < n; ++i) chain->dyn_slot[i] = -1;
    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        rd_idx_t node = chain->dyn_order[di];
        rd_dyn_node_t* d = &chain->dyn[di];
        rd_idx_t j = chain->joint_idx[node];
        d->node   = node;
        d->parent = chain->parent_list[node];
        d->danc   = chain->dyn_parent[node];
        d->jidx   = j;
        d->vidx   = (j < 0) ? (rd_idx_t)-1
                            : (rd_idx_t)(chain->has_floating_base ? (6 + j) : j);
        d->s_axis = chain->s_axis[node];
        d->s_sign = chain->s_sign[node];
        chain->dyn_slot[node] = (rd_idx_t)di;
    }

    /* CSR of dynamics children, indexed by node. */
    for (rd_int_t i = 0; i <= n; ++i) chain->dyn_child_start[i] = 0;
    for (rd_int_t i = 0; i < n; ++i) {
        if (!rd_chain_node_is_dynamic(chain, (rd_idx_t)i)) continue;
        rd_idx_t p = chain->dyn_parent[i];
        if (p >= 0) chain->dyn_child_start[p + 1]++;
    }
    for (rd_int_t i = 0; i < n; ++i) {
        chain->dyn_child_start[i + 1] += chain->dyn_child_start[i];
    }
    {
        rd_int_t* fill = (rd_int_t*)RD_CALLOC(n, sizeof(rd_int_t));
        if (!fill) { RD_FREE(SI); rd_chain_free(chain); return RD_ERR_ALLOC_FAILED; }
        for (rd_int_t ti = 0; ti < n; ++ti) {
            rd_idx_t node = chain->topo_order[ti];
            if (!rd_chain_node_is_dynamic(chain, node)) continue;
            rd_idx_t p = chain->dyn_parent[node];
            if (p < 0) continue;
            chain->dyn_child[chain->dyn_child_start[p] + fill[p]] = node;
            fill[p]++;
        }
        RD_FREE(fill);
    }

    /* ---- Fold fixed links into their nearest moving ancestor -------------
     * A fixed node's pose in that ancestor is a constant: its own offsets
     * composed with those of any fixed nodes in between. */
    {
        rd_real_t* Tf = (rd_real_t*)RD_MALLOC(sizeof(rd_real_t) * (size_t)n * 16);
        if (!Tf) { RD_FREE(SI); rd_chain_free(chain); return RD_ERR_ALLOC_FAILED; }

        for (rd_int_t i = 0; i < n; ++i) {
            rd_real_t acc[16], tmp[16];
            memcpy(acc, &chain->T_joint_offset[i*16], 16 * sizeof(rd_real_t));
            rd_idx_t p = chain->parent_list[i];
            while (p != -1 && !rd_chain_node_is_dynamic(chain, p)) {
                rd_mat4_mul_se3(&chain->T_joint_offset[p*16], acc, tmp);
                memcpy(acc, tmp, 16 * sizeof(rd_real_t));
                p = chain->parent_list[p];
            }
            memcpy(&Tf[i*16], acc, 16 * sizeof(rd_real_t));
        }

        /* Each fixed node folds straight into its dynamics ancestor, never
         * into an intermediate fixed node, so no entry is read after it has
         * been written and the order does not matter. */
        for (rd_int_t i = 0; i < n; ++i) {
            if (rd_chain_node_is_dynamic(chain, (rd_idx_t)i)) continue;
            rd_idx_t a = chain->dyn_parent[i];
            if (a < 0) continue;
            rd_real_t Ti[16];
            rd_mat4_inv(&Tf[i*16], Ti);
            rd_spatial_inertia_congruence(Ti, &SI[i*36], &SI[a*36]);
            memset(&SI[i*36], 0, 36 * sizeof(rd_real_t));
        }
        RD_FREE(Tf);
    }

    /* Repack. A sum of rigid-body spatial inertias, and a congruence of one,
     * is still a rigid-body spatial inertia, so the ten-number form survives
     * the fold -- it just describes the composite body now. */
    for (rd_int_t i = 0; i < n; ++i) {
        const rd_real_t* Is = &SI[i*36];
        rd_real_t* icp = &chain->inertia_compact[i * RD_INERTIA_COMPACT_LEN];
        rd_real_t m = Is[0];
        icp[0] = m;
        icp[1] = -Is[4*6 + 2];   /* h = m*c, read straight off the m*[c]x block */
        icp[2] =  Is[3*6 + 2];
        icp[3] = -Is[3*6 + 1];
        icp[4] = Is[3*6 + 3];   /* Jxx */
        icp[5] = Is[4*6 + 4];   /* Jyy */
        icp[6] = Is[5*6 + 5];   /* Jzz */
        icp[7] = Is[3*6 + 4];   /* Jxy */
        icp[8] = Is[3*6 + 5];   /* Jxz */
        icp[9] = Is[4*6 + 5];   /* Jyz */
    }

    RD_FREE(SI);
    return RD_OK;
}

void rd_chain_free(rd_chain_t* chain) {
    if (!chain) return;
    
    RD_FREE(chain->parent_list);
    RD_FREE(chain->topo_order);
    RD_FREE(chain->children_count);
    RD_FREE(chain->children_list);
    RD_FREE(chain->joint_idx);
    RD_FREE(chain->joint_type);
    RD_FREE(chain->axes);
    RD_FREE(chain->T_joint_offset);
    
    if (chain->frame_names) {
        for (rd_int_t i = 0; i < chain->n_nodes; ++i) {
            RD_FREE(chain->frame_names[i]);
        }
        RD_FREE(chain->frame_names);
    }
    
    RD_FREE(chain->parent_path);
    RD_FREE(chain->parent_path_len);
    RD_FREE(chain->inertia_compact);
    RD_FREE(chain->s_axis);
    RD_FREE(chain->s_sign);
    RD_FREE(chain->dyn_order);
    RD_FREE(chain->dyn_parent);
    RD_FREE(chain->dyn_child);
    RD_FREE(chain->dyn_child_start);
    RD_FREE(chain->dyn);
    RD_FREE(chain->dyn_slot);
    
    /* Zero out the struct */
    memset(chain, 0, sizeof(rd_chain_t));
}

rd_idx_t rd_chain_find_frame(const rd_chain_t* chain, const char* name) {
    if (!chain || !name) return -1;
    
    for (rd_int_t i = 0; i < chain->n_nodes; ++i) {
        if (strcmp(chain->frame_names[i], name) == 0) {
            return (rd_idx_t)i;
        }
    }
    return -1;
}
