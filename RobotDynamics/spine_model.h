/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file spine_model.h
 * @brief Spine Robot Model Definition
 * 
 * 3-DOF spine with floating base (9 total DOF)
 * Structure: hind_body -> hind_spine -> front_spine -> front_body
 */

#ifndef SPINE_MODEL_H
#define SPINE_MODEL_H

#include "robot_dynamics.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the spine robot model
 * @return Pointer to static model definition
 */
static inline const rd_model_t* spine_model_get(void) {
    static const rd_model_t model = {
        .name = "spine",
        .num_links = 5,
        .num_joints = 3,
        .use_floating_base = 1,
        .total_dof = 9,  /* 6 (base) + 3 (joints) */
        
        .gravity = {RD_REAL(0.0), RD_REAL(0.0), -RD_GRAVITY},
        
        .links = {
            /* Link 0: hind_body (base) */
            [0] = {
                .name = "hind_body",
                .parent_idx = -1,
                .pos_parent = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)},
                .rpy_parent = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)},
                .inertia = {
                    .mass = RD_REAL(0.377),
                    .com = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)},
                    .I_com = {
                        .Ixx = RD_REAL(0.0001832),
                        .Iyy = RD_REAL(0.0003327),
                        .Izz = RD_REAL(0.0002337),
                        .Ixy = RD_REAL(0.0), .Ixz = RD_REAL(0.0), .Iyz = RD_REAL(0.0)
                    }
                },
                .joint = {
                    .type = RD_JOINT_FLOATING,
                    .axis = RD_AXIS_X,
                    .q_min = -RD_PI, .q_max = RD_PI,
                    .dq_max = RD_REAL(100.0), .tau_max = RD_REAL(0.0),
                    .damping = RD_REAL(0.0), .friction = RD_REAL(0.0)
                }
            },
            
            /* Link 1: hind_spine */
            [1] = {
                .name = "hind_spine",
                .parent_idx = 0,
                .pos_parent = {RD_REAL(0.00454), RD_REAL(-0.017638872), RD_REAL(0.0)},
                .rpy_parent = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)},
                .inertia = {
                    .mass = RD_REAL(0.399),
                    .com = {RD_REAL(0.109455866), RD_REAL(0.014797864), RD_REAL(0.0)},
                    .I_com = {
                        .Ixx = RD_REAL(0.0001953),
                        .Iyy = RD_REAL(0.0005357),
                        .Izz = RD_REAL(0.0004759),
                        .Ixy = RD_REAL(0.0), .Ixz = RD_REAL(0.0), .Iyz = RD_REAL(0.0)
                    }
                },
                .joint = {
                    .type = RD_JOINT_REVOLUTE,
                    .axis = RD_AXIS_Y,
                    .q_min = RD_REAL(-10.0), .q_max = RD_REAL(10.0),
                    .dq_max = RD_REAL(100.0), .tau_max = RD_REAL(12.0),
                    .damping = RD_REAL(0.0), .friction = RD_REAL(0.0)
                }
            },
            
            /* Link 2: front_spine */
            [2] = {
                .name = "front_spine",
                .parent_idx = 1,
                .pos_parent = {RD_REAL(0.1175), RD_REAL(0.036), RD_REAL(0.0)},
                .rpy_parent = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)},
                .inertia = {
                    .mass = RD_REAL(0.052),
                    .com = {RD_REAL(0.05875), RD_REAL(0.0073), RD_REAL(0.0)},
                    .I_com = {
                        .Ixx = RD_REAL(0.00001),
                        .Iyy = RD_REAL(0.000084358),
                        .Izz = RD_REAL(0.000082313),
                        .Ixy = RD_REAL(0.0), .Ixz = RD_REAL(0.0), .Iyz = RD_REAL(0.0)
                    }
                },
                .joint = {
                    .type = RD_JOINT_REVOLUTE,
                    .axis = RD_AXIS_Y,
                    .q_min = RD_REAL(-10.0), .q_max = RD_REAL(10.0),
                    .dq_max = RD_REAL(100.0), .tau_max = RD_REAL(12.0),
                    .damping = RD_REAL(0.0), .friction = RD_REAL(0.0)
                }
            },
            
            /* Link 3: front_body */
            [3] = {
                .name = "front_body",
                .parent_idx = 2,
                .pos_parent = {RD_REAL(0.1175), RD_REAL(0.0), RD_REAL(0.0)},
                .rpy_parent = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)},
                .inertia = {
                    .mass = RD_REAL(0.377197524),
                    .com = {RD_REAL(0.00454141), RD_REAL(-0.017638872), RD_REAL(0.0)},
                    .I_com = {
                        .Ixx = RD_REAL(0.0001832),
                        .Iyy = RD_REAL(0.0003327),
                        .Izz = RD_REAL(0.0002337),
                        .Ixy = RD_REAL(0.0), .Ixz = RD_REAL(0.0), .Iyz = RD_REAL(0.0)
                    }
                },
                .joint = {
                    .type = RD_JOINT_REVOLUTE,
                    .axis = RD_AXIS_Y,
                    .q_min = RD_REAL(-10.0), .q_max = RD_REAL(10.0),
                    .dq_max = RD_REAL(100.0), .tau_max = RD_REAL(12.0),
                    .damping = RD_REAL(0.0), .friction = RD_REAL(0.0)
                }
            },
            /* Link 4: end_effector (Fixed Joint) */
            [4] = {
                .name = "end_effector",
                .parent_idx = 3,

                .pos_parent = {RD_REAL(0.05868359), RD_REAL(0.0), RD_REAL(0.0)},
                .rpy_parent = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)},
                
                .inertia = {
                    .mass = RD_REAL(0.0),
                    .com = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)},
                    .I_com = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)}
                },
                
                .joint = {
                    .type = RD_JOINT_FIXED,
                    .axis = RD_AXIS_X,
                    .q_min = 0, .q_max = 0
                }
            }
        }
    };
    
    return &model;
}

#ifdef __cplusplus
}
#endif

#endif /* SPINE_MODEL_H */
