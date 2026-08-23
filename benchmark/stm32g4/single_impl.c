/* SPDX-License-Identifier: Apache-2.0 */
/*
 * SINGLE=1 builds the suite against dist/robot_dynamics.h rather than the
 * RobotDynamics/ tree, so the shipped file is exercised the way somebody who
 * downloaded it would -- including whatever RD_MATH_BACKEND is set to, which
 * has to reach this translation unit as much as any other.
 *
 * This is the whole of the "one .c file" the header asks for.
 */
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
