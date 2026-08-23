#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
urdf2c.py -- Convert a URDF into a RobotDynamics `rd_model_t` C header.

The C model (rd_model.h) is more restrictive than URDF, so this converter
enforces / checks the following:

  * joint axes must be axis-aligned (+/- X, Y, Z)  -> rd_axis_t
  * link names are truncated to 15 chars          -> rd_link_t.name[16]
  * links are emitted so that parent_idx < child_idx, which is what
    rd_chain_build()'s topological ordering actually relies on.
  * only revolute / continuous / prismatic / fixed joints are supported.
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from collections import OrderedDict

AXIS_ENUM = {
    (1, 0, 0): "RD_AXIS_X",
    (0, 1, 0): "RD_AXIS_Y",
    (0, 0, 1): "RD_AXIS_Z",
    (-1, 0, 0): "RD_AXIS_NEG_X",
    (0, -1, 0): "RD_AXIS_NEG_Y",
    (0, 0, -1): "RD_AXIS_NEG_Z",
}


def parse_triple(text, default=(0.0, 0.0, 0.0)):
    if text is None:
        return default
    parts = [float(v) for v in text.replace(",", " ").split()]
    if len(parts) != 3:
        raise ValueError(f"expected 3 numbers, got {text!r}")
    return tuple(parts)


def quantise_axis(xyz):
    """Map a URDF axis onto the nearest rd_axis_t, or fail loudly."""
    norm = math.sqrt(sum(v * v for v in xyz))
    if norm < 1e-9:
        raise ValueError("zero-length joint axis")
    unit = tuple(v / norm for v in xyz)
    for key, enum in AXIS_ENUM.items():
        if all(abs(u - k) < 1e-6 for u, k in zip(unit, key)):
            return enum
    raise ValueError(
        f"axis {xyz} is not axis-aligned; rd_axis_t cannot represent it"
    )


class Link:
    def __init__(self, name):
        self.name = name
        self.mass = 0.0
        self.com = (0.0, 0.0, 0.0)
        # Ixx Iyy Izz Ixy Ixz Iyz
        self.inertia = (0.0,) * 6
        self.parent = None          # link name
        self.joint_name = None
        self.joint_order = 1 << 30  # position of this link's joint in the URDF
        self.origin_xyz = (0.0, 0.0, 0.0)
        self.origin_rpy = (0.0, 0.0, 0.0)
        self.joint_type = "RD_JOINT_FIXED"
        self.joint_axis = "RD_AXIS_X"
        self.q_min = 0.0
        self.q_max = 0.0
        self.dq_max = 0.0
        self.tau_max = 0.0
        self.damping = 0.0
        self.friction = 0.0


def load_urdf(path):
    root = ET.parse(path).getroot()

    links = OrderedDict()
    for el in root.findall("link"):
        name = el.get("name")
        link = Link(name)
        inertial = el.find("inertial")
        if inertial is not None:
            mass_el = inertial.find("mass")
            if mass_el is not None:
                link.mass = float(mass_el.get("value", "0"))
            origin = inertial.find("origin")
            if origin is not None:
                link.com = parse_triple(origin.get("xyz"))
            it = inertial.find("inertia")
            if it is not None:
                link.inertia = (
                    float(it.get("ixx", "0")),
                    float(it.get("iyy", "0")),
                    float(it.get("izz", "0")),
                    float(it.get("ixy", "0")),
                    float(it.get("ixz", "0")),
                    float(it.get("iyz", "0")),
                )
        links[name] = link

    for order, el in enumerate(root.findall("joint")):
        jtype = el.get("type")
        child = el.find("child").get("link")
        parent = el.find("parent").get("link")
        if child not in links:
            raise ValueError(f"joint references unknown child link {child!r}")
        link = links[child]
        link.parent = parent
        link.joint_name = el.get("name")
        link.joint_order = order

        origin = el.find("origin")
        if origin is not None:
            link.origin_xyz = parse_triple(origin.get("xyz"))
            link.origin_rpy = parse_triple(origin.get("rpy"))

        if jtype in ("revolute", "continuous"):
            link.joint_type = "RD_JOINT_REVOLUTE"
        elif jtype == "prismatic":
            link.joint_type = "RD_JOINT_PRISMATIC"
        elif jtype == "fixed":
            link.joint_type = "RD_JOINT_FIXED"
        else:
            raise ValueError(f"unsupported joint type {jtype!r} on {el.get('name')}")

        if link.joint_type != "RD_JOINT_FIXED":
            axis_el = el.find("axis")
            xyz = parse_triple(axis_el.get("xyz") if axis_el is not None else None,
                               default=(1.0, 0.0, 0.0))
            link.joint_axis = quantise_axis(xyz)

            limit = el.find("limit")
            if limit is not None:
                link.q_min = float(limit.get("lower", "0"))
                link.q_max = float(limit.get("upper", "0"))
                link.dq_max = float(limit.get("velocity", "0"))
                link.tau_max = float(limit.get("effort", "0"))
            dyn = el.find("dynamics")
            if dyn is not None:
                link.damping = float(dyn.get("damping", "0"))
                link.friction = float(dyn.get("friction", "0"))

    return links


def topo_sort(links):
    """
    Order links so every parent precedes its children, which is the invariant
    rd_chain_build() relies on when it falls back to link-index order.

    The traversal is depth-first with children visited in URDF joint-declaration
    order. That is deliberate: it is the same order Pinocchio (and therefore
    bard) assigns joint indices in, so a q/qd vector is interchangeable between
    the two without a permutation. Breadth-first would also satisfy the parent-
    before-child invariant, but on a branched robot it silently scrambles q --
    a quadruped would come out as all four hips, then all four thighs, instead
    of leg by leg.
    """
    roots = [n for n, l in links.items() if l.parent is None]
    if len(roots) != 1:
        raise ValueError(f"expected exactly one root link, found {roots}")

    children = {n: [] for n in links}
    for name, link in links.items():
        if link.parent is not None:
            children[link.parent].append(name)
    for kids in children.values():
        kids.sort(key=lambda n: links[n].joint_order)

    ordered, stack = [], [roots[0]]
    while stack:
        node = stack.pop()
        ordered.append(node)
        stack.extend(reversed(children[node]))

    if len(ordered) != len(links):
        missing = set(links) - set(ordered)
        raise ValueError(f"disconnected links: {sorted(missing)}")
    return ordered


def cname(name, used):
    """Shorten a URDF link name to fit rd_link_t.name[16] and stay unique."""
    short = name[:15]
    if short not in used:
        used.add(short)
        return short
    for i in range(1, 100):
        suffix = str(i)
        cand = name[: 15 - len(suffix)] + suffix
        if cand not in used:
            used.add(cand)
            return cand
    raise ValueError(f"cannot make {name!r} unique within 15 chars")


def R(v):
    return f"RD_REAL({v!r})"


def emit(model_name, links, order, floating_base, src_path):
    idx = {name: i for i, name in enumerate(order)}
    n_links = len(order)
    n_joints = sum(
        1 for n in order if links[n].joint_type != "RD_JOINT_FIXED"
    )
    nv = (6 + n_joints) if floating_base else n_joints
    nq = (7 + n_joints) if floating_base else n_joints

    guard = f"RD_MODEL_{model_name.upper()}_H"
    used_names = set()
    short_names = {n: cname(n, used_names) for n in order}

    out = []
    w = out.append
    w("/* SPDX-License-Identifier: Apache-2.0 */")
    w("/**")
    w(f" * @file model_{model_name}.h")
    w(f" * @brief `{model_name}` robot model, generated from {src_path}")
    w(" *")
    w(" * Generated by tools/urdf2c.py -- do not edit by hand.")
    w(" *")
    w(f" *   links  : {n_links}")
    w(f" *   joints : {n_joints}   (revolute/prismatic; fixed joints excluded)")
    w(f" *   base   : {'floating' if floating_base else 'fixed'}")
    w(f" *   nq={nq}  nv={nv}")
    w(" *")
    w(" * Joint order, i.e. the meaning of each element of q_joints/qd_joints.")
    w(" * Links are emitted depth-first in URDF joint order, so this matches the")
    w(" * indexing Pinocchio and bard use for the same URDF.")
    w(" *")
    actuated = [n for n in order if links[n].joint_type != "RD_JOINT_FIXED"]
    for j, name in enumerate(actuated):
        w(f" *   q[{j}]  {links[name].joint_name}  ->  link {name}")
    if floating_base:
        w(" *")
        w(" * q_base  = [x, y, z, qw, qx, qy, qz]  (quaternion scalar FIRST)")
        w(" * qd_base = [vx, vy, vz, wx, wy, wz]   (root body frame)")
    w(" */")
    w("")
    w(f"#ifndef {guard}")
    w(f"#define {guard}")
    w("")
    w('#include "robot_dynamics.h"')
    w("")
    w(f"#define {model_name.upper()}_N_LINKS   {n_links}")
    w(f"#define {model_name.upper()}_N_JOINTS  {n_joints}")
    w(f"#define {model_name.upper()}_NQ        {nq}")
    w(f"#define {model_name.upper()}_NV        {nv}")
    w("")
    w(f"static inline const rd_model_t* {model_name}_model_get(void) {{")
    w("    static const rd_model_t model = {")
    w(f'        .name = "{model_name[:31]}",')
    w(f"        .num_links = {n_links},")
    w(f"        .num_joints = {n_joints},")
    w(f"        .use_floating_base = {1 if floating_base else 0},")
    w(f"        .total_dof = {nv},")
    w("        .gravity = {RD_REAL(0.0), RD_REAL(0.0), -RD_GRAVITY},")
    w("        .links = {")

    for i, name in enumerate(order):
        link = links[name]
        parent_idx = -1 if link.parent is None else idx[link.parent]
        # The base link's own "joint" describes how the model attaches to world.
        if parent_idx == -1:
            jtype = "RD_JOINT_FLOATING" if floating_base else "RD_JOINT_FIXED"
        else:
            jtype = link.joint_type
        ixx, iyy, izz, ixy, ixz, iyz = link.inertia

        w(f"            /* [{i}] {name} */")
        w(f"            [{i}] = {{")
        w(f'                .name = "{short_names[name]}",')
        w(f"                .parent_idx = {parent_idx},")
        w("                .pos_parent = {%s, %s, %s},"
          % tuple(R(v) for v in link.origin_xyz))
        w("                .rpy_parent = {%s, %s, %s},"
          % tuple(R(v) for v in link.origin_rpy))
        w("                .inertia = {")
        w(f"                    .mass = {R(link.mass)},")
        w("                    .com = {%s, %s, %s},"
          % tuple(R(v) for v in link.com))
        w("                    .I_com = {")
        w(f"                        .Ixx = {R(ixx)}, .Iyy = {R(iyy)}, .Izz = {R(izz)},")
        w(f"                        .Ixy = {R(ixy)}, .Ixz = {R(ixz)}, .Iyz = {R(iyz)}")
        w("                    }")
        w("                },")
        w("                .joint = {")
        w(f"                    .type = {jtype},")
        w(f"                    .axis = {link.joint_axis},")
        w(f"                    .q_min = {R(link.q_min)}, .q_max = {R(link.q_max)},")
        w(f"                    .dq_max = {R(link.dq_max)}, .tau_max = {R(link.tau_max)},")
        w(f"                    .damping = {R(link.damping)}, .friction = {R(link.friction)},")
        # URDF carries no armature; rd_chain_set_armature() fills it in.
        w(f"                    .armature = {R(0.0)}")
        w("                }")
        w("            },")

    w("        }")
    w("    };")
    w("    return &model;")
    w("}")
    w("")
    w(f"#endif /* {guard} */")
    return "\n".join(out) + "\n", n_links, n_joints, nv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("-n", "--name", required=True, help="model identifier, e.g. go2")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--floating-base", action="store_true")
    args = ap.parse_args()

    links = load_urdf(args.urdf)
    order = topo_sort(links)
    text, n_links, n_joints, nv = emit(
        args.name, links, order, args.floating_base, args.urdf
    )
    with open(args.output, "w") as fh:
        fh.write(text)
    print(
        f"{args.name:10s} links={n_links:3d} joints={n_joints:3d} nv={nv:3d} "
        f"base={'floating' if args.floating_base else 'fixed':8s} -> {args.output}"
    )


if __name__ == "__main__":
    sys.exit(main())
