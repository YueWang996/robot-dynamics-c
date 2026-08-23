#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
validate.py -- cross-check RobotDynamics against Pinocchio.

Builds tools/validate_dump.c, feeds it random configurations, and compares its
forward kinematics, spatial velocities, RNEA torques, CRBA mass matrix and
geometric Jacobians against Pinocchio's on the same URDF.

    python3 tools/validate.py --urdf-root /path/to/bard [--double] [-n 20]

Convention mapping (this is most of what the comparison is actually testing):

  quaternion   RobotDynamics packs the base quaternion scalar-first,
               [x y z qw qx qy qz]; Pinocchio's free-flyer is scalar-last,
               [x y z qx qy qz qw].
  base twist   both express the free-flyer velocity in the ROOT BODY frame.
  spatial      both order spatial vectors [linear, angular].
  fixed joints Pinocchio folds fixed-joint links into their parent body and
               keeps them only as frames; RobotDynamics keeps them as zero-DOF
               nodes. Inertially the two are equivalent, so M and tau must still
               agree -- only the link/joint counts differ.
"""

import argparse
import os
import subprocess
import sys
import tempfile

import numpy as np
import pinocchio as pin

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import urdf2c

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

ROBOTS = {
    #  name          urdf path (relative to --urdf-root)            floating base
    "simple_arm": ("examples/simple_arm.urdf", False),
    "spine":      ("tests/spine.urdf", True),
    "xarm7":      ("examples/example_robots/xarm_description/urdf/xarm7.urdf", False),
    "go2":        ("examples/example_robots/go2_description/urdf/go2.urdf", True),
    # Unitree G1, 29 DOF. From unitreerobotics/unitree_ros (BSD-3-Clause);
    # drop g1_29dof.urdf in beside the others and this row starts running.
    "g1":         ("examples/example_robots/g1_description/g1_29dof.urdf", True),
}


def build(double: bool, extra: list | None = None) -> str:
    tag = ("f64" if double else "f32") + ("_" + "_".join(
        e.lstrip("-D").replace("=", "") for e in extra) if extra else "")
    out = os.path.join(tempfile.gettempdir(), f"rd_validate_{tag}")
    cmd = [
        "cc", "-O2", "-std=c11", "-g",
        *(extra or []),
        f"-DRD_USE_SINGLE_PRECISION={0 if double else 1}",
        "-DRD_MAX_LINKS=48", "-DRD_MAX_JOINTS=32",
        "-I", os.path.join(ROOT, "RobotDynamics"),
        "-I", os.path.join(ROOT, "benchmark", "models"),
        os.path.join(HERE, "validate_dump.c"),
        os.path.join(ROOT, "RobotDynamics", "rd_algorithms.c"),
        os.path.join(ROOT, "RobotDynamics", "rd_chain.c"),
        os.path.join(ROOT, "RobotDynamics", "rd_state.c"),
        "-lm", "-o", out,
    ]
    subprocess.run(cmd, check=True)
    return out


def run_dump(binary, robot, payload):
    p = subprocess.run([binary, robot], input=payload, capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError(f"validate_dump failed: {p.stderr}")
    out = {"T": {}, "V": {}, "M": {}, "JW": {}, "JL": {}, "NAME": {}}
    for line in p.stdout.splitlines():
        f = line.split()
        tag = f[0]
        if tag in ("T", "V", "M", "JW", "JL"):
            out[tag][int(f[1])] = np.array([float(x) for x in f[2:]])
        elif tag == "NAME":
            out["NAME"][int(f[1])] = f[2]
        elif tag in ("TAU", "ACC", "QDD", "G", "NLE", "TAUEXT", "QDDEXT"):
            out[tag] = np.array([float(x) for x in f[1:]])
        elif tag == "SHAPE":
            out["shape"] = tuple(int(x) for x in f[1:])
        elif tag == "EEF":
            out["eef"] = int(f[1])
        elif tag == "REAL_BYTES":
            out["real_bytes"] = int(f[1])
    return out


def make_config(rng, nj, floating, n_nodes=0):
    """Velocity-space vectors are packed to nv, base first, matching the C API."""
    nv = nj + (6 if floating else 0)
    cfg = {}
    if floating:
        cfg["p"] = rng.uniform(-0.4, 0.4, 3)
        quat = rng.normal(size=4)
        quat /= np.linalg.norm(quat)
        cfg["quat_wxyz"] = quat                      # w x y z
    cfg["q"] = rng.uniform(-1.2, 1.2, nj)
    cfg["qd"] = rng.uniform(-1.5, 1.5, nv)
    cfg["qdd"] = rng.uniform(-2.0, 2.0, nv)
    cfg["tau"] = rng.uniform(-3.0, 3.0, nv)
    # Reflected rotor inertia. A floating base has no rotors, so its six stay
    # zero; the joints get something big enough to matter next to a link.
    cfg["armature"] = rng.uniform(0.01, 0.3, nv)
    if floating:
        cfg["armature"][:6] = 0.0
    # A spatial force on every link, fixed ones included: those are folded into
    # the moving link above them, and carrying them there is the part of f_ext
    # most likely to be wrong.
    cfg["f_ext"] = rng.uniform(-4.0, 4.0, (n_nodes, 6))
    return cfg


def payload_for(cfg, floating):
    parts = []
    if floating:
        parts += [*cfg["p"], *cfg["quat_wxyz"]]
    parts += [*cfg["q"], *cfg["qd"], *cfg["qdd"], *cfg["tau"], *cfg["armature"]]
    parts += [float(x) for x in cfg["f_ext"].reshape(-1)]
    return " ".join(repr(float(x)) for x in parts) + "\n"


def frame_map(urdf_path, model):
    """
    C link name -> pinocchio frame name.

    urdf2c truncates link names to the fifteen characters rd_link_t holds, so a
    humanoid's `right_shoulder_pitch_link` reaches the C side shortened, and
    where two links collide at fifteen characters it appends an ordinal. The
    mapping is recovered by replaying that same shortening over the same URDF
    with urdf2c's own code, which makes it exact rather than a guess.
    """
    links = urdf2c.load_urdf(urdf_path)
    order = urdf2c.topo_sort(links)
    used, out = set(), {}
    for full in order:
        short = urdf2c.cname(full, used)
        if model.existFrame(full):
            out[short] = full
    return out


def pin_quantities(model, data, cfg, floating, link_names, eef_name, resolved):
    nj = len(cfg["q"])
    if floating:
        w, x, y, z = cfg["quat_wxyz"]
        q = np.concatenate([cfg["p"], [x, y, z, w], cfg["q"]])   # scalar-last
    else:
        q = cfg["q"].copy()
    v, a, tau_in = cfg["qd"].copy(), cfg["qdd"].copy(), cfg["tau"].copy()
    model.armature[:] = cfg["armature"]

    pin.forwardKinematics(model, data, q, v, a)
    pin.updateFramePlacements(model, data)

    T = {}
    V = {}
    for name in link_names:
        if name not in resolved:
            continue
        fid = model.getFrameId(resolved[name])
        T[name] = data.oMf[fid].homogeneous
        V[name] = pin.getFrameVelocity(model, data, fid, pin.ReferenceFrame.WORLD).vector

    # f_ext, ours per link in the link frame, theirs per joint in the joint
    # frame. A frame's placement carries one to the other, and several links can
    # land on the same joint, so they accumulate.
    fext = pin.StdVec_Force()
    for _ in range(model.njoints):
        fext.append(pin.Force.Zero())
    for name, row in zip(link_names, cfg["f_ext"]):
        if name not in resolved:
            continue
        fid = model.getFrameId(resolved[name])
        fr = model.frames[fid]
        if fr.parentJoint == 0:
            continue                       # anchored to the world
        fext[fr.parentJoint] += fr.placement.act(pin.Force(row[:3], row[3:]))

    tau = pin.rnea(model, data, q, v, a).copy()
    M = pin.crba(model, data, q)
    M = np.triu(M) + np.triu(M, 1).T          # Pinocchio fills the upper triangle

    fid = model.getFrameId(resolved.get(eef_name, eef_name))
    JW = pin.computeFrameJacobian(model, data, q, fid, pin.ReferenceFrame.WORLD)
    JL = pin.computeFrameJacobian(model, data, q, fid, pin.ReferenceFrame.LOCAL)

    acc = pin.getFrameAcceleration(model, data, fid, pin.ReferenceFrame.WORLD).vector

    qdd_fd = pin.aba(model, data, q, v, tau_in).copy()
    g = pin.computeGeneralizedGravity(model, data, q)
    nle = pin.nonLinearEffects(model, data, q, v)

    # Last, and copied: rnea and aba hand back references into `data`, so these
    # would otherwise alias tau and qdd_fd, and aba overwrites the data.a that
    # `acc` is read from above.
    tau_ext = pin.rnea(model, data, q, v, a, fext).copy()
    qdd_ext = pin.aba(model, data, q, v, tau_in, fext).copy()

    return dict(T=T, V=V, tau=tau, M=M, JW=JW, JL=JL, acc=acc,
                tau_ext=tau_ext, qdd_ext=qdd_ext,
                qdd=qdd_fd, g=g, nle=nle, nv=model.nv)


def err(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    denom = max(1.0, np.abs(a).max(), np.abs(b).max())
    return float(np.abs(a - b).max() / denom)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf-root", required=True)
    ap.add_argument("--double", action="store_true", help="build the C side in float64")
    ap.add_argument("-n", "--samples", type=int, default=20)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--tol", type=float, default=None)
    ap.add_argument("--cflag", action="append", default=[],
                    help="extra -D for the C build, e.g. --cflag=-DRD_FAST_TRIG=1")
    args = ap.parse_args()

    binary = build(args.double, args.cflag)
    tol = args.tol if args.tol is not None else (1e-10 if args.double else 3e-5)
    rng = np.random.default_rng(args.seed)

    print(f"C side: {'float64' if args.double else 'float32'}   "
          f"tolerance: {tol:g}   samples: {args.samples}\n")

    grand_fail = 0
    for robot, (rel, floating) in ROBOTS.items():
        urdf = os.path.join(args.urdf_root, rel)
        if not os.path.exists(urdf):
            print(f"{robot}: URDF not found, skipped")
            continue

        root = pin.JointModelFreeFlyer() if floating else None
        try:
            model = (pin.buildModelFromUrdf(urdf, root) if root
                     else pin.buildModelFromUrdf(urdf))
        except ValueError as exc:
            # urdfdom is stricter than bard's parser -- simple_arm.urdf, for one,
            # declares revolute joints with no <limit>, which it rejects outright.
            print(f"{robot}: pinocchio cannot parse this URDF, skipped\n    {exc}\n")
            continue
        data = model.createData()

        # One throwaway dump, to learn the C model's shape and link names.
        # Pinocchio's nv already tells us the joint count either way.
        cfg0 = make_config(rng, model.nv - (6 if floating else 0), floating,
                           len(model.frames))
        d0 = run_dump(binary, robot, payload_for(cfg0, floating))
        n, nj, nv, fb = d0["shape"]
        link_names = [d0["NAME"][i] for i in range(n)]
        eef_name = d0["NAME"][d0["eef"]]
        resolved = frame_map(urdf, model)

        if nv != model.nv:
            print(f"{robot}: nv mismatch  C={nv}  pinocchio={model.nv}")
            grand_fail += 1
            continue

        worst = {k: 0.0 for k in ("T", "V", "tau", "qdd", "g", "nle",
                                  "M", "JW", "JL", "acc", "tauext", "qddext")}
        missing_frames = 0

        for s in range(args.samples):
            cfg = make_config(rng, nj, floating, n)
            d = run_dump(binary, robot, payload_for(cfg, floating))
            ref = pin_quantities(model, data, cfg, floating, link_names, eef_name,
                                 resolved)

            for i, name in enumerate(link_names):
                if name not in ref["T"]:
                    missing_frames += 1
                    continue
                # library stores 4x4 column-major
                Tc = d["T"][i].reshape(4, 4).T
                worst["T"] = max(worst["T"], err(Tc, ref["T"][name]))
                worst["V"] = max(worst["V"], err(d["V"][i], ref["V"][name]))

            worst["tau"] = max(worst["tau"], err(d["TAU"], ref["tau"]))
            worst["qdd"] = max(worst["qdd"], err(d["QDD"], ref["qdd"]))
            worst["g"]   = max(worst["g"],   err(d["G"],   ref["g"]))
            worst["nle"] = max(worst["nle"], err(d["NLE"], ref["nle"]))
            Mc = np.array([d["M"][r] for r in range(nv)])
            worst["M"] = max(worst["M"], err(Mc, ref["M"]))
            JWc = np.array([d["JW"][r] for r in range(6)])
            worst["JW"] = max(worst["JW"], err(JWc, ref["JW"]))
            JLc = np.array([d["JL"][r] for r in range(6)])
            worst["JL"] = max(worst["JL"], err(JLc, ref["JL"]))
            worst["acc"] = max(worst["acc"], err(d["ACC"], ref["acc"]))
            if "TAUEXT" in d:
                worst["tauext"] = max(worst["tauext"], err(d["TAUEXT"], ref["tau_ext"]))
            if "QDDEXT" in d:
                worst["qddext"] = max(worst["qddext"], err(d["QDDEXT"], ref["qdd_ext"]))

        print(f"{robot}  (C: {n} links / {nj} joints / nv {nv}"
              f"{', floating' if fb else ''}   "
              f"pinocchio: {model.njoints - 1} joints / nv {model.nv})")
        for k in ("T", "V", "tau", "qdd", "g", "nle", "M", "JW", "JL", "acc",
                  "tauext", "qddext"):
            status = "ok  " if worst[k] <= tol else "FAIL"
            if worst[k] > tol:
                grand_fail += 1
            print(f"    [{status}] {k:4s} max rel err {worst[k]:.3e}")
        if missing_frames:
            print(f"    ({missing_frames} frame lookups skipped: name not in pinocchio model)")
        print()

    print("ALL CHECKS PASS" if grand_fail == 0 else f"{grand_fail} CHECK(S) FAILED")
    return 1 if grand_fail else 0


if __name__ == "__main__":
    sys.exit(main())
