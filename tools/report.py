#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
report.py -- turn captured benchmark CSVs into the Markdown tables used by
docs/PROFILING.md.

Usage:
    python3 tools/report.py benchmark/results/*.csv
"""

import argparse
import csv
import io
import sys
from collections import OrderedDict

ALGO_ORDER = [
    "update_kinematics",
    "fk_frame",
    "jacobian_world",
    "jacobian_local",
    "rnea",
    "gravity_comp",
    "crba",
    "spatial_accel",
    "spatial_velocity",
    "_heap_rnea",
    "_heap_crba",
]

ROBOT_ORDER = ["simple_arm", "spine", "xarm7", "go2"]


def load(paths):
    rows, meta = [], {}
    for path in paths:
        with open(path) as fh:
            text = fh.read()
        header = {}
        body = []
        for line in text.splitlines():
            if line.startswith("#"):
                if "=" in line:
                    k, v = line[1:].strip().split("=", 1)
                    header[k.strip()] = v.strip()
            elif line.strip():
                body.append(line)
        reader = csv.DictReader(io.StringIO("\n".join(body)))
        for row in reader:
            row["_src"] = path
            rows.append(row)
        arch = header.get("arch", path)
        meta[arch.split()[0].lower()] = header
    return rows, meta


def index(rows):
    """(arch, robot, algorithm) -> ns_per_call, keeping the fastest duplicate."""
    out = {}
    shape = {}
    for r in rows:
        key = (r["arch"], r["robot"], r["algorithm"])
        ns = float(r["ns_per_call"])
        if key not in out or ns < out[key]:
            out[key] = ns
        shape[r["robot"]] = (int(r["n_links"]), int(r["n_joints"]), int(r["nv"]))
    return out, shape


def fmt_us(ns):
    return f"{ns / 1000.0:.2f}"


def cycles(ns, hz=150_000_000):
    return f"{ns * 1e-9 * hz:,.0f}"


def table_per_arch(data, shape, arch, hz=150_000_000):
    robots = [r for r in ROBOT_ORDER if any(k[1] == r and k[0] == arch for k in data)]
    lines = []
    head = "| Algorithm | " + " | ".join(
        f"{r}<br><sub>{shape[r][0]}L / {shape[r][2]}dof</sub>" for r in robots
    ) + " |"
    lines.append(head)
    lines.append("|" + "---|" * (len(robots) + 1))
    for algo in ALGO_ORDER:
        cells = []
        present = False
        for r in robots:
            ns = data.get((arch, r, algo))
            if ns is None:
                cells.append("--")
            else:
                present = True
                cells.append(f"{fmt_us(ns)}")
        if present:
            label = algo if not algo.startswith("_") else f"*{algo}*"
            lines.append(f"| `{label}` | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def table_ratio(data, shape):
    robots = [r for r in ROBOT_ORDER if ("arm", r, "rnea") in data]
    lines = []
    lines.append("| Algorithm | " + " | ".join(robots) + " |")
    lines.append("|" + "---|" * (len(robots) + 1))
    for algo in ALGO_ORDER:
        cells, present = [], False
        for r in robots:
            a = data.get(("arm", r, algo))
            v = data.get(("riscv", r, algo))
            if a and v:
                present = True
                cells.append(f"{v / a:.2f}x")
            else:
                cells.append("--")
        if present:
            label = algo if not algo.startswith("_") else f"*{algo}*"
            lines.append(f"| `{label}` | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def table_control_budget(data, shape):
    """A realistic torque-control tick: update_kinematics + RNEA."""
    lines = []
    lines.append("| Robot | dof | Arm tick | Arm max rate | RISC-V tick | RISC-V max rate |")
    lines.append("|---|---|---|---|---|---|")
    for r in ROBOT_ORDER:
        row = [r, str(shape[r][2])]
        ok = True
        for arch in ("arm", "riscv"):
            uk = data.get((arch, r, "update_kinematics"))
            rn = data.get((arch, r, "rnea"))
            if uk is None or rn is None:
                ok = False
                break
            tick_us = (uk + rn) / 1000.0
            row.append(f"{tick_us:.2f} us")
            row.append(f"{1e6 / tick_us:,.0f} Hz")
        if ok:
            lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines)


def table_heap_share(data, shape):
    lines = []
    lines.append("| Robot | Arch | RNEA total | heap part | share | CRBA total | heap part | share |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for r in ROBOT_ORDER:
        for arch in ("arm", "riscv"):
            rn = data.get((arch, r, "rnea"))
            hr = data.get((arch, r, "_heap_rnea"))
            cr = data.get((arch, r, "crba"))
            hc = data.get((arch, r, "_heap_crba"))
            if None in (rn, hr, cr, hc):
                continue
            lines.append(
                f"| {r} | {arch} | {fmt_us(rn)} us | {fmt_us(hr)} us | {hr/rn*100:.1f}% "
                f"| {fmt_us(cr)} us | {fmt_us(hc)} us | {hc/cr*100:.1f}% |"
            )
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    args = ap.parse_args()

    rows, meta = load(args.csv)
    data, shape = index(rows)

    print("### Arm (Cortex-M33) -- microseconds per call\n")
    print(table_per_arch(data, shape, "arm"))
    print("\n### RISC-V (Hazard3) -- microseconds per call\n")
    print(table_per_arch(data, shape, "riscv"))
    print("\n### RISC-V / Arm ratio (higher = RISC-V slower)\n")
    print(table_ratio(data, shape))
    print("\n### Control-loop budget: update_kinematics + rnea\n")
    print(table_control_budget(data, shape))
    print("\n### Heap traffic as a share of algorithm cost\n")
    print(table_heap_share(data, shape))

    if "host" in {k[0] for k in data}:
        print("\n### Host reference -- microseconds per call\n")
        print(table_per_arch(data, shape, "host"))


if __name__ == "__main__":
    sys.exit(main())
