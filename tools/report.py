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
    "aba",
    "crba",
    "gravity_comp",
    "spatial_accel",
    "spatial_velocity",
    "_heap_probe",
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
        shape[r["robot"]] = (int(r["n_links"]), int(r["n_joints"]), int(r["nv"]))
        if not r["ns_per_call"]:
            out.setdefault(key, None)   # reported unsupported by the harness
            continue
        ns = float(r["ns_per_call"])
        if out.get(key) is None or ns < out[key]:
            out[key] = ns
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
                cells.append("n/a" if (arch, r, algo) in data else "--")
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


def table_fd_vs_id(data, shape):
    """Forward dynamics against inverse dynamics, and against CRBA."""
    lines = []
    lines.append("| Robot | dof | RNEA | ABA | ABA/RNEA | CRBA | ABA/CRBA |")
    lines.append("|---|---|---|---|---|---|---|")
    for r in ROBOT_ORDER:
        rn = data.get(("arm", r, "rnea"))
        ab = data.get(("arm", r, "aba"))
        cr = data.get(("arm", r, "crba"))
        if None in (rn, ab, cr):
            continue
        lines.append(f"| `{r}` | {shape[r][2]} | {fmt_us(rn)} us | {fmt_us(ab)} us "
                     f"| {ab/rn:.2f}x | {fmt_us(cr)} us | {ab/cr:.2f}x |")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    args = ap.parse_args()

    rows, meta = load(args.csv)
    data, shape = index(rows)

    # Emit a table for every architecture present, so a new board shows up
    # without touching this script.
    LABEL = {
        "arm":     "Arm Cortex-M33 @ 150 MHz (RP2350)",
        "riscv":   "RISC-V Hazard3 @ 150 MHz (RP2350)",
        "stm32g4": "Arm Cortex-M4F @ 170 MHz (STM32G474)",
        "stm32l4": "Arm Cortex-M4F @ 80 MHz (STM32L413)",
        "host":    "Host reference",
    }
    present = [a for a in ("arm", "stm32g4", "stm32l4", "riscv")
               if any(k[0] == a for k in data)]
    for arch in present:
        print(f"### {LABEL.get(arch, arch)} -- microseconds per call\n")
        print(table_per_arch(data, shape, arch))
        print()
    print("### RISC-V / Arm ratio (higher = RISC-V slower)\n")
    print(table_ratio(data, shape))
    print("\n### Control-loop budget: update_kinematics + rnea\n")
    print(table_control_budget(data, shape))
    print("\n### Forward vs inverse dynamics (Arm)\n")
    print(table_fd_vs_id(data, shape))

    if "host" in {k[0] for k in data}:
        print("\n### Host reference -- microseconds per call\n")
        print(table_per_arch(data, shape, "host"))


if __name__ == "__main__":
    sys.exit(main())
