#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
amalgamate.py -- fold the library into one header for distribution.

The multi-file tree under RobotDynamics/ is what gets maintained and read. What
gets shipped is one file you drop into a project, in the style stb uses:

    /* exactly one .c file in your project */
    #define RD_IMPLEMENTATION
    #include "robot_dynamics.h"

    /* everywhere else */
    #include "robot_dynamics.h"

    python3 tools/amalgamate.py -o dist/robot_dynamics.h

The transformation is deliberately small, so that the shipped file stays
readable and stays honest about where each part came from: local #includes are
dropped because the file they name is already above, per-file SPDX lines are
folded into one at the top, and a banner marks each section with its source
path. Nothing else is rewritten.

Section order is the dependency order of the tree. The three .c files go inside
RD_IMPLEMENTATION, which sits outside the header guard so that a translation
unit including the header twice -- once plain, once to instantiate -- still
gets the implementation.
"""

import argparse
import difflib
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "RobotDynamics"

# Dependency order. spine_model.h is an example model rather than part of the
# library, so it stays out of the shipped file.
PUBLIC = ["rd_config.h", "rd_model.h", "rd_math.h",
          "rd_chain.h", "rd_state.h", "rd_algorithms.h"]
IMPL = ["rd_chain.c", "rd_state.c", "rd_algorithms.c"]

LOCAL_INCLUDE = re.compile(r'^\s*#\s*include\s+"(rd_[a-z_]+\.[ch])"\s*$')
SPDX = re.compile(r'^/\* SPDX-License-Identifier: .* \*/\s*$')
GUARD_OPEN = re.compile(r'^#\s*(ifndef|define)\s+ROBOT_DYNAMICS_H\s*$')
GUARD_CLOSE = re.compile(r'^#\s*endif\s*/\* ROBOT_DYNAMICS_H \*/\s*$')


def git(*args, default=""):
    try:
        return subprocess.run(("git", *args), cwd=ROOT, capture_output=True,
                              text=True, check=True).stdout.strip()
    except Exception:
        return default


def version():
    text = (SRC / "robot_dynamics.h").read_text()
    m = re.search(r"@version\s+([0-9][^\s*]*)", text)
    return m.group(1) if m else "unknown"


def banner_comment():
    """The doc comment at the top of robot_dynamics.h, up to its guard."""
    lines = (SRC / "robot_dynamics.h").read_text().splitlines()
    out = []
    for line in lines:
        if GUARD_OPEN.match(line):
            break
        if SPDX.match(line):
            continue
        out.append(line)
    while out and not out[-1].strip():
        out.pop()
    # The caller continues inside this comment, so hand it back still open.
    if out and out[-1].strip() == "*/":
        out.pop()
    return out


def body(name):
    """One source file, with the includes it no longer needs taken out."""
    out = []
    for line in (SRC / name).read_text().splitlines():
        if LOCAL_INCLUDE.match(line) or SPDX.match(line):
            continue
        out.append(line)
    # A local include usually sits in a block of its own; collapse what it left.
    text = "\n".join(out)
    return re.sub(r"\n{3,}", "\n\n", text).strip("\n")


def section(name):
    rule = "/* " + "=" * 74 + " */"
    return [rule, f"/* RobotDynamics/{name:<58} */", rule, "", body(name), ""]


def build():
    ver, commit = version(), git("rev-parse", "--short", "HEAD", default="unknown")
    dirty = " (working tree modified)" if git("status", "--porcelain") else ""

    out = ["/* SPDX-License-Identifier: Apache-2.0 */"]
    out += banner_comment()
    out += [
        " *",
        " * ---------------------------------------------------------------------",
        f" * Single-header build, version {ver}, generated from {commit}{dirty}.",
        " * Edits belong in the RobotDynamics/ tree; regenerate with",
        " * tools/amalgamate.py.",
        " *",
        " * Drop this file into your project. In exactly one .c file:",
        " *",
        " *     #define RD_IMPLEMENTATION",
        ' *     #include "robot_dynamics.h"',
        " *",
        " * and include it plainly everywhere else. Without that one definition",
        " * the algorithms will not link.",
        " */",
        "",
        "#ifndef ROBOT_DYNAMICS_H",
        "#define ROBOT_DYNAMICS_H",
        "",
    ]
    for name in PUBLIC:
        out += section(name)
    out += ["#endif /* ROBOT_DYNAMICS_H */", ""]

    out += [
        "#ifdef RD_IMPLEMENTATION",
        "#ifndef RD_IMPLEMENTATION_DONE",
        "#define RD_IMPLEMENTATION_DONE",
        "",
    ]
    for name in IMPL:
        out += section(name)
    out += ["#endif /* RD_IMPLEMENTATION_DONE */",
            "#endif /* RD_IMPLEMENTATION */", ""]

    return "\n".join(out)


# ----------------------------------------------------------------------------
# Verification
#
# The claim this file makes is that the shipped header is the same library, so
# the check is machine code rather than behaviour alone: build both ways for
# Cortex-M4 and compare every public function instruction for instruction.
# Absolute addresses differ because one object holds what three used to, so
# they are normalised away; nothing else is.
# ----------------------------------------------------------------------------

ARM = os.environ.get(
    "RD_ARM_GCC",
    "/Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi-gcc")
ARM_FLAGS = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard",
             "-O3", "-std=gnu11", "-DRD_USE_SINGLE_PRECISION=1",
             "-DRD_MAX_LINKS=40", "-DRD_MAX_JOINTS=24"]

INSN = re.compile(r"^\s*[0-9a-f]+:\t")     # the instruction's own address
ADDR = re.compile(r"\b[0-9a-f]{1,8} <")   # and any address it names


def disassemble(objdump, *objects):
    """{function: [instructions]} for every object given, addresses normalised."""
    out, cur = {}, None
    for obj in objects:
        text = subprocess.run([objdump, "-d", "--no-show-raw-insn", str(obj)],
                              capture_output=True, text=True, check=True).stdout
        cur = None
        for line in text.splitlines():
            if line.startswith("Disassembly") or "file format" in line:
                cur = None
                continue
            head = re.match(r"^[0-9a-f]+ <([^>]+)>:", line)
            if head:
                cur = head.group(1)
                out[cur] = []
            elif cur and line.strip():
                out[cur].append(ADDR.sub("<", INSN.sub("", line.rstrip())))
    return out


def run(cmd, **kw):
    return subprocess.run([str(c) for c in cmd], check=True, **kw)


def verify(header_path):
    """Compile the single header every way that matters and diff the code."""
    cc = os.environ.get("CC", "cc")
    ok = True
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        shutil.copy(header_path, tmp / "robot_dynamics.h")
        shutil.copy(SRC / "spine_model.h", tmp / "spine_model.h")
        (tmp / "impl.c").write_text(
            '#define RD_IMPLEMENTATION\n#include "robot_dynamics.h"\n')
        # A second consumer proves the header carries no duplicate definitions.
        (tmp / "user.c").write_text(
            '#include "robot_dynamics.h"\n'
            "rd_int_t rd_probe_nv_(const rd_chain_t* c) { return rd_chain_get_nv(c); }\n")

        warn = ["-std=c99", "-pedantic", "-Wall", "-Wextra"]
        for label, extra in (("float32", []),
                             ("float64", ["-DRD_USE_SINGLE_PRECISION=0", "-DRD_FAST_TRIG=0"])):
            exe = tmp / f"t_{label}"
            run([cc, "-O2", *warn, *extra, "-I", tmp, ROOT / "test_main.c",
                 tmp / "impl.c", tmp / "user.c", "-lm", "-o", exe])
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            passed = result.returncode == 0 and "PASSED" in result.stdout
            print(f"  [{'ok  ' if passed else 'FAIL'}] {label}: three translation units, "
                  f"clean at -Wall -Wextra -pedantic, smoke test passes")
            ok &= passed

        cxx = os.environ.get("CXX", "c++")
        if shutil.which(cxx):
            try:
                run([cxx, "-O1", "-std=c++17", "-Wall", "-I", tmp, "-x", "c++",
                     tmp / "user.c", "-c", "-o", tmp / "cxx.o"])
                print("  [ok  ] C++: includes cleanly, extern \"C\" nesting is fine")
            except subprocess.CalledProcessError:
                print("  [FAIL] C++: does not include cleanly")
                ok = False

        if not shutil.which(ARM):
            print("  [skip] Cortex-M4 code comparison: no arm-none-eabi-gcc "
                  "(set RD_ARM_GCC)")
            return ok

        objdump = ARM.replace("-gcc", "-objdump")
        run([ARM, *ARM_FLAGS, "-I", tmp, "-c", tmp / "impl.c", "-o", tmp / "single.o"])
        multi = []
        for name in IMPL:
            obj = tmp / (name[:-2] + ".o")
            run([ARM, *ARM_FLAGS, "-I", SRC, "-c", SRC / name, "-o", obj])
            multi.append(obj)

        one, many = disassemble(objdump, tmp / "single.o"), disassemble(objdump, *multi)
        shared = sorted(set(one) & set(many))
        differing = [f for f in shared if one[f] != many[f]]
        missing = sorted(set(many) - set(one))
        for f in differing[:5]:
            print(f"         {f} differs:")
            for line in list(difflib.unified_diff(many[f], one[f], lineterm="", n=0))[:8]:
                if not line.startswith(("---", "+++", "@@")):
                    print("           ", line)
        if differing or missing:
            print(f"  [FAIL] Cortex-M4: {len(differing)} of {len(shared)} functions "
                  f"differ, {len(missing)} missing")
            ok = False
        else:
            print(f"  [ok  ] Cortex-M4: all {len(shared)} functions identical "
                  "instruction for instruction to the multi-file build")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("-o", "--output", default=str(ROOT / "dist" / "robot_dynamics.h"),
                    help="where to write the single header")
    ap.add_argument("--verify", action="store_true",
                    help="build the result every way that matters and compare its "
                         "Cortex-M4 code against the multi-file build")
    args = ap.parse_args()

    text = build()
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)

    print(f"{path}  {len(text.encode()):,} bytes, {text.count(chr(10)) + 1:,} lines")

    if args.verify:
        return 0 if verify(path) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
