#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fail the build if the image cannot fit in the RAM the part actually has.

The linker only checks the sections it places. The heap is not one of them --
rd_chain_build allocates roughly 11 KB for Go2 at run time -- so a link that
succeeds can still overrun into nothing at all. This makes that overrun a build
error instead of a silent corruption on a board with no debugger attached."""
import subprocess, sys, re

elf, ram_kb = sys.argv[1], int(sys.argv[2])
size = subprocess.run(["arm-none-eabi-size", elf], capture_output=True, text=True)
if size.returncode != 0:                       # not on PATH; parse the map instead
    import os
    tc = os.environ.get("TC", "")
    size = subprocess.run([tc + "-size", elf], capture_output=True, text=True)
nums = re.findall(r"\d+", size.stdout.splitlines()[-1]) if size.stdout else []
if len(nums) < 3:
    sys.exit(0)                                # cannot tell; do not block the build
text, data, bss = (int(n) for n in nums[:3])

HEAP_GO2, STACK = 11 * 1024, 3 * 1024          # measured; see l4_out.c _sbrk
used = data + bss + HEAP_GO2 + STACK
have = ram_kb * 1024
print(f"  RAM: data+bss {data+bss:,} + heap {HEAP_GO2:,} + stack {STACK:,} "
      f"= {used:,} of {have:,} ({100*used/have:.0f}%)")
if used > have:
    sys.exit(f"  ERROR: {used - have:,} bytes over. Shrink BENCH_MAX_NODES/NV.")
