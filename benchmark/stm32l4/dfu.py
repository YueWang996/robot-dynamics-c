#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
dfu.py -- drive the STM32L413 benchmark over USB DFU, with no SWD probe.

The board has no debugger, so the ROM bootloader is the entire interface:
download the image, start it with Go, let it reset itself when it is done, and
read the results back out of the flash region it wrote them to.

    python3 dfu.py run rd_bench.bin 0x08010000
    python3 dfu.py read

Nothing here writes to 0x08000000. The first flash word staying erased is what
makes the part fall back into the bootloader at every reset, so a firmware that
hangs or faults costs a power cycle and nothing more.
"""

import re
import subprocess
import sys
import time


CLI = ("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer"
       "/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI")

OUT_ADDR, OUT_SIZE = 0x08030000, 16 * 2048     # must match l4_flash.h
ANSI = re.compile(r"\x1b\[[0-9;]*m")

sys.stdout.reconfigure(line_buffering=True)   # this runs under make; do not buffer


def cli(*args, check=True, timeout=120):
    try:
        p = subprocess.run([CLI, "-c", "port=usb1", *args],
                           capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        if check:
            raise SystemExit(f"STM32_Programmer_CLI timed out: {' '.join(args)}")
        return ""
    out = ANSI.sub("", p.stdout + p.stderr)
    if check and ("Error" in out or p.returncode != 0):
        raise SystemExit(out)
    return out


def decode(raw: bytes) -> str:
    # 0xFF is erased flash and also the padding on a part-filled doubleword,
    # so it can appear mid-stream. The payload is ASCII, so dropping every
    # 0xFF is safe and reassembles a stream that was flushed line by line.
    text = bytes(b for b in raw if b != 0xFF).decode("ascii", "replace")
    end = text.find("\nEND\n")
    return text[: end + 5] if end >= 0 else text


def read_results(path="out.bin", check=True):
    out = cli("-u", hex(OUT_ADDR), str(OUT_SIZE), path, check=check, timeout=60)
    if "Error" in out:
        return ""
    return decode(open(path, "rb").read())


def wait_for_finish(timeout=600, poll=3.0):
    """While the program runs, nothing services USB, so a read either fails or
    times out; both are just 'not done yet'. The region was erased before Go,
    so the END marker can only come from this run."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(poll)
        text = read_results(check=False)
        # Three ways a run can legitimately be over: the benchmark's last
        # line, the probe's, or a fault -- the probe ends by faulting on
        # purpose, and a fault elsewhere is still an end of run.
        if "flash_err=" in text or "\nEND\n" in text or "!FAULT" in text:
            return time.time() - t0, text
        if text.strip():
            print(f"   ... {len(text.strip())} bytes so far "
                  f"({time.time()-t0:.0f}s)")
    raise SystemExit(f"no END marker within {timeout}s; run 'make read' to see "
                     f"how far it got")


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    cmd = sys.argv[1]

    if cmd == "read":
        print(read_results())
        return

    if cmd == "run":
        binfile, addr = sys.argv[2], sys.argv[3]
        first = (OUT_ADDR - 0x08000000) // 2048
        print(f"-> erasing result pages {first}..{first + OUT_SIZE // 2048 - 1}")
        cli("-e", f"[{first} {first + OUT_SIZE // 2048 - 1}]")
        print(f"-> downloading {binfile} to {addr}")
        cli("-w", binfile, addr, "-v")
        print(f"-> go {addr}; the board resets itself when it finishes")
        cli("-g", addr, check=False, timeout=60)
        took, text = wait_for_finish()
        print(f"-> finished in {took:.0f}s\n")
        print(text)
        return

    raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
