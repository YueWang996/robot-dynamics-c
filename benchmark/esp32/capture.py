#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
capture.py -- read the benchmark CSV off an ESP32 over USB CDC.

The port disappears while esptool has the chip and re-enumerates when the new
firmware boots, so this waits for it to come back rather than assuming it is
there. The sketch holds off until the host opens the port, so nothing is lost
to the race.

    python3 capture.py /dev/cu.usbmodem101 results.csv
"""

import os
import subprocess
import sys
import time

BEGIN, END = "BEGIN_CSV", "END_CSV"


def wait_for_port(port, timeout=30):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if os.path.exists(port):
            time.sleep(0.5)          # let the CDC interface settle
            return
        time.sleep(0.2)
    raise SystemExit(f"{port} never appeared")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem101"
    out = sys.argv[2] if len(sys.argv) > 2 else "results.csv"

    wait_for_port(port)
    # Raw mode, no hangup on close: reopening must not reset the board.
    subprocess.run(["stty", "-f", port, "raw", "115200", "-hupcl"], check=True)

    text, started, t0 = [], False, time.time()
    with open(port, "rb", buffering=0) as fh:
        while time.time() - t0 < 300:
            chunk = fh.read(256)
            if not chunk:
                time.sleep(0.05)
                continue
            s = chunk.decode("utf-8", "replace")
            sys.stdout.write(s)
            sys.stdout.flush()
            text.append(s)
            joined = "".join(text)
            if BEGIN in joined:
                started = True
            if started and END in joined:
                break

    joined = "".join(text)
    if BEGIN not in joined or END not in joined:
        raise SystemExit("\nnever saw a complete CSV block")
    body = joined[joined.index(BEGIN) + len(BEGIN):joined.index(END)]
    open(out, "w").write(body.strip().replace("\r", "") + "\n")
    print(f"\n-> wrote {out}")


if __name__ == "__main__":
    main()
