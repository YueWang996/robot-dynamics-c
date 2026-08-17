#!/usr/bin/env python3
"""
capture.py -- flash a benchmark UF2 to an RP2350 and scrape its CSV report.

Usage:
    python3 tools/capture.py <firmware.uf2> <output.csv> [--timeout SECONDS]

Waits for the board to re-enumerate as a USB CDC device after flashing, reads
until the END_CSV marker, and writes everything between BEGIN_CSV and END_CSV.
Needs no third-party modules.
"""

import argparse
import glob
import os
import subprocess
import sys
import termios
import time


def find_serial_port(exclude, deadline):
    """Wait for a new /dev/cu.usbmodem* to appear."""
    while time.time() < deadline:
        ports = set(glob.glob("/dev/cu.usbmodem*")) - exclude
        if ports:
            # Give the CDC endpoint a moment to settle before opening.
            time.sleep(1.0)
            return sorted(ports)[0]
        time.sleep(0.25)
    return None


def open_raw(port):
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                      # iflag
    attrs[1] = 0                      # oflag
    attrs[3] = 0                      # lflag: no canon, no echo
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def reboot_to_bootsel():
    """Best-effort: put a running board back into BOOTSEL."""
    subprocess.run(["picotool", "reboot", "-f", "-u"],
                   capture_output=True, text=True)
    time.sleep(2.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("uf2")
    ap.add_argument("output")
    ap.add_argument("--timeout", type=float, default=420.0)
    args = ap.parse_args()

    before = set(glob.glob("/dev/cu.usbmodem*"))

    info = subprocess.run(["picotool", "info"], capture_output=True, text=True)
    if "not in BOOTSEL mode" in (info.stdout + info.stderr) or info.returncode != 0:
        print("[capture] board is running; rebooting into BOOTSEL", flush=True)
        reboot_to_bootsel()
        before = set(glob.glob("/dev/cu.usbmodem*"))

    print(f"[capture] flashing {args.uf2}", flush=True)
    load = subprocess.run(["picotool", "load", "-x", args.uf2],
                          capture_output=True, text=True)
    print(load.stdout.strip() or load.stderr.strip(), flush=True)
    if load.returncode != 0:
        return 1

    deadline = time.time() + args.timeout
    port = find_serial_port(before, deadline)
    if port is None:
        print("[capture] no serial port appeared", file=sys.stderr)
        return 1
    print(f"[capture] reading {port}", flush=True)

    fd = open_raw(port)
    buf = b""
    try:
        while time.time() < deadline:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                chunk = b""
            except OSError as exc:
                print(f"[capture] read error: {exc}", file=sys.stderr)
                break
            if chunk:
                buf += chunk
                if b"END_CSV" in buf and b"BEGIN_CSV" in buf:
                    break
            else:
                time.sleep(0.05)
    finally:
        os.close(fd)

    text = buf.decode("utf-8", errors="replace")
    if "BEGIN_CSV" not in text or "END_CSV" not in text:
        print("[capture] incomplete report; got:\n" + text[-2000:], file=sys.stderr)
        return 1

    start = text.index("BEGIN_CSV") + len("BEGIN_CSV")
    end = text.index("END_CSV")
    body = text[start:end].strip().replace("\r", "")

    with open(args.output, "w") as fh:
        fh.write(body + "\n")
    print(f"[capture] wrote {args.output} ({len(body.splitlines())} lines)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
