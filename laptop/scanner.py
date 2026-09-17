"""UDP listener for the stereo vergence scanner.

Receives one JSON packet per bearing from the NodeMCU, fuses the two readings,
and writes one CSV per completed sweep.

    python scanner.py --out scans/

Packet format, as emitted by `sendPacket()` in the firmware:

    {"seq":0,"t":12345,"theta":-89.00,"sL":93,"sR":87,"dL":812,"dR":809}

`dL`/`dR` are millimetres, or -1 for out-of-range. Sweeps run THETA_MIN to
THETA_MAX and then restart, so a drop in theta marks a sweep boundary.

UDP is the right choice here and also the reason for the bookkeeping below:
packets are cheap to send from a device with no TCP buffers to stall on, but
they can be dropped or reordered. A lost bearing is a hole in the scan, not a
failure, so we record how many arrived rather than pretending the sweep is
complete.
"""

from __future__ import annotations

import argparse
import csv
import json
import socket
import sys
from datetime import datetime
from pathlib import Path

from geometry import fuse

LISTEN_PORT = 5005  # must match LAPTOP_PORT in the firmware
EXPECTED_PER_SWEEP = 179  # -89..+89 at 1 deg

CSV_COLUMNS = [
    "seq", "t_ms", "theta_deg", "servo_l", "servo_r",
    "dist_l_mm", "dist_r_mm", "x_m", "y_m",
    "range_m", "disagreement_m", "n_valid", "flag",
]


def parse_packet(raw: bytes) -> dict | None:
    """Decode one datagram, or None if it is malformed.

    A half-written or corrupted packet should cost one bearing, not the run.
    """
    try:
        pkt = json.loads(raw.decode("utf-8").strip())
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not all(k in pkt for k in ("seq", "theta", "sL", "sR", "dL", "dR")):
        return None
    return pkt


def to_row(pkt: dict) -> dict:
    f = fuse(
        theta_deg=float(pkt["theta"]),
        servo_l=int(pkt["sL"]),
        servo_r=int(pkt["sR"]),
        dist_l_mm=int(pkt["dL"]),
        dist_r_mm=int(pkt["dR"]),
    )
    p = f.point
    return {
        "seq": pkt["seq"],
        "t_ms": pkt.get("t", ""),
        "theta_deg": f"{f.theta_deg:.2f}",
        "servo_l": pkt["sL"],
        "servo_r": pkt["sR"],
        "dist_l_mm": pkt["dL"],
        "dist_r_mm": pkt["dR"],
        "x_m": f"{p.x:.4f}" if p else "",
        "y_m": f"{p.y:.4f}" if p else "",
        "range_m": f"{p.range_m:.4f}" if p else "",
        "disagreement_m": f"{f.disagreement_m:.4f}" if f.disagreement_m is not None else "",
        "n_valid": f.n_valid,
        "flag": f.flag,
    }


def write_sweep(rows: list[dict], out_dir: Path, index: int) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = out_dir / f"sweep_{index:03d}_{stamp}.csv"
    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=CSV_COLUMNS)
        w.writeheader()
        w.writerows(rows)
    return path


def summarise(rows: list[dict]) -> str:
    total = len(rows)
    ok = sum(r["flag"] == "ok" for r in rows)
    edges = sum(r["flag"] == "edge" for r in rows)
    single = sum(r["flag"] == "single" for r in rows)
    blind = sum(r["flag"] == "no_return" for r in rows)
    missing = max(0, EXPECTED_PER_SWEEP - total)
    return (f"{total} bearings ({missing} dropped) — "
            f"{ok} fused, {edges} edges, {single} one-eyed, {blind} no return")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=Path("scans"),
                    help="directory for per-sweep CSVs (default: scans/)")
    ap.add_argument("--port", type=int, default=LISTEN_PORT)
    ap.add_argument("--sweeps", type=int, default=0,
                    help="stop after N sweeps (default: run until Ctrl-C)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    print(f"listening on udp/{args.port} — Ctrl-C to stop", file=sys.stderr)

    rows: list[dict] = []
    last_theta = None
    sweep_index = 0

    try:
        while True:
            raw, _ = sock.recvfrom(512)
            pkt = parse_packet(raw)
            if pkt is None:
                continue

            theta = float(pkt["theta"])
            # Theta climbs across a sweep and resets at the start of the next.
            if last_theta is not None and theta < last_theta and rows:
                path = write_sweep(rows, args.out, sweep_index)
                print(f"sweep {sweep_index}: {summarise(rows)} -> {path}",
                      file=sys.stderr)
                sweep_index += 1
                rows = []
                if args.sweeps and sweep_index >= args.sweeps:
                    return 0
            last_theta = theta
            rows.append(to_row(pkt))

    except KeyboardInterrupt:
        if rows:
            path = write_sweep(rows, args.out, sweep_index)
            print(f"\npartial sweep {sweep_index}: {summarise(rows)} -> {path}",
                  file=sys.stderr)
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
