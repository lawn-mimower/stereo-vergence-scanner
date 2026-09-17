"""Render a sweep CSV as a top-down map.

    python plot_scan.py scans/sweep_000_*.csv
    python plot_scan.py scans/sweep_000_*.csv --save map.png

Confident points are drawn solid, one-eyed points hollow, and flagged edges in
a contrasting colour — the flags are the interesting part of the scan, so they
are drawn on top rather than filtered out.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt

from geometry import BASELINE_M

STYLES = {
    "ok":     dict(c="#2563eb", s=14, label="fused",  zorder=2),
    "single": dict(c="#94a3b8", s=12, label="one eye", zorder=1,
                   facecolors="none", edgecolors="#94a3b8"),
    "edge":   dict(c="#dc2626", s=34, label="edge",   zorder=3, marker="x"),
}


def load(path: Path) -> list[dict]:
    with path.open() as fh:
        return [r for r in csv.DictReader(fh)]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--save", type=Path, help="write a PNG instead of showing a window")
    args = ap.parse_args()

    rows = load(args.csv)
    fig, ax = plt.subplots(figsize=(7, 7))

    for flag, style in STYLES.items():
        pts = [(float(r["x_m"]), float(r["y_m"]))
               for r in rows if r["flag"] == flag and r["x_m"]]
        if not pts:
            continue
        kw = dict(style)
        if "facecolors" in kw:
            kw.pop("c", None)
        ax.scatter([p[0] for p in pts], [p[1] for p in pts], **kw)

    # The rig itself, for scale.
    ax.plot([-BASELINE_M / 2, BASELINE_M / 2], [0, 0], "k-", lw=3, zorder=4)
    ax.scatter([0], [0], c="k", s=20, marker="^", zorder=5, label="rig")

    blind = sum(r["flag"] == "no_return" for r in rows)
    ax.set_title(f"{args.csv.name}\n{len(rows)} bearings, {blind} without a return")
    ax.set_xlabel("x (m, + is right)")
    ax.set_ylabel("y (m, forward)")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    if args.save:
        fig.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"wrote {args.save}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
