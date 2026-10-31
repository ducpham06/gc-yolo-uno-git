#!/usr/bin/env python3
"""Plot closed-loop logs produced by the board (via capture_serial.py) or the host build.

usage: python tools/plot_results.py results/            # folder containing log_<scenario>.csv
"""
import sys, glob, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(fn):
    """Read a log CSV, skipping rows that were truncated or garbled on the serial link."""
    with open(fn) as f:
        header = f.readline().strip().split(",")
        rows, bad = [], 0
        for line in f:
            parts = line.strip().split(",")
            if len(parts) != len(header):
                bad += 1
                continue
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                bad += 1
    if bad:
        print(f"{os.path.basename(fn)}: skipped {bad} malformed row(s)")
    if not rows:
        raise SystemExit(f"{fn}: no valid rows")
    arr = np.array(rows)
    return {name: arr[:, i] for i, name in enumerate(header)}


def main(folder):
    files = sorted(glob.glob(os.path.join(folder, "log_*.csv")))
    if not files:
        sys.exit(f"no log_*.csv in {folder}")
    logs = {os.path.basename(f)[4:-4]: load(f) for f in files}
    n = len(logs)
    fig, ax = plt.subplots(4, n, figsize=(4.2 * n, 11), squeeze=False)
    for c, (name, d) in enumerate(logs.items()):
        t = d["t"]
        a = ax[0, c]
        a.plot(t, d["h_w1"], "r", label="link 2 vs wedge plane 1")
        a.plot(t, d["h_w2"], "m", label="link 2 vs wedge plane 2")
        a.plot(t, d["h_min"], "k:", lw=1, label="min over all pairs")
        a.axhline(0, color="gray", ls="--", lw=0.8)
        a.set_title(name); a.set_ylabel("safety metric h"); a.legend(fontsize=7)
        a = ax[1, c]
        for i in range(3):
            a.plot(t, d[f"q{i+1}"], label=f"q{i+1}")
        a.set_ylabel("joint angle [rad]"); a.legend(fontsize=7)
        a = ax[2, c]
        a.plot(t, d["d1"], "C0", lw=1, label="d1 (true)")
        a.plot(t, d["dh1"], "C0--", lw=1, label="d1 (DOB)")
        a.plot(t, d["pen_mm"] / 10, "C3", lw=1, label="penetration [cm]")
        a.set_ylabel("disturbance [Nm] / pen."); a.legend(fontsize=7)
        a = ax[3, c]
        a.plot(t, d["ctrl_us"], ".", ms=2)
        fb = d["fallback"] > 0
        a.plot(t[fb], d["ctrl_us"][fb], "rx", ms=4, label="QP fallback")
        a.set_ylabel("controller time [us]"); a.set_xlabel("t [s]"); a.legend(fontsize=7)
    fig.tight_layout()
    out = os.path.join(folder, "results.png")
    fig.savefig(out, dpi=110)
    print("wrote", out)

    # Top view of the tip path (cf. Fig. 4 of the paper).
    fig, axs = plt.subplots(1, n, figsize=(4.2 * n, 5.0), squeeze=False)
    for c, (name, d) in enumerate(logs.items()):
        a = axs[0, c]
        # workspace box and excluded wedge {x >= 0.8 + |y|}, drawn in the (y, x) plane
        a.plot([-2, 2, 2, -2, -2], [-0.5, -0.5, 2.8, 2.8, -0.5], color="#3355bb", lw=1, label="workspace box")
        a.fill([0, 2, -2], [0.8, 2.8, 2.8], color="#f4a6a6", alpha=0.6, label="excluded wedge")
        a.plot(d["y2"], d["x2"], "g-", lw=1, label="tip p2")
        a.plot(d["y2"][0], d["x2"][0], "go"); a.plot(d["y2"][-1], d["x2"][-1], "g*", ms=10)
        a.plot([0], [0], "ks", label="base")
        a.set_xlim(-2.2, 2.2); a.set_ylim(-0.7, 3.0); a.set_aspect("equal")
        a.set_xlabel("y [m]"); a.set_ylabel("x [m]"); a.set_title(name + " (top view)"); a.legend(fontsize=7)
    fig.tight_layout()
    out2 = os.path.join(folder, "topview.png")
    fig.savefig(out2, dpi=110)
    print("wrote", out2)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "results")
