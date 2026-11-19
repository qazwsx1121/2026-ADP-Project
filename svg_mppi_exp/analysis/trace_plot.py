#!/usr/bin/env python3
"""trace.csv 분석: 레이스라인에서 크게 벗어난 구간(이탈 이벤트)을 찾아 지도 위에 그린다.

사용 (컨테이너): python3 trace_plot.py <run_dir> [--dmax 0.6] [--xlim ..] [--ylim ..]
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

MAP_DIR = os.path.expanduser("~/forza_ws/race_stack/stack_master/maps/berlin")


def load_trace(run):
    rows = list(csv.DictReader(open(os.path.join(run, "trace.csv"))))
    keys = ["t", "x", "y", "yaw", "v", "steer_cmd", "speed_cmd", "calc_ms", "coll_rate", "s", "d", "best_end_x", "best_end_y"]
    T = {k: np.array([float(r[k]) for r in rows]) for k in keys}
    T["collision"] = np.array([r["collision"] for r in rows])
    return T


def excursions(T, dmax):
    """|d| > dmax 인 연속 구간 -> (start_idx, end_idx)"""
    bad = np.abs(T["d"]) > dmax
    ev, i = [], 0
    while i < len(bad):
        if bad[i]:
            j = i
            while j < len(bad) and bad[j]:
                j += 1
            if T["t"][j - 1] - T["t"][i] > 0.2:
                ev.append((i, j))
            i = j
        else:
            i += 1
    return ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--dmax", type=float, default=0.6)
    ap.add_argument("--xlim", type=float, nargs=2, default=[-5.5, 1.8])
    ap.add_argument("--ylim", type=float, nargs=2, default=[-7.5, 1.0])
    ap.add_argument("--pre", type=float, default=1.5, help="seconds before excursion to plot")
    a = ap.parse_args()
    T = load_trace(a.run)
    ev = excursions(T, a.dmax)
    print(f"{len(ev)} excursions (|d|>{a.dmax} m)")
    for i, j in ev:
        k = i + np.argmax(np.abs(T["d"][i:j]))
        print(f"  t={T['t'][i]:7.2f}-{T['t'][j-1]:7.2f}s  s={T['s'][i]:5.1f}  max|d|={abs(T['d'][k]):.2f} at ({T['x'][k]:.2f},{T['y'][k]:.2f})"
              f"  v_in={T['v'][i]:.2f}  calc_ms(mean)={np.nanmean(T['calc_ms'][max(0,i-40):i]):.1f}")
    if not ev:
        return

    meta = yaml.safe_load(open(os.path.join(MAP_DIR, "berlin.yaml")))
    img = np.array(Image.open(os.path.join(MAP_DIR, meta["image"])).convert("L"))
    H, W = img.shape
    r, o = meta["resolution"], meta["origin"]
    ext = [o[0], o[0] + W * r, o[1], o[1] + H * r]

    n = len(ev)
    fig, axes = plt.subplots(2, n, figsize=(6 * n, 10), squeeze=False)
    for c, (i, j) in enumerate(ev):
        dt = T["t"][1] - T["t"][0]
        i0 = max(0, i - int(a.pre / max(dt, 1e-3)))
        j1 = min(len(T["t"]), j + 20)
        ax = axes[0][c]
        ax.imshow(img, cmap="gray", extent=ext, origin="upper")
        # a normal lap in the same s-range for reference
        sc = ax.scatter(T["x"][i0:j1], T["y"][i0:j1], c=T["t"][i0:j1] - T["t"][i], cmap="plasma", s=6)
        for k in range(i0, j1, 8):  # predicted best-path end
            ax.plot([T["x"][k], T["best_end_x"][k]], [T["y"][k], T["best_end_y"][k]], color="c", lw=0.5, alpha=0.6)
        fig.colorbar(sc, ax=ax, label="t - t_excursion [s]")
        ax.set_xlim(a.xlim); ax.set_ylim(a.ylim); ax.set_aspect("equal")
        ax.set_title(f"excursion {c+1}: t={T['t'][i]:.1f}s s={T['s'][i]:.1f}")
        ax2 = axes[1][c]
        tt = T["t"][i0:j1] - T["t"][i]
        ax2.plot(tt, T["steer_cmd"][i0:j1], label="steer cmd [rad]")
        ax2.plot(tt, T["v"][i0:j1] / 5, label="v/5 [m/s]")
        ax2.plot(tt, T["d"][i0:j1], label="d (left +) [m]")
        ax2.plot(tt, T["coll_rate"][i0:j1], label="MPPI collision rate")
        ax2.plot(tt, T["calc_ms"][i0:j1] / 100, label="calc ms /100")
        ax2.axhline(0.4189, color="k", ls=":", lw=0.8); ax2.axhline(-0.4189, color="k", ls=":", lw=0.8)
        ax2.legend(fontsize=7); ax2.grid(alpha=0.3); ax2.set_xlabel("t - t_excursion [s]")
    fig.tight_layout()
    fn = os.path.join(a.run, "excursions.png")
    fig.savefig(fn, dpi=80)
    print("saved", fn)


if __name__ == "__main__":
    main()
