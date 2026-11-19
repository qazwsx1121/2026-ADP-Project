#!/usr/bin/env python3
"""berlin 헤어핀에서 svg_mppi 노드가 만드는 기준 경로 지도(ref SDF)를 그대로 재현해 그린다.

사용 (컨테이너):
  python3 ref_sdf_check.py --s 20 22 24 --out ~/shared_dir/svg_mppi_exp/analysis/out
노드의 calc_waypoints + build_reference_sdf 를 파이썬으로 옮긴 것 (num_waypoints 0.1 m 간격, 최근접점 거리).
"""
import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

MAP_DIR = os.path.expanduser("~/forza_ws/race_stack/stack_master/maps/berlin")


def load_map():
    meta = yaml.safe_load(open(os.path.join(MAP_DIR, "berlin.yaml")))
    img = np.array(Image.open(os.path.join(MAP_DIR, meta["image"])).convert("L"), dtype=float) / 255.0
    occ = (1.0 - img) > meta["occupied_thresh"]  # True = wall
    return occ, meta["resolution"], meta["origin"]


def load_wpnts():
    d = json.load(open(os.path.join(MAP_DIR, "global_waypoints.json")))
    w = d["global_traj_wpnts_iqp"]["wpnts"]
    return np.array([[p["x_m"], p["y_m"], p["s_m"], p["vx_mps"]] for p in w])


def window(path, nearest, num, interval=0.1):
    n = len(path)
    out = [nearest]
    cur = nearest
    for _ in range(1, num):
        idx = cur
        for _ in range(n):
            idx = (idx + 1) % n
            if np.hypot(*(path[idx, :2] - path[cur, :2])) >= interval:
                break
        out.append(idx)
        cur = idx
    return np.array(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--s", type=float, nargs="+", default=[20.0, 22.0, 24.0])
    ap.add_argument("--num", type=int, nargs="+", default=[80])
    ap.add_argument("--xlim", type=float, nargs=2, default=[-6.0, 2.0])
    ap.add_argument("--ylim", type=float, nargs=2, default=[-9.0, 1.0])
    ap.add_argument("--out", default=os.path.expanduser("~/shared_dir/svg_mppi_exp/analysis/out"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    occ, res, origin = load_map()
    H, W = occ.shape
    path = load_wpnts()

    # grid of the plotted area
    xs = np.arange(a.xlim[0], a.xlim[1], 0.05)
    ys = np.arange(a.ylim[0], a.ylim[1], 0.05)
    X, Y = np.meshgrid(xs, ys)
    col = ((X - origin[0]) / res).astype(int)
    row = (H - 1 - (Y - origin[1]) / res).astype(int)
    inside = (col >= 0) & (col < W) & (row >= 0) & (row < H)
    wall = np.ones_like(X, dtype=bool)
    wall[inside] = occ[row[inside], col[inside]]

    fig, axes = plt.subplots(len(a.num), len(a.s), figsize=(5 * len(a.s), 5.6 * len(a.num)), squeeze=False)
    for r, num in enumerate(a.num):
        for c, s0 in enumerate(a.s):
            ax = axes[r][c]
            i0 = int(np.argmin(np.abs(path[:, 2] - s0)))
            win = window(path, i0, num)
            P = path[win, :2]
            D = np.min(np.hypot(X[..., None] - P[:, 0], Y[..., None] - P[:, 1]), axis=-1)
            NI = np.argmin(np.hypot(X[..., None] - P[:, 0], Y[..., None] - P[:, 1]), axis=-1)
            Dm = np.ma.masked_where(wall, D)
            im = ax.pcolormesh(X, Y, Dm, cmap="viridis_r", vmin=0, vmax=1.5, shading="auto")
            ax.contour(X, Y, wall.astype(float), levels=[0.5], colors="k", linewidths=1.2)
            ax.plot(path[:, 0], path[:, 1], color="0.6", lw=0.8)
            ax.plot(P[:, 0], P[:, 1], color="r", lw=2, label=f"ref window {num * 0.1:.0f} m")
            # mark the part of the window that is > 3 m ahead along the path
            far = np.arange(num) * 0.1 > 3.0
            ax.plot(P[far, 0], P[far, 1], color="m", lw=2.5, label="window part > 3 m ahead")
            ax.plot(*path[i0, :2], "wo", mec="k", ms=8)
            ax.set_title(f"car at s={path[i0, 2]:.1f} m, window {num * 0.1:.0f} m")
            ax.set_xlim(a.xlim)
            ax.set_ylim(a.ylim)
            ax.set_aspect("equal")
            ax.legend(loc="lower left", fontsize=7)
            fig.colorbar(im, ax=ax, fraction=0.04, label="ref distance d [m] (cost q_dist*d^2)")
            # cells in free space that are near (<0.5 m) a waypoint that is > 3 m ahead,
            # while being > 1.0 m from the first 3 m of the window -> "pulled toward the far side"
            near_far = ~wall & (D < 0.6) & (NI * 0.1 > 3.0)
            ax.contourf(X, Y, near_far.astype(float), levels=[0.5, 1.5], colors="none", hatches=["...."])
    fig.tight_layout()
    fn = os.path.join(a.out, "ref_sdf_" + "_".join(str(n) for n in a.num) + ".png")
    fig.savefig(fn, dpi=90)
    print("saved", fn)


if __name__ == "__main__":
    main()
