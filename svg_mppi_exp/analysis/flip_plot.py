#!/usr/bin/env python3
"""flip_*.json (조향이 뒤집힌 순간 solver 가 본 것) 을 그림으로.

사용: python3 flip_plot.py <run_dir> [--max 6]
- 회색: 충돌 지도(라이다+inflate) 칸, 검정: 실제 벽
- 후보 궤적(상위 N개) 색 = 가중치, 파랑 굵게 = 최종 선택(best), 초록 = SVG guide 입자, 빨강 = 기준 경로
"""
import argparse
import glob
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

MAP_DIR = os.path.expanduser("~/forza_ws/race_stack/stack_master/maps/berlin")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--max", type=int, default=6)
    a = ap.parse_args()
    files = sorted(glob.glob(os.path.join(a.run, "flip_*.json")))[: a.max]
    if not files:
        print("no flip_*.json")
        return
    meta = yaml.safe_load(open(os.path.join(MAP_DIR, "berlin.yaml")))
    img = np.array(Image.open(os.path.join(MAP_DIR, meta["image"])).convert("L"))
    H, W = img.shape
    r, o = meta["resolution"], meta["origin"]
    ext = [o[0], o[0] + W * r, o[1], o[1] + H * r]
    n = len(files)
    fig, axes = plt.subplots(1, n, figsize=(6.5 * n, 6.5), squeeze=False)
    for ax, fn in zip(axes[0], files):
        d = json.load(open(fn))
        x0 = d["x0"]
        ax.imshow(img, cmap="gray", extent=ext, origin="upper", alpha=0.5)
        obs = np.array(d["obstacles"]) if d["obstacles"] else np.zeros((0, 2))
        ax.scatter(obs[:, 0], obs[:, 1], s=1, c="0.55", label="costmap occupied")
        w = np.array(d["weights"])
        wn = w / w.max() if w.max() > 0 else w
        order = np.argsort(wn)
        for k in order:
            c = np.array(d["candidates"][k])
            ax.plot(c[:, 0], c[:, 1], color=plt.cm.viridis(wn[k]), lw=0.6, alpha=0.5)
        ref = np.array(d["ref"])
        if len(ref):
            ax.plot(ref[:, 0], ref[:, 1], "r-", lw=1.5, label="reference")
        b = np.array(d["best"])
        g = np.array(d["guide"])
        ax.plot(g[:, 0], g[:, 1], "g-", lw=2.5, label=f"SVG guide (cost {d['guide_cost']:.0f}, coll {d['guide_collision_cost']:.0f})")
        ax.plot(b[:, 0], b[:, 1], "b-", lw=2.5, label="output (weighted mean)")
        ax.plot(x0[0], x0[1], "ko", ms=6)
        # how many top candidates turn left vs right at the end
        left = sum(1 for c in d["candidates"] if (np.arctan2(np.sin(c[-1][2] - x0[2]), np.cos(c[-1][2] - x0[2])) > 0))
        ax.set_title(f"{os.path.basename(fn)}\nsteer {d['prev_steer']:+.2f} -> {d['steer']:+.2f}, collision rate {d['collision_rate']:.2f}\n"
                     f"top{len(d['candidates'])} candidates: {left} turn left / {len(d['candidates']) - left} right", fontsize=9)
        ax.set_xlim(x0[0] - 3, x0[0] + 3)
        ax.set_ylim(x0[1] - 3, x0[1] + 3)
        ax.set_aspect("equal")
        ax.legend(fontsize=7, loc="lower left")
    fig.tight_layout()
    out = os.path.join(a.run, "flips.png")
    fig.savefig(out, dpi=80)
    print("saved", out)


if __name__ == "__main__":
    main()
