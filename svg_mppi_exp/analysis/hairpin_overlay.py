#!/usr/bin/env python3
"""헤어핀 구간에서 여러 실행의 주행 궤적을 겹쳐 그린다.

사용: python3 hairpin_overlay.py <out.png> "<제목1>:<run_dir1>" "<제목2>:<run_dir2>" ...
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

MAP_DIR = os.path.expanduser("~/forza_ws/race_stack/stack_master/maps/berlin")
POCKET = (-0.3, 1.4, -4.2, -2.1)


def main():
    out = sys.argv[1]
    specs = [a.rsplit(":", 1) for a in sys.argv[2:]]
    meta = yaml.safe_load(open(os.path.join(MAP_DIR, "berlin.yaml")))
    img = np.array(Image.open(os.path.join(MAP_DIR, meta["image"])).convert("L"))
    H, W = img.shape
    r, o = meta["resolution"], meta["origin"]
    ext = [o[0], o[0] + W * r, o[1], o[1] + H * r]
    fig, axes = plt.subplots(1, len(specs), figsize=(5.2 * len(specs), 5.6), squeeze=False)
    for ax, (title, run) in zip(axes[0], specs):
        rows = list(csv.DictReader(open(os.path.join(run, "trace.csv"))))
        x = np.array([float(q["x"]) for q in rows])
        y = np.array([float(q["y"]) for q in rows])
        ax.imshow(img, cmap="gray", extent=ext, origin="upper", interpolation="nearest")
        ax.add_patch(plt.Rectangle((POCKET[0], POCKET[2]), POCKET[1] - POCKET[0], POCKET[3] - POCKET[2],
                                   fill=False, ec="#d9a400", lw=1.5, ls="--"))
        ax.text(POCKET[0] + 0.05, POCKET[3] - 0.25, "dead-end pocket", color="#b08300", fontsize=9)
        ax.scatter(x, y, s=1.2, c="#2e6fd8", alpha=0.35)
        ins = (x > POCKET[0]) & (x < POCKET[1]) & (y > POCKET[2]) & (y < POCKET[3])
        ax.scatter(x[ins], y[ins], s=1.5, c="#d23c3c", alpha=0.6)
        ax.set_xlim(-5.0, 2.0)
        ax.set_ylim(-7.5, 0.5)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=11)
        ax.set_xticks([])
        ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("saved", out)


if __name__ == "__main__":
    main()
