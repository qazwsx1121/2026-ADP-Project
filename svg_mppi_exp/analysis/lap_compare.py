#!/usr/bin/env python3
"""헤어핀 진입 구간(s 범위)을 지날 때마다 상태를 비교: 정상 통과 vs 주머니 진입.

사용: python3 lap_compare.py <run_dir> [--s 21 24.5]
"""
import argparse
import csv
import os

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--s", type=float, nargs=2, default=[21.0, 24.4])
    ap.add_argument("--marks", type=float, nargs="+", default=[21.5, 22.0, 22.5, 23.0, 23.5, 24.0])
    a = ap.parse_args()
    rows = list(csv.DictReader(open(os.path.join(a.run, "trace.csv"))))
    t = np.array([float(r["t"]) for r in rows])
    s = np.array([float(r["s"]) for r in rows])
    d = np.array([float(r["d"]) for r in rows])
    v = np.array([float(r["v"]) for r in rows])
    st = np.array([float(r["steer_cmd"]) for r in rows])
    cr = np.array([float(r["coll_rate"]) for r in rows])
    cm = np.array([float(r["calc_ms"]) for r in rows])
    # passes: contiguous segments with s in range and moving forward
    inr = (s >= a.s[0]) & (s <= a.s[1])
    passes, i = [], 0
    while i < len(s):
        if inr[i]:
            j = i
            while j < len(s) and inr[j]:
                j += 1
            if j - i > 5 and s[i] < a.s[0] + 0.3:
                passes.append((i, j))
            i = j
        else:
            i += 1
    print("pass  t_in    result  | " + " ".join(f"s={m:4.1f}(st,d,v)    " for m in a.marks) + "| calc max  coll max")
    for i, j in passes:
        # outcome: within 3 s after the pass, does |d| exceed 0.8 ?
        k2 = np.searchsorted(t, t[j - 1] + 3.0)
        bad = np.max(np.abs(d[i:k2])) > 0.8
        cells = []
        for m in a.marks:
            k = i + np.argmin(np.abs(s[i:j] - m))
            cells.append(f"{st[k]:+.2f},{d[k]:+.2f},{v[k]:.2f}")
        print(f"{t[i]:7.1f}  {'POCKET' if bad else 'ok    '} | " + "  ".join(cells) +
              f" | {np.nanmax(cm[i:j]):5.1f}  {np.nanmax(cr[i:j]):.2f}")


if __name__ == "__main__":
    main()
