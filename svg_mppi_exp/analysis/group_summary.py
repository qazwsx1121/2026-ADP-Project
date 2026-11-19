#!/usr/bin/env python3
"""여러 seed 결과를 조건별로 합산: python3 group_summary.py <prefix(gA, gB ...)> ... [--runs <runs_dir>]

조건 이름 = run 폴더 이름의 태그 앞부분 (예: 20261119_..._gA_s2026 -> gA)
"""
import argparse
import glob
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from summarize import summarize  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("groups", nargs="+")
    ap.add_argument("--runs", default=os.path.expanduser("~/shared_dir/svg_mppi_exp/runs"))
    a = ap.parse_args()
    print("group | seeds | laps | clean | clean% | obs_hits | wall_hits | recovers | clean_mean | best | calc_mean | per-seed (laps/clean/hits)")
    for g in a.groups:
        runs = sorted(d for d in glob.glob(os.path.join(a.runs, f"*_{g}_s*")) if glob.glob(os.path.join(d, "race_result_*.png")))
        if not runs:
            print(f"{g} | 0")
            continue
        tot = dict(laps=0, clean_laps=0, obs_hits=0, wall_hits=0, teleports=0)
        clean_times, best, calc, per = [], [], [], []
        for r in runs:
            o = summarize(r)
            for k in tot:
                tot[k] += o.get(k, 0) or 0
            js = sorted(glob.glob(os.path.join(r, "race_result_*.json")))
            laps = json.load(open(js[-1]))["laps"]
            clean_times += [l["time"] for l in laps if not l["total"] and not l["partial"]]
            if o.get("best"):
                best.append(o["best"])
            if o.get("calc_mean"):
                calc.append(o["calc_mean"])
            seed = re.search(r"_s(\d+)$", r).group(1)
            per.append(f"s{seed}:{o.get('laps')}/{o.get('clean_laps')}/{o.get('hits')}")
        cm = sum(clean_times) / len(clean_times) if clean_times else float("nan")
        pct = 100.0 * tot["clean_laps"] / tot["laps"] if tot["laps"] else 0.0
        print(f"{g} | {len(runs)} | {tot['laps']} | {tot['clean_laps']} | {pct:.0f}% | {tot['obs_hits']} | {tot['wall_hits']} | "
              f"{tot['teleports']} | {cm:.2f} | {min(best) if best else float('nan'):.2f} | "
              f"{sum(calc) / len(calc) if calc else float('nan'):.1f} | {' '.join(per)}")


if __name__ == "__main__":
    main()
