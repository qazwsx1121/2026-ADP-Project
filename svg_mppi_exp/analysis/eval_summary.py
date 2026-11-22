#!/usr/bin/env python3
"""최종 평가 방식(장애물 1/3/5개, 첫 충돌까지 연속 랩) 결과를 설정별로 합산한다.

run 폴더 이름: *_<설정>_o<장애물 수>_s<seed>  (eval_sweep.sh 가 이렇게 만듦)
사용: python3 eval_summary.py <설정> [<설정> ...] [--runs <runs 폴더>]

칸마다: 평균 연속 랩 / 10바퀴 완주한 run 수 / 연속 구간 최고 랩의 평균 (0 랩 run 제외)
평가 점수는 6개 순위(장애물 1/3/5 x lap count, best lap time)의 합이라, 연속 랩(=충돌 없음)이 가장 중요하다.
"""
import argparse
import glob
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from summarize import summarize  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cfgs", nargs="+")
    ap.add_argument("--runs", default=os.path.expanduser("~/shared_dir/svg_mppi_exp/runs"))
    a = ap.parse_args()
    obs_list = (1, 3, 5)
    print("설정 | " + " | ".join(f"장애물 {n}: 연속랩 평균 / 10바퀴 run / 최고랩 평균" for n in obs_list) + " | 충돌 위치(s)")
    for c in a.cfgs:
        cells, crash_s = [], []
        for n in obs_list:
            runs = sorted(d for d in glob.glob(os.path.join(a.runs, f"*_{c}_o{n}_s*"))
                          if glob.glob(os.path.join(d, "race_result_*.json")))
            if not runs:
                cells.append("-")
                continue
            res = [summarize(r) for r in runs]
            st = [o.get("streak", 0) for o in res]
            full = sum(1 for x in st if x >= 10)
            bests = [o["streak_best"] for o in res if o.get("streak", 0) > 0]
            cells.append(f"{sum(st) / len(st):.1f} / {full}of{len(res)} / "
                         f"{(sum(bests) / len(bests)) if bests else float('nan'):.2f}")
            for r in runs:
                stop = os.path.join(r, "STOP")
                if os.path.exists(stop):
                    m = re.search(r"s=([\d.]+)", open(stop).read())
                    crash_s.append(f"o{n}:{float(m.group(1)):.0f}" if m else f"o{n}:?")
        print(f"{c} | " + " | ".join(cells) + " | " + " ".join(crash_s))


if __name__ == "__main__":
    main()
