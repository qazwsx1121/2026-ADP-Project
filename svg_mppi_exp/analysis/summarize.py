#!/usr/bin/env python3
"""실행 결과 요약: 랩/베스트/평균/충돌/복구/주머니 진입/계산시간.

사용: python3 summarize.py <run_dir> [<run_dir> ...]
주머니(헤어핀 입구 정면 막다른 공간) = x in [-0.3, 1.4], y in [-4.2, -2.1] 영역에 차가 들어간 횟수.
"""
import csv
import glob
import json
import os
import sys

POCKET = (-0.3, 1.4, -4.2, -2.1)


def summarize(run):
    out = {"run": os.path.basename(run.rstrip("/"))}
    js = sorted(glob.glob(os.path.join(run, "race_result_*.json")))
    if js:
        r = json.load(open(js[-1]))
        laps = r["laps"]
        clean = [l["time"] for l in laps if not l["total"] and not l["partial"]]
        out.update(laps=r["laps_completed"], best=r["best_lap"], hits=r["collisions"]["total"],
                   obs_hits=r["collisions"]["obstacle"], wall_hits=r["collisions"]["wall"],
                   teleports=len(r.get("teleports", [])),
                   clean_mean=sum(clean) / len(clean) if clean else float("nan"),
                   clean_laps=len(clean), slow_laps=sum(1 for t in clean if clean and t > min(clean) + 3.0))
    tr = os.path.join(run, "trace.csv")
    if os.path.exists(tr):
        rows = list(csv.DictReader(open(tr)))
        inside, entries, last_t = False, 0, -99
        calc = []
        for row in rows:
            x, y, t = float(row["x"]), float(row["y"]), float(row["t"])
            ins = POCKET[0] < x < POCKET[1] and POCKET[2] < y < POCKET[3]
            if ins and not inside and t - last_t > 3.0:
                entries += 1
                last_t = t
            inside = ins
            c = float(row["calc_ms"])
            if c == c:
                calc.append(c)
        out["pocket"] = entries
        if calc:
            calc.sort()
            out["calc_mean"] = sum(calc) / len(calc)
            out["calc_p95"] = calc[int(0.95 * (len(calc) - 1))]
            out["overrun%"] = 100.0 * sum(1 for c in calc if c > 25.0) / len(calc)
    return out


def main():
    keys = ["run", "laps", "clean_laps", "best", "clean_mean", "slow_laps", "pocket", "hits", "obs_hits", "wall_hits",
            "teleports", "calc_mean", "calc_p95", "overrun%"]
    print(" | ".join(keys))
    for run in sys.argv[1:]:
        o = summarize(run)
        print(" | ".join(("%.2f" % o[k]) if isinstance(o.get(k), float) else str(o.get(k, "-")) for k in keys))


if __name__ == "__main__":
    main()
