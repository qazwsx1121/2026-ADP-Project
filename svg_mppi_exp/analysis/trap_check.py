#!/usr/bin/env python3
"""seed 2026 2랩째 (헤어핀 s 24.6 장애물) 통과 여부: 그 배치가 놓인 동안 s 22~27 에서의 충돌/복구 수와 랩 결과."""
import csv, glob, json, os, sys
for run in sys.argv[1:]:
    ev = list(csv.DictReader(open(os.path.join(run, "events.csv"))))
    t0 = t1 = None
    for e in ev:
        if e["event"] == "obstacles":
            if t0 is not None and t1 is None:
                t1 = float(e["t"])
            if "s=24.6" in e["info"] and t0 is None:
                t0 = float(e["t"])
    if t0 is None:
        print(os.path.basename(run), "trap layout not reached"); continue
    t1 = t1 or 1e9
    bad = [e for e in ev if e["event"] in ("collision", "recover") and t0 <= float(e["t"]) < t1]
    coll = sum(1 for e in bad if e["event"] == "collision")
    rec = [e["info"] for e in bad if e["event"] == "recover"]
    nop = sum(1 for e in ev if e["event"] == "recover" and t0 <= float(e["t"]) < t1 and "no progress" in e["info"])
    r = json.load(open(sorted(glob.glob(os.path.join(run, "race_result_*.json")))[-1]))
    lap2 = r["laps"][1] if len(r["laps"]) > 1 else None
    print(f"{os.path.basename(run)}: trap {t1 - t0 if t1 < 1e9 else -1:.0f}s, collisions {coll}, recovers {len(rec)} (no-progress {nop}), "
          f"lap2={lap2['time'] if lap2 else '-'} hits={lap2['total'] if lap2 else '-'} partial={lap2['partial'] if lap2 else '-'} | laps={r['laps_completed']} hits={r['collisions']['total']}")
