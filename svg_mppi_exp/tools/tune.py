#!/usr/bin/env python3
"""Optuna 로 상황별 파라미터 표를 학습한다 (학습 A+B안).

전제: 시뮬 + svg_mppi + race_manager + exp_monitor 가 이미 돌고 있다 (run_exp.sh 를 긴 dur 로 실행).
한 trial = 파라미터 표 하나를 넣고 -> 다음 랩 경계부터 laps 랩을 달린 결과로 점수 계산.
  점수(작을수록 좋음) = 평균 랩타임 + hit_penalty * 충돌 수 + teleport_penalty * 복구 수
결과는 sqlite(DB)에 쌓여서 중단 후 이어서 돌릴 수 있고, 다른 PC로 파일만 옮겨도 된다.
가장 좋은 표는 <out>/best_schedule.yaml 로 저장 -> param_scheduler.py 에 그대로 넣어 주행 중 자동 적용.

사용 (컨테이너, optuna 가 있는 venv 에서):
  보통은 train.sh 가 대신 실행한다 (PYTHONPATH 에 svg_mppi_exp/pylib 필요: optuna 설치 위치)
"""
import argparse
import glob
import json
import os
import sys
import threading
import time

import optuna
import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from param_scheduler import ParamScheduler, normalize  # noqa: E402


def newest_json(log_dir):
    js = sorted(glob.glob(os.path.join(log_dir, "race_result_*.json")), key=os.path.getmtime)
    return js[-1] if js else None


def read_json(path):
    for _ in range(5):
        try:
            return json.load(open(path))
        except (json.JSONDecodeError, OSError):
            time.sleep(0.2)
    return None


def build_table(space, trial):
    """space.yaml -> trial 이 고른 값으로 schedule 표(dict)를 만든다."""
    table = {"default": dict(space.get("fixed", {}).get("default", {})), "sectors": [], "obstacle": {}}
    sectors = {s["name"]: {"name": s["name"], "s": s["s"], "params": dict(s.get("params", {}))} for s in space.get("sectors", [])}
    table["obstacle"] = dict(space.get("fixed", {}).get("obstacle", {}))
    for key, spec in space["space"].items():
        scope, pname = key.split(".", 1)
        lo, hi = spec[0], spec[1]
        log = len(spec) > 2 and spec[2] == "log"
        if isinstance(lo, int) and isinstance(hi, int):
            v = trial.suggest_int(key, lo, hi, log=log)
        else:
            v = trial.suggest_float(key, float(lo), float(hi), log=log)
        if scope == "default":
            table["default"][pname] = v
        elif scope == "obstacle":
            table["obstacle"][pname] = v
        else:
            sectors[scope]["params"][pname] = v
    table["sectors"] = list(sectors.values())
    return table


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--space", required=True)
    ap.add_argument("--log-dir", required=True, help="race_manager log_dir (run_exp.sh OUT)")
    ap.add_argument("--study", default="berlin")
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--laps", type=int, default=3)
    ap.add_argument("--hit-penalty", type=float, default=10.0)
    ap.add_argument("--teleport-penalty", type=float, default=15.0)
    ap.add_argument("--lap-timeout", type=float, default=60.0)
    ap.add_argument("--out", default=os.path.expanduser("~/shared_dir/svg_mppi_exp/tuning"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    space = yaml.safe_load(open(a.space))

    rclpy.init()
    sched = ParamScheduler()
    ex = MultiThreadedExecutor(2)
    ex.add_node(sched)
    threading.Thread(target=ex.spin, daemon=True).start()

    def wait_laps(n_after):
        """다음 랩 경계를 기다린 뒤 n_after 랩 결과를 돌려준다 (laps list)."""
        t_start = time.time()
        j = None
        while j is None:
            p = newest_json(a.log_dir)
            j = read_json(p) if p else None
            if j is None:
                time.sleep(1)
        base = len(j["laps"])
        # 현재 진행 중인 랩은 파라미터가 섞였으므로 버린다
        while True:
            j = read_json(newest_json(a.log_dir)) or j
            if len(j["laps"]) > base:
                break
            if time.time() - t_start > a.lap_timeout:
                return None, j
            time.sleep(0.5)
        base = len(j["laps"])
        t_lap = time.time()
        while len(j["laps"]) < base + n_after:
            time.sleep(0.5)
            j2 = read_json(newest_json(a.log_dir))
            if j2:
                if len(j2["laps"]) > len(j["laps"]):
                    t_lap = time.time()
                j = j2
            if time.time() - t_lap > a.lap_timeout:
                break
        return j["laps"][base:base + n_after], j

    def objective(trial):
        table = build_table(space, trial)
        sched.default = normalize(table["default"])
        sched.sectors = [(s["name"], float(s["s"][0]), float(s["s"][1]), normalize(s["params"])) for s in table["sectors"]]
        sched.obstacle = normalize(table["obstacle"])
        sched.state = None  # force re-apply
        laps, j = wait_laps(a.laps)
        if not laps:
            return 1e3
        tele = j.get("teleports", [])
        lap_ids = {l["lap"] for l in laps}
        n_tele = sum(1 for t in tele if t.get("lap") in lap_ids)
        hits = sum(l["total"] for l in laps)
        mean_t = sum(l["time"] for l in laps) / len(laps)
        missing = a.laps - len(laps)
        score = mean_t + a.hit_penalty * hits + a.teleport_penalty * n_tele + 30.0 * missing
        trial.set_user_attr("laps", [l["time"] for l in laps])
        trial.set_user_attr("hits", hits)
        trial.set_user_attr("teleports", n_tele)
        print(f"trial {trial.number}: score={score:.2f} mean={mean_t:.2f} hits={hits} tele={n_tele} params={trial.params}", flush=True)
        log = os.path.join(a.out, f"{a.study}_trials.csv")
        new_file = not os.path.exists(log)
        with open(log, "a") as f:
            if new_file:
                f.write("time,trial,score,mean_lap,hits,teleports,laps,params\n")
            f.write(f"{time.strftime('%H:%M:%S')},{trial.number},{score:.3f},{mean_t:.3f},{hits},{n_tele},"
                    f"\"{' '.join('%.2f' % l['time'] for l in laps)}\",\"{trial.params}\"\n")
        return score

    storage = f"sqlite:///{os.path.join(a.out, a.study)}.db"
    study = optuna.create_study(study_name=a.study, storage=storage, load_if_exists=True, direction="minimize",
                                sampler=optuna.samplers.TPESampler(seed=0, n_startup_trials=8))
    if space.get("baseline") and len(study.trials) == 0:
        study.enqueue_trial(space["baseline"])  # 첫 trial 은 현재 설정 (비교 기준)
    study.optimize(objective, n_trials=a.trials)

    best = optuna.trial.FixedTrial(study.best_trial.params)
    table = build_table(space, best)
    out = os.path.join(a.out, f"{a.study}_best_schedule.yaml")
    yaml.safe_dump(table, open(out, "w"), allow_unicode=True, sort_keys=False)
    print(f"best score {study.best_value:.2f}  -> {out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
