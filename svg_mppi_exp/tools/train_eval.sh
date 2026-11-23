#!/bin/bash
# 최종 평가 방식으로 학습 (설정 하나로 장애물 1/3/5 개, 충돌하면 그 run 끝, pf).
#
# 사용 (컨테이너 터미널, misys@):
#   bash ~/shared_dir/svg_mppi_exp/tools/train_eval.sh <study> [key=value ...]
# key:
#   trials=100      trial 수. trial 1개 = 장애물 1/3/5 x 최대 laps 바퀴 ≈ 4~5분 -> 하룻밤 100개 정도
#   laps=4          조건마다 최대 바퀴 (평가는 10바퀴, 학습은 시간 때문에 4)
#   seed=100        장애물 seed (장애물 k 개 조건은 seed+k). 평가 seed 는 비공개라 검증은 다른 seed 로
#   base=<yaml>     기본 설정 (기본: configs/school_orig_t8.yaml)
#   space=<yaml>    탐색 공간 (기본: configs/tune_space.yaml)
#   pose=pf         pf(기본, 평가와 같음) | gt
#   extra="..."     run_exp.sh 에 넘길 옵션 (예: "snoise=0.03 sdrop=0.02")
# 결과: tuning/<study>_trials.csv, tuning/<study>_best.yaml (최종 설정 파일 형태), tuning/<study>.db
# 멈추기: Ctrl-C (또는 pkill -INT -f tune_eval.py). 같은 이름으로 다시 실행하면 이어서.
# 밤새 돌리기:
#   nohup bash ~/shared_dir/svg_mppi_exp/tools/train_eval.sh berlin_e1 trials=100 > ~/shared_dir/svg_mppi_exp/tuning/berlin_e1_train.log 2>&1 &

STUDY=${1:?usage: train_eval.sh <study> [key=value ...]}; shift
trials=100; laps=4; seed=100; base=""; space=""; pose=pf; extra=""
for kv in "$@"; do printf -v "${kv%%=*}" '%s' "${kv#*=}"; done
EXP=~/shared_dir/svg_mppi_exp
source /opt/ros/humble/setup.bash
source ~/forza_ws/race_stack/install/setup.bash
[ -z "$base" ] && base=$EXP/configs/school_orig_t8.yaml
[ -z "$space" ] && space=$EXP/configs/tune_space.yaml
export PYTHONPATH=$EXP/pylib:$PYTHONPATH
mkdir -p $EXP/tuning
echo "study=$STUDY trials=$trials laps=$laps seed=$seed base=$base pose=$pose extra=$extra ($(date))"
python3 $EXP/tools/tune_eval.py --space "$space" --base "$base" --study "$STUDY" --trials $trials --laps $laps \
  --seed $seed --pose $pose --out $EXP/tuning --runs $EXP/runs $extra
