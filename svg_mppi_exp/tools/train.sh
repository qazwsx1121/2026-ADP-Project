#!/bin/bash
# 학습 한 번에: 시뮬+MPPI+race_manager(장애물 4개)를 길게 띄우고 tune.py(Optuna)로 구간별 파라미터 표를 학습한다.
#
# 사용 (컨테이너 터미널, misys@):
#   bash ~/shared_dir/svg_mppi_exp/tools/train.sh <study 이름> [key=value ...]
# key:
#   trials=60          trial 수 (trial 1개 ≈ laps+1 랩 ≈ 1.5 분)
#   laps=4             trial 하나에서 평가하는 랩 수 (랩마다 장애물 배치가 바뀜)
#   seed=100           학습용 seed (검증 seed 2026 11 7 42 5 와 겹치지 않게)
#   params=<yaml>      svg_mppi 설정 (기본: 패키지 svg_mppi_light.yaml)
#   space=<yaml>       탐색 공간 (기본: configs/tune_space.yaml)
#   noise="onoise=0.10 orot=1 snoise=0.03 sdrop=0.02 sghost=0.005"   노이즈를 켜고 학습 (run_exp.sh 옵션 그대로)
# 중간에 Ctrl-C 해도 결과는 tuning/<study>.db 에 남고, 같은 이름으로 다시 실행하면 이어서 학습한다.
# 결과: ~/shared_dir/svg_mppi_exp/tuning/<study>_best_schedule.yaml, <study>_trials.csv

STUDY=${1:?usage: train.sh <study> [trials=60 laps=4 seed=100 params=... space=...]}; shift
trials=60; laps=4; seed=100; params=""; space=""; noise=""
for kv in "$@"; do printf -v "${kv%%=*}" '%s' "${kv#*=}"; done   # key=value (값에 공백이 있어도 됨)
EXP=~/shared_dir/svg_mppi_exp
[ -z "$space" ] && space=$EXP/configs/tune_space.yaml
source /opt/ros/humble/setup.bash
source ~/forza_ws/race_stack/install/setup.bash
[ -z "$params" ] && params=$(ros2 pkg prefix svg_mppi)/share/svg_mppi/config/svg_mppi_light.yaml
mkdir -p $EXP/tuning

# 1) 시뮬을 길게 (trial 수 x 2분 + 여유)
dur=$(( trials * (laps + 2) * 20 + 600 ))
TAG=train_${STUDY}
START=$(date +%Y%m%d_%H%M%S)   # 이 시각 이후에 생긴 실험 폴더만 쓴다 (이어서 학습할 때 옛 폴더를 잡지 않게)
bash $EXP/tools/run_exp.sh $TAG dur=$dur obs=4 seed=$seed params=$params $noise > $EXP/tuning/${STUDY}_sim.log 2>&1 &
SIM_PID=$!
stop_sim() {
  kill $SIM_PID 2>/dev/null; sleep 1
  pkill -f "run_exp.sh $TAG" 2>/dev/null
  pkill -f "ros2 launch" 2>/dev/null; pkill -f rviz2 2>/dev/null; pkill -f gym_bridge 2>/dev/null
  pkill -f svg_mppi_node 2>/dev/null; pkill -f exp_monitor.py 2>/dev/null; pkill -f obstacle_spawner 2>/dev/null; pkill -f "race_manager/scorer" 2>/dev/null
  pkill -f obstacle_noise.py 2>/dev/null; pkill -f scan_noise.py 2>/dev/null
}
trap stop_sim EXIT

# 2) race_manager 기록 파일이 생길 때까지 대기
OUT=""
for i in $(seq 1 120); do
  OUT=$(ls -dt $EXP/runs/*_$TAG 2>/dev/null | head -1)
  if [ -n "$OUT" ] && [[ "$(basename "$OUT")" < "$START" ]]; then OUT=""; fi   # 이번 실행 전에 만들어진 폴더는 무시
  [ -n "$OUT" ] && ls $OUT/race_result_*.json >/dev/null 2>&1 && break
  sleep 2
done
[ -z "$OUT" ] && { echo "simulation did not start, see $EXP/tuning/${STUDY}_sim.log"; exit 1; }
echo "sim running: $OUT"

# 3) 학습 (optuna 는 svg_mppi_exp/pylib 에 설치되어 있음)
PYTHONPATH=$EXP/pylib:$PYTHONPATH python3 $EXP/tools/tune.py --space $space --log-dir $OUT \
  --study $STUDY --trials $trials --laps $laps 2>&1 | tee $EXP/tuning/${STUDY}_tune.log
echo "done. best table: $EXP/tuning/${STUDY}_best_schedule.yaml"
