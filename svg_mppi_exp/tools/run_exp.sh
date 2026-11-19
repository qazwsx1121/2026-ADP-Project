#!/bin/bash
# 실험 1회 실행 (컨테이너 안에서): 시뮬 + svg_mppi + race_manager + 기록/복구 -> DUR 초 후 종료 -> 전광판 PNG
#
# 사용 (WSL에서):
#   docker exec exciting_rhodes bash /home/misys/shared_dir/svg_mppi_exp/tools/run_exp.sh <tag> [key=value ...]
# key:
#   dur=120            주행 시간 [s] (svg_mppi 가 /drive 를 내기 시작한 뒤부터)
#   params=<yaml>      svg_mppi 파라미터 파일 (기본: 패키지 svg_mppi_light.yaml)
#   mode=svg_mppi      svg_mppi | forward_mppi
#   obs=0 seed=2026    race_manager 장애물 수 / seed
#   gui=0              1이면 RViz + 전광판 창을 띄움 (WSLg)
#   recover=1          충돌/정지 시 자동 복구
#   set="a:=1 b:=2"    시작 후 ros2 param set /svg_mppi 로 바꿀 값 (공백 구분, 이름:=값)
#   sched=<yaml>       param_scheduler.py 표 (구간/장애물별 파라미터 자동 변경)
#   debug=1            조향이 뒤집히는 순간 solver 상태를 OUT/flip_*.json 으로 저장
#   [장애물 노이즈] onoise=0.10 위치 σ[m]  orot=1 회전 무작위  odyn=0.02 매초 추가로 흔듦[m]  nseed=0
#   [라이다 노이즈] snoise=0.03 거리 σ[m]  sdrop=0.02 빔 누락 확률  sghost=0.005 허상 점 확률 (컨트롤러만 봄)
# 결과: ~/shared_dir/svg_mppi_exp/runs/<날짜_시간>_<tag>/  (race_result_*.json/png, trace.csv, events.csv, *.log)

TAG=${1:-run}; shift
dur=120; params=""; mode=svg_mppi; obs=0; seed=2026; gui=0; recover=1; set=""; sched=""; debug=0
onoise=0; orot=0; odyn=0; nseed=0; snoise=0; sdrop=0; sghost=0
for kv in "$@"; do printf -v "${kv%%=*}" '%s' "${kv#*=}"; done   # key=value (값에 공백이 있어도 됨)

source /opt/ros/humble/setup.bash
source ~/forza_ws/race_stack/install/setup.bash
[ -z "$params" ] && params=$(ros2 pkg prefix svg_mppi)/share/svg_mppi/config/svg_mppi_light.yaml
EXP=~/shared_dir/svg_mppi_exp
OUT=$EXP/runs/$(date +%Y%m%d_%H%M%S)_$TAG
mkdir -p "$OUT"
WPNTS=~/forza_ws/race_stack/stack_master/maps/berlin/global_waypoints.json
echo "tag=$TAG dur=$dur params=$params mode=$mode obs=$obs seed=$seed gui=$gui recover=$recover set=$set sched=$sched debug=$debug onoise=$onoise orot=$orot odyn=$odyn nseed=$nseed snoise=$snoise sdrop=$sdrop sghost=$sghost" | tee "$OUT/config.txt"
cp "$params" "$OUT/params.yaml"
if [ "$debug" = 1 ]; then printf '    debug:\n      dump_dir: "%s"\n' "$OUT" >> "$OUT/params.yaml"; fi
[ -n "$sched" ] && cp "$sched" "$OUT/schedule.yaml"
nz() { python3 -c "import sys; sys.exit(0 if float('$1') != 0 else 1)"; }
SCAN_NOISE=0; OBS_NOISE=0
{ nz $snoise || nz $sdrop || nz $sghost; } && SCAN_NOISE=1
{ nz $onoise || nz $orot || nz $odyn; } && OBS_NOISE=1
if [ $SCAN_NOISE = 1 ]; then  # controller sees /scan_noisy, the simulator keeps the true scan
  sed -i 's#^\(\s*scan_topic:\s*\)/scan\b#\1/scan_noisy#' "$OUT/params.yaml"
  grep -q "scan_topic: /scan_noisy" "$OUT/params.yaml" || { echo "could not switch scan_topic"; exit 1; }
fi

cleanup() {
  for p in $PIDS; do kill -INT -- -$p 2>/dev/null; done
  sleep 4
  for p in $PIDS; do kill -TERM -- -$p 2>/dev/null; done
  sleep 2
  for p in $PIDS; do kill -KILL -- -$p 2>/dev/null; done
  pkill -f rviz2 2>/dev/null; pkill -f "race_manager" 2>/dev/null; pkill -f exp_monitor.py 2>/dev/null; pkill -f param_scheduler.py 2>/dev/null
  pkill -f obstacle_noise.py 2>/dev/null; pkill -f scan_noise.py 2>/dev/null
}
trap cleanup EXIT
PIDS=""
start() {  # start <name> <cmd...> : new process group so it can be stopped as a whole
  local name=$1; shift
  setsid "$@" > "$OUT/$name.log" 2>&1 &
  PIDS="$PIDS $!"
}

# 이전 실행 잔여 프로세스 정리
pkill -f "ros2 launch" 2>/dev/null; pkill -f rviz2 2>/dev/null; pkill -f gym_bridge 2>/dev/null; pkill -f svg_mppi_node 2>/dev/null
pkill -f exp_monitor.py 2>/dev/null; pkill -f race_manager 2>/dev/null; pkill -f param_scheduler.py 2>/dev/null
pkill -f obstacle_noise.py 2>/dev/null; pkill -f scan_noise.py 2>/dev/null; sleep 2

start base ros2 launch stack_master base_system_launch.xml map_name:=berlin pose_source:=gt
if [ "$gui" != "1" ]; then
  for i in $(seq 1 20); do pgrep -f "rviz2/rviz2" >/dev/null && break; sleep 0.5; done
  sleep 1; pkill -f "rviz2/rviz2"   # GUI 없이: RViz만 끔 (나머지 노드는 그대로)
fi
# 시뮬 준비 대기
for i in $(seq 1 60); do ros2 topic list 2>/dev/null | grep -q /global_waypoints_scaled && break; sleep 1; done
sleep 3

start monitor python3 $EXP/tools/exp_monitor.py --ros-args -p out_dir:=$OUT -p waypoints_file:=$WPNTS -p recover:=$([ "$recover" = 1 ] && echo true || echo false)
[ $SCAN_NOISE = 1 ] && start scannoise python3 $EXP/tools/scan_noise.py --ros-args -p sigma:=$snoise -p drop:=$sdrop -p ghost:=$sghost -p seed:=$nseed
start mppi ros2 launch svg_mppi svg_mppi_launch.xml mpc_mode:=$mode params:=$OUT/params.yaml
# MPPI 가 주행을 시작할 때까지 대기
for i in $(seq 1 60); do grep -q "SVG-MPPI ready" "$OUT/mppi.log" && break; sleep 1; done
for kv in $set; do ros2 param set /svg_mppi "${kv%%:=*}" "${kv#*:=}" >> "$OUT/param_set.log" 2>&1; done
[ -n "$sched" ] && start sched python3 $EXP/tools/param_scheduler.py --ros-args -p table:=$OUT/schedule.yaml
if [ $OBS_NOISE = 1 ]; then
  # spawner -> /sim/obstacles_raw -> obstacle_noise.py -> /sim/obstacles (simulator); scorer started last (RACE_PID)
  CFG=$(ros2 pkg prefix race_manager)/share/race_manager/config/race_manager_params.yaml
  start onoise python3 $EXP/tools/obstacle_noise.py --ros-args -p pos_sigma:=$onoise -p rot:=$([ "$orot" = 1 ] && echo true || echo false) \
    -p dynamic_sigma:=$odyn -p seed:=$nseed
  start spawner ros2 run race_manager obstacle_spawner --ros-args --params-file $CFG -p waypoints_file:=$WPNTS \
    -p num_obstacles:=$obs -p seed:=$seed -r /sim/obstacles:=/sim/obstacles_raw -r /sim/obstacles/markers:=/sim/obstacles_raw/markers
  start race ros2 run race_manager scorer --ros-args --params-file $CFG -p map_name:=berlin -p waypoints_file:=$WPNTS \
    -p seed:=$seed -p log_dir:=$OUT
  for i in $(seq 1 30); do grep -q "new layout" "$OUT/onoise.log" && break; sleep 0.5; done
  grep -q "new layout" "$OUT/onoise.log" || { echo "ERROR: obstacle_noise did not publish obstacles (see $OUT/onoise.log)"; exit 1; }
else
  start race ros2 launch race_manager race_manager_launch.xml map_name:=berlin seed:=$seed num_obstacles:=$obs \
    log_dir:=$OUT plot:=$([ "$gui" = 1 ] && echo true || echo false)
fi

echo "running $dur s ... ($OUT)"
sleep "$dur"

# race_manager 를 먼저 Ctrl-C 로 끝내 요약을 남긴다
for p in $PIDS; do :; done
RACE_PID=$(echo $PIDS | awk '{print $NF}')
kill -INT -- -$RACE_PID 2>/dev/null; sleep 3
cleanup; trap - EXIT

J=$(ls -t "$OUT"/race_result_*.json 2>/dev/null | head -1)
if [ -n "$J" ]; then
  ros2 run race_manager report --no-show "$J" > "$OUT/report.log" 2>&1
  python3 - "$J" <<'EOF'
import json, sys
r = json.load(open(sys.argv[1]))
laps = r["laps"]
print("laps=%d best=%s total_hits=%d (obs %d, wall %d) teleports=%d" % (
    r["laps_completed"], r["best_lap"], r["collisions"]["total"], r["collisions"]["obstacle"],
    r["collisions"]["wall"], len(r.get("teleports", []))))
print("lap times:", " ".join("%.2f%s" % (l["time"], "*" if l["total"] or l["partial"] else "") for l in laps))
EOF
fi
grep -c "control loop" "$OUT/mppi.log" | xargs echo "overrun warnings:"
echo "OUT=$OUT"
