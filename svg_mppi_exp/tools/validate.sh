#!/bin/bash
# 학습한 표를 학습에 안 쓴 seed 들로 검증하고, 표 없이(기본 설정) 돌린 결과와 비교한다.
#
# 사용 (컨테이너 터미널, misys@):
#   bash ~/shared_dir/svg_mppi_exp/tools/validate.sh <표.yaml> [key=value ...]
# key:
#   seeds="2026 11 7 42 5"   검증 seed (학습 seed 와 겹치지 않게)
#   dur=240                   seed 하나당 주행 시간 [s]
#   params=<yaml>             svg_mppi 설정 (기본: 패키지 svg_mppi_light.yaml)
#   base=1                    1이면 표 없이 같은 seed 도 돌려 비교 (시간 2배)
#   tag=v                     결과 폴더 이름 앞부분
#   pose=gt            gt | pf (최종 평가는 pf 권장, 교수님 답변 11/20)
#   noise="onoise=0.10 orot=1 ..."  노이즈를 켜고 검증 (run_exp.sh 옵션 그대로, 두 조건 모두 같은 노이즈)
# 결과: runs/*_<tag>T_s<seed> (표 적용), runs/*_<tag>B_s<seed> (기준), 마지막에 합계 표 출력

TABLE=${1:?usage: validate.sh <schedule.yaml> [seeds=... dur=240 params=... base=1 tag=v]}; shift
seeds="2026 11 7 42 5"; dur=240; params=""; base=1; tag=v; noise=""; pose=gt
for kv in "$@"; do printf -v "${kv%%=*}" '%s' "${kv#*=}"; done   # key=value (값에 공백이 있어도 됨)
EXP=~/shared_dir/svg_mppi_exp
source /opt/ros/humble/setup.bash
source ~/forza_ws/race_stack/install/setup.bash
[ -z "$params" ] && params=$(ros2 pkg prefix svg_mppi)/share/svg_mppi/config/svg_mppi_light.yaml
for s in $seeds; do
  bash $EXP/tools/run_exp.sh ${tag}T_s$s dur=$dur obs=4 seed=$s params=$params sched=$TABLE nseed=$s pose=$pose $noise | tail -4
  [ "$base" = 1 ] && bash $EXP/tools/run_exp.sh ${tag}B_s$s dur=$dur obs=4 seed=$s params=$params nseed=$s pose=$pose $noise | tail -4
done
echo; echo "=== T = 학습한 표 적용, B = 기준 ==="
python3 $EXP/analysis/group_summary.py ${tag}T ${tag}B
