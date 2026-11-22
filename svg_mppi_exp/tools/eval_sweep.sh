#!/bin/bash
# 최종 평가 방식 실험: 구성 x 장애물 1/3/5 개 x seed. 자동 복구 없이, 첫 충돌 / 10바퀴 / 3분 중 먼저 오는 것에서 끝낸다.
#
# 사용 (컨테이너 터미널, misys@):
#   bash ~/shared_dir/svg_mppi_exp/tools/eval_sweep.sh [key=value ...]
# key:
#   cfgs="C G"            구성 (gen_cfg.py 참고: A, C, G, CF, GF)
#   obs="1 3 5"           장애물 수 (평가와 같음)
#   seeds="2026 11 7 42 5"
#   dur=180 maxlaps=10    평가: 최대 3분 또는 10바퀴
#   curv=0.65             G/GF 의 guide.max_curvature [1/m]
#   base=<yaml>           구성을 만들 기본 설정 (기본: 패키지 svg_mppi_light.yaml, 학교 PC: school_*.yaml)
#   pose=pf               pf(기본, 평가와 같음) | gt
#   tag=ev                결과 폴더 이름 앞부분
#   noise="..."           run_exp.sh 노이즈 옵션 그대로 (예: "snoise=0.03 sdrop=0.02 sghost=0.005")
# 실행 1회 = 최대 3분 + 준비 약 35초. 기본 2 x 3 x 5 = 30회 ≈ 최대 1시간 50분 (충돌하면 더 빨리 끝남)
# 결과: runs/*_<tag><구성>_o<장애물 수>_s<seed>, 마지막에 eval_summary.py 표

cfgs="C G"; obs="1 3 5"; seeds="2026 11 7 42 5"; dur=180; maxlaps=10; curv=0.65; base=""; pose=pf; tag=ev; noise=""
for kv in "$@"; do printf -v "${kv%%=*}" '%s' "${kv#*=}"; done
EXP=~/shared_dir/svg_mppi_exp
source /opt/ros/humble/setup.bash
source ~/forza_ws/race_stack/install/setup.bash
[ -z "$base" ] && base=$(ros2 pkg prefix svg_mppi)/share/svg_mppi/config/svg_mppi_light.yaml
GEN=$EXP/configs/generated; mkdir -p $GEN
b=$(basename $base .yaml)
for c in $cfgs; do
  python3 $EXP/tools/gen_cfg.py $base $c $GEN/${c}_${b}_k${curv}.yaml --curv $curv || exit 1
done
total=$(( $(echo $cfgs | wc -w) * $(echo $obs | wc -w) * $(echo $seeds | wc -w) )); n=0
for s in $seeds; do
  for c in $cfgs; do
    for o in $obs; do
      n=$((n + 1)); g=${tag}${c}_o${o}
      echo "[$n/$total] $g seed $s ($(date +%H:%M))"
      bash $EXP/tools/run_exp.sh ${g}_s$s eval=1 dur=$dur maxlaps=$maxlaps obs=$o seed=$s nseed=$s pose=$pose \
        params=$GEN/${c}_${b}_k${curv}.yaml $noise | grep -E "stopped:|laps=|ERROR"
    done
  done
done
echo; echo "=== 구성별 (장애물 1/3/5: 평균 연속 랩 / 10바퀴 완주 run / 연속 구간 최고 랩 평균) ==="
python3 $EXP/analysis/eval_summary.py $(for c in $cfgs; do echo ${tag}${c}; done)
