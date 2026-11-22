#!/bin/bash
# 노이즈 강건성 실험: 설정 x 노이즈 종류 x seed 를 모두 돌리고 조건별 합계를 출력한다.
#
# 사용 (컨테이너 터미널, misys@):
#   bash ~/shared_dir/svg_mppi_exp/tools/noise_sweep.sh [key=value ...]
# key:
#   cfgs="G GF"                구성 (gen_cfg.py: A, C, G, CF, GF)
#   levels="N2 N3 N4"          노이즈 (아래 표)
#   seeds="2026 11 7 42 5"
#   dur=240                    실행 1회 주행 시간 [s]
#   tag=ns                     결과 폴더 이름 앞부분
#   curv=0.65                  G/GF 의 guide.max_curvature [1/m]
#   base=<yaml>                구성을 만들 기본 설정 (학교 PC: 5번에서 고른 school_*.yaml, 기본: 패키지 light)
#   pose=pf                    pf(기본) | gt
# 예상 시간 = cfgs 수 x levels 수 x seeds 수 x 4.5 분  (기본 2 x 3 x 5 = 30회 ≈ 2시간 15분)
#
# 구성:  C  = 원형 0.15, guide 끔 (패키지 기본값)
#        G  = C + guide 켬, 레이스라인 곡률이 curv 보다 크면 guide 쉼 (트랙 구간·좌표별 설정 없음)
#        CF / GF = C / G + 라이다 외톨이 점 제거 필터
# 노이즈: N0 = 없음
#        N2 = 장애물 위치 σ 0.10 m + 회전 무작위 (배치마다)
#        N3 = 라이다 거리 σ 0.03 m + 빔 누락 2% + 허상 점 0.5% (컨트롤러가 보는 스캔에만)
#        N4 = N2 + N3
# 결과 폴더: runs/*_<tag><설정><노이즈>_s<seed>, 마지막에 group_summary 표

cfgs="G GF"; levels="N2 N3 N4"; seeds="2026 11 7 42 5"; dur=240; tag=ns; base=""; pose=pf; curv=0.65
for kv in "$@"; do printf -v "${kv%%=*}" '%s' "${kv#*=}"; done
EXP=~/shared_dir/svg_mppi_exp
C=$EXP/configs
source /opt/ros/humble/setup.bash
source ~/forza_ws/race_stack/install/setup.bash
[ -z "$base" ] && base=$(ros2 pkg prefix svg_mppi)/share/svg_mppi/config/svg_mppi_light.yaml
GEN=$EXP/configs/generated; mkdir -p $GEN
b=$(basename $base .yaml)
for c in $cfgs; do python3 $EXP/tools/gen_cfg.py $base $c $GEN/${c}_${b}_k${curv}.yaml --curv $curv || exit 1; done
cfg_args() { echo "params=$GEN/${1}_${b}_k${curv}.yaml"; }
noise_args() {
  case $1 in
    N0) echo "" ;;
    N2) echo "onoise=0.10 orot=1" ;;
    N3) echo "snoise=0.03 sdrop=0.02 sghost=0.005" ;;
    N4) echo "onoise=0.10 orot=1 snoise=0.03 sdrop=0.02 sghost=0.005" ;;
    *)  echo "unknown level $1" >&2; exit 1 ;;
  esac
}
groups=""
total=$(( $(echo $cfgs | wc -w) * $(echo $levels | wc -w) * $(echo $seeds | wc -w) )); n=0
for s in $seeds; do
  for c in $cfgs; do
    for l in $levels; do
      n=$((n + 1)); g=${tag}${c}${l}
      echo "[$n/$total] $g seed $s  ($(date +%H:%M))"
      bash $EXP/tools/run_exp.sh ${g}_s$s dur=$dur obs=4 seed=$s nseed=$s pose=$pose $(cfg_args $c) $(noise_args $l) | grep -E "laps=|ERROR"
    done
  done
done
for c in $cfgs; do for l in $levels; do groups="$groups ${tag}${c}${l}"; done; done
echo; echo "=== 조건별 합계 (랩 / 클린 랩 / 장애물 충돌 / 벽 충돌 / 복구) ==="
python3 $EXP/analysis/group_summary.py $groups
