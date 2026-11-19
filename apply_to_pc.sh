#!/bin/bash
# 저장소 내용을 이 PC(수업 컨테이너)에 적용한다. 처음 설치와 업데이트 모두 이 스크립트 하나로.
#
#   1) ~/shared_dir/svg_mppi_exp 의 코드 폴더(tools, configs, analysis, report, pylib)와 안내서를 저장소 것으로 갱신
#      -> runs/, tuning/ 같은 실험·학습 결과는 건드리지 않음
#   2) ~/shared_dir/proj-svg_mppi (공식 코드) 갱신
#   3) ~/forza_ws/race_stack/svg_mppi 를 저장소 것으로 교체 (기존 것은 백업) 후 빌드
#
# 사용 (컨테이너 터미널, misys@):
#   처음:     cd ~/shared_dir && git clone https://github.com/qazwsx1121/2026-ADP-Project.git
#   업데이트: cd ~/shared_dir/2026-ADP-Project && git pull
#   적용:     bash ~/shared_dir/2026-ADP-Project/apply_to_pc.sh            (빌드까지)
#             bash ~/shared_dir/2026-ADP-Project/apply_to_pc.sh --no-build (파일만)
# 경로를 바꾸려면: SHARED=<shared_dir 경로> WS=<race_stack 경로> bash apply_to_pc.sh
set -e
REPO=$(cd "$(dirname "$0")" && pwd)
SHARED=${SHARED:-$HOME/shared_dir}
WS=${WS:-$HOME/forza_ws/race_stack}
BUILD=1; [ "$1" = "--no-build" ] && BUILD=0
STAMP=$(date +%Y%m%d_%H%M%S)

echo "저장소: $REPO ($(git -C "$REPO" log --oneline -1 2>/dev/null || echo 'git 정보 없음'))"
echo "적용 대상: $SHARED, $WS"

# 0) 학습·실험 중이면 중단: 실행 중인 bash 스크립트 파일을 덮어쓰면 그 실행이 깨진다
if ps -eo args | grep -qE "[t]rain\.sh|[r]un_exp\.sh|[n]oise_sweep\.sh|[v]alidate\.sh|[b]atch2?\.sh"; then
  echo "학습이나 실험이 돌고 있습니다. 끝난 뒤에 다시 실행하세요:"
  ps -eo args | grep -E "[t]rain\.sh|[r]un_exp\.sh|[n]oise_sweep\.sh|[v]alidate\.sh|[b]atch2?\.sh" | head -3
  exit 1
fi
[ -d "$WS/src" ] || [ -d "$WS/stack_master" ] || { echo "race_stack 을 찾을 수 없습니다: $WS"; exit 1; }

same() { [ "$(realpath -m "$1")" = "$(realpath -m "$2")" ]; }

# 실행 권한(chmod +x) 변경을 git 이 "수정됨"으로 보지 않게: 안 그러면 다음 git pull 이 막힐 수 있다
git -C "$REPO" rev-parse --git-dir >/dev/null 2>&1 && git -C "$REPO" config core.fileMode false

# 1) svg_mppi_exp: 코드 폴더만 (결과 폴더 runs/ tuning/ 등은 그대로)
mkdir -p "$SHARED"
if same "$REPO/svg_mppi_exp" "$SHARED/svg_mppi_exp"; then
  echo "[1/3] svg_mppi_exp 는 저장소 폴더 그대로 사용 (링크) — 복사 생략"
else
  mkdir -p "$SHARED/svg_mppi_exp"
  for d in tools configs analysis report pylib; do
    [ -d "$REPO/svg_mppi_exp/$d" ] || continue
    mkdir -p "$SHARED/svg_mppi_exp/$d"
    cp -r "$REPO/svg_mppi_exp/$d/." "$SHARED/svg_mppi_exp/$d/"
  done
  cp "$REPO"/svg_mppi_exp/*.md "$SHARED/svg_mppi_exp/" 2>/dev/null || true
  echo "[1/3] $SHARED/svg_mppi_exp 의 tools/configs/analysis/report/pylib 갱신 (runs, tuning 은 그대로)"
fi
chmod +x "$SHARED"/svg_mppi_exp/tools/*.sh 2>/dev/null || true

# 2) 공식 코드
if same "$REPO/proj-svg_mppi" "$SHARED/proj-svg_mppi"; then
  echo "[2/3] proj-svg_mppi 는 저장소 폴더 그대로 사용 — 복사 생략"
else
  mkdir -p "$SHARED/proj-svg_mppi"
  cp -r "$REPO/proj-svg_mppi/." "$SHARED/proj-svg_mppi/"
  echo "[2/3] $SHARED/proj-svg_mppi 갱신"
fi

# 3) race_stack 의 svg_mppi 교체 (기존 것은 백업) + 빌드
if [ -d "$WS/svg_mppi" ]; then
  mkdir -p "$SHARED/svg_mppi_exp/backup"
  mv "$WS/svg_mppi" "$SHARED/svg_mppi_exp/backup/race_stack_svg_mppi_$STAMP"
  echo "      기존 $WS/svg_mppi -> $SHARED/svg_mppi_exp/backup/race_stack_svg_mppi_$STAMP"
fi
cp -r "$REPO/svg_mppi" "$WS/svg_mppi"
echo "[3/3] $WS/svg_mppi 교체"

if [ $BUILD = 1 ]; then
  set +e
  source /opt/ros/humble/setup.bash
  [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
  cd "$WS" && colcon build --packages-select svg_mppi --cmake-args -DSVG_MPPI_CORE_DIR="$SHARED/proj-svg_mppi/src/mppi_controller" 2>&1 | tail -3
  [ ${PIPESTATUS[0]} -eq 0 ] || { echo "빌드 실패: 위 메시지를 확인하세요"; exit 1; }
  echo "빌드 완료. 새 터미널에서: source ~/forza_ws/race_stack/install/setup.bash"
fi
echo "적용 끝."
