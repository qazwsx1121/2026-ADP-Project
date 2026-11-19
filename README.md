# 2026-ADP-Project

숭실대 2026-2학기 「자율주행 프로그래밍」 8조 팀 프로젝트 저장소입니다.
ForzaETH race stack(ROS 2 Humble)의 PP/TLN 컨트롤러를 **SVG-MPPI**(Honda et al., ICRA 2024)로 교체하고, 상황별 파라미터 학습을 붙여 기말 평가 맵 **berlin**에서 주행합니다.

```
/scan  /car_state/odom  /global_waypoints_scaled  →  svg_mppi_node  →  /drive
                                      ↑
               param_scheduler (학습된 구간별 파라미터를 주행 중 변경)
```

- 컨트롤러는 SVG-MPPI 하나만 씁니다. 학습 결과는 `/drive`를 직접 내지 않고, MPPI의 파라미터만 바꿉니다.
- 공식 SVG-MPPI 알고리즘 코드는 **수정하지 않고** 그대로 컴파일합니다. 우리가 만든 것은 ROS 2 연결 노드와 실험·학습 도구입니다.

## 저장소 구조

| 경로 | 내용 |
|---|---|
| [`svg_mppi/`](svg_mppi/) | ROS 2 연결 노드 패키지 (공식 ROS 1 노드 3개를 노드 1개로 이식, 주행 중 파라미터 변경, 연속 장애물용 가이드 기준 경로 등). 자세한 내용은 [`svg_mppi/README.md`](svg_mppi/README.md) |
| [`proj-svg_mppi/`](proj-svg_mppi/) | 공식 SVG-MPPI 코드 ([kohonda/proj-svg_mppi](https://github.com/kohonda/proj-svg_mppi), MIT, 수정 없음) |
| [`svg_mppi_exp/tools/`](svg_mppi_exp/tools/) | 실험 실행(`run_exp.sh`, `batch*.sh`), 학습(`train.sh`, `tune.py`), 검증(`validate.sh`), 주행 중 파라미터 적용(`param_scheduler.py`) |
| [`svg_mppi_exp/configs/`](svg_mppi_exp/configs/) | 실험별 파라미터 파일, 학습 탐색 범위(`tune_space.yaml`) |
| [`svg_mppi_exp/analysis/`](svg_mppi_exp/analysis/) | 결과 요약·그래프 스크립트 |
| [`svg_mppi_exp/report/`](svg_mppi_exp/report/) | 실험 보고서 (`index.html`, 그림) |
| `svg_mppi_exp/pylib/` | Optuna 등 파이썬 라이브러리 (컨테이너에 pip 없이 쓰기 위해 포함) |
| [`학교PC_시작하기.md`](학교PC_시작하기.md) | 학교 PC에서 세팅 → 학습 → 검증까지 단계별 안내 |

## 시작하기

수업용 Docker 컨테이너(이미지 `misys:forza_full`) 안에서 실행합니다. 도구들이 `~/shared_dir` 아래 경로를 쓰므로 그 위치에 맞춰 둡니다.

```bash
# [컨테이너]
cd ~/shared_dir && git clone https://github.com/qazwsx1121/2026-ADP-Project.git
ln -s ~/shared_dir/2026-ADP-Project/proj-svg_mppi ~/shared_dir/proj-svg_mppi
ln -s ~/shared_dir/2026-ADP-Project/svg_mppi_exp  ~/shared_dir/svg_mppi_exp
chmod +x ~/shared_dir/svg_mppi_exp/tools/*.sh

# 빌드 (svg_mppi는 반드시 race_stack 바로 아래)
source /opt/ros/humble/setup.bash
source ~/forza_ws/race_stack/install/setup.bash
cp -r ~/shared_dir/2026-ADP-Project/svg_mppi ~/forza_ws/race_stack/
cd ~/forza_ws/race_stack && colcon build --packages-select svg_mppi && source install/setup.bash
```

- 이미 `~/shared_dir/proj-svg_mppi`나 `svg_mppi_exp` 폴더가 있으면, 링크를 만들기 전에 이름을 바꾸거나 옮겨 두세요.
- 새 터미널마다 위의 `source` 두 줄이 필요합니다. `~/.bashrc`에 추가해 두면 편합니다.

### 주행 (터미널 3개)

```bash
ros2 launch stack_master base_system_launch.xml map_name:=berlin pose_source:=gt        # T1 시뮬
ros2 launch svg_mppi svg_mppi_launch.xml                                                 # T2 SVG-MPPI
ros2 launch race_manager race_manager_launch.xml map_name:=berlin seed:=2026 num_obstacles:=0   # T3
```

- CPU가 부족하면(계산 시간 25 ms 초과) T2에 가벼운 설정을 씁니다:
  `params:=$(ros2 pkg prefix svg_mppi)/share/svg_mppi/config/svg_mppi_light.yaml`
- 주행 중 파라미터 변경: `ros2 param set /svg_mppi <이름> <값>` (예: `reference_sdf.reference_speed_scale 1.2`, `svg_mppi.lambda 1.0`, `guide.enable true`)

### 실험·학습

```bash
C=~/shared_dir/svg_mppi_exp/configs
bash ~/shared_dir/svg_mppi_exp/tools/run_exp.sh check dur=60 obs=0 params=$C/school_light_t8.yaml     # 1회 실험
bash ~/shared_dir/svg_mppi_exp/tools/train.sh berlin_v1 trials=60 laps=4 seed=100 params=$C/<설정>.yaml  # 학습
```

설정 고르는 법, 학습·검증 절차, 문제 해결은 [`학교PC_시작하기.md`](학교PC_시작하기.md)에 정리되어 있습니다.

## 주요 결과 요약

- **λ 3.0 → 1.0**: berlin 왼쪽 위 S자 헤어핀에서 막다른 공간 진입이 장애물 0 기준 39랩 중 13회 → 44랩 중 0회.
- **충돌 지도 부풀림 rigid → circle 0.15 m**: 장애물 4개, seed 5개 각 240 s 기준 28랩·충돌 34회 → 65랩·충돌 11회.
- 상세 근거는 [`svg_mppi/README.md`](svg_mppi/README.md)와 [`svg_mppi_exp/report/`](svg_mppi_exp/report/)를 참고하세요.

## 일정

- 최종 평가: **2026-12-16 13:00–18:00, 문화관 408호**, 팀당 최대 30분 (직접 세팅 후 실시간 파라미터 조정 시연)
- 맵: berlin / seed 비공개 (랜덤으로 간주하고 여러 seed로 검증)

## 팀

8조 — 지민혁, 이해담, 국소이, 서병수

## 참고

- Honda et al., "Stein Variational Guided Model Predictive Path Integral Control: Proposal and Experiments with Fast Maneuvering Vehicles", ICRA 2024 ([arXiv:2309.11040](https://arxiv.org/abs/2309.11040))
- 공식 코드: https://github.com/kohonda/proj-svg_mppi
- ForzaETH race stack: https://github.com/ForzaETH/race_stack
