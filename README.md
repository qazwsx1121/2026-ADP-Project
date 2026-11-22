# 2026-ADP-Project

숭실대 2026-2학기 「자율주행 프로그래밍」 8조 팀 프로젝트 저장소입니다.
ForzaETH race stack(ROS 2 Humble)의 PP/TLN 컨트롤러를 **SVG-MPPI**(Honda et al., ICRA 2024)로 교체하고, 상황별 파라미터 학습을 붙여 기말 평가 맵 **berlin**에서 주행합니다.

```
/scan  /car_state/odom  /global_waypoints_scaled  →  svg_mppi_node  →  /drive
                                      ↑
               param_scheduler (학습된 파라미터를 주행 중 적용, 트랙 구간·좌표별 값 없음)
```

- 컨트롤러는 SVG-MPPI 하나만 씁니다. 학습 결과는 `/drive`를 직접 내지 않고, MPPI의 파라미터만 바꿉니다.
- 공식 SVG-MPPI 알고리즘 코드는 **수정하지 않고** 그대로 컴파일합니다. 우리가 만든 것은 ROS 2 연결 노드와 실험·학습 도구입니다.

## 저장소 구조

| 경로 | 내용 |
|---|---|
| [`svg_mppi/`](svg_mppi/) | ROS 2 연결 노드 패키지 (공식 ROS 1 노드 3개를 노드 1개로 이식, 주행 중 파라미터 변경, 연속 장애물용 가이드 기준 경로 등). 자세한 내용은 [`svg_mppi/README.md`](svg_mppi/README.md) |
| [`proj-svg_mppi/`](proj-svg_mppi/) | 공식 SVG-MPPI 코드 ([kohonda/proj-svg_mppi](https://github.com/kohonda/proj-svg_mppi), MIT, 수정 없음) |
| [`svg_mppi_exp/tools/`](svg_mppi_exp/tools/) | 실험 실행(`run_exp.sh`, `batch*.sh`), 학습(`train.sh`, `tune.py`), 검증(`validate.sh`), 주행 중 파라미터 적용(`param_scheduler.py`), 노이즈 실험(`obstacle_noise.py`, `scan_noise.py`, `noise_sweep.sh`) |
| [`svg_mppi_exp/configs/`](svg_mppi_exp/configs/) | 실험별 파라미터 파일, 학습 탐색 범위(`tune_space.yaml`) |
| [`svg_mppi_exp/analysis/`](svg_mppi_exp/analysis/) | 결과 요약·그래프 스크립트 |
| [`svg_mppi_exp/report/`](svg_mppi_exp/report/) | 실험 보고서 (`index.html`, 그림) |
| `svg_mppi_exp/pylib/` | Optuna 등 파이썬 라이브러리 (컨테이너에 pip 없이 쓰기 위해 포함) |
| [`apply_to_pc.sh`](apply_to_pc.sh) | 저장소 내용을 이 PC(컨테이너)에 적용: 코드 갱신 + 빌드, 실험·학습 결과는 보존 |
| [`학교PC_시작하기.md`](학교PC_시작하기.md) | 학교 PC에서 세팅 → 학습 → 검증까지 단계별 안내 |
| [`학교PC_노이즈실험.md`](학교PC_노이즈실험.md) | 평가 때 장애물 노이즈 대비: 노이즈 실험, 노이즈를 켠 학습·검증 |

## 시작하기

수업용 Docker 컨테이너(이미지 `misys:forza_full`) 안에서 실행합니다. 호스트가 우분투든 WSL이든, 실행은 컨테이너 안에서 하므로 같습니다. 도구들은 컨테이너의 `~/shared_dir`(호스트와 공유되는 폴더)과 `~/forza_ws/race_stack`을 씁니다.

```bash
# [컨테이너] 처음 한 번
cd ~/shared_dir && git clone https://github.com/qazwsx1121/2026-ADP-Project.git
bash ~/shared_dir/2026-ADP-Project/apply_to_pc.sh

# [컨테이너] 저장소가 바뀌었을 때
cd ~/shared_dir/2026-ADP-Project && git pull
bash ~/shared_dir/2026-ADP-Project/apply_to_pc.sh
```

`apply_to_pc.sh`가 하는 일:
1. `~/shared_dir/svg_mppi_exp`의 코드 폴더(`tools`, `configs`, `analysis`, `report`, `pylib`)와 안내서를 저장소 것으로 갱신합니다. **`runs/`, `tuning/` 같은 실험·학습 결과는 건드리지 않습니다.**
2. `~/shared_dir/proj-svg_mppi`(공식 코드)를 갱신합니다.
3. `~/forza_ws/race_stack/svg_mppi`를 저장소 것으로 바꾸고(기존 것은 `svg_mppi_exp/backup/`에 보관) 빌드합니다.

- 학습이나 실험이 돌고 있으면 아무것도 바꾸지 않고 멈춥니다(실행 중인 스크립트를 덮어쓰면 그 실행이 깨지기 때문).
- `git clone`은 호스트에서 해도 됩니다. 호스트의 공유 폴더(우분투 `dkrun.sh` 기준 `~/shared_dir`)에 받으면 컨테이너에서도 보입니다.
- 새 터미널마다 `source /opt/ros/humble/setup.bash`, `source ~/forza_ws/race_stack/install/setup.bash`가 필요합니다. `~/.bashrc`에 추가해 두면 편합니다.

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

실험 도구는 차량 위치를 **`pose_source:=pf`(파티클 필터)로 기본 실행**합니다(교수님 답변 11/20: gt는 정성 평가 감점, `/map` 사용은 GT 아님). 예전 결과와 비교할 때만 `pose=gt`를 줍니다. 기록 노드가 pf 위치 오차(GT 대비)를 `trace.csv`에 남기고, `summarize.py`/`group_summary.py`가 `loc_*` 열로 보여 줍니다. 11/20 이전 결과는 모두 gt 기준입니다.

노이즈를 켠 실험은 `run_exp.sh`에 옵션을 더하거나(`onoise=0.10 orot=1` 장애물 위치·회전, `snoise=0.03 sdrop=0.02 sghost=0.005` 라이다) `noise_sweep.sh`로 한 번에 돌립니다. 자세한 내용은 [`학교PC_노이즈실험.md`](학교PC_노이즈실험.md)에 있습니다.

```bash
bash ~/shared_dir/svg_mppi_exp/tools/noise_sweep.sh base=$C/<설정>.yaml        # 노이즈 강건성 실험 (약 2시간 15분)
bash ~/shared_dir/svg_mppi_exp/tools/train.sh berlin_noise params=$C/<설정>.yaml noise="onoise=0.10 orot=1 snoise=0.03 sdrop=0.02 sghost=0.005"
```

## 주요 결과 요약

- **λ 3.0 → 1.0**: berlin 왼쪽 위 S자 헤어핀에서 막다른 공간 진입이 장애물 0 기준 39랩 중 13회 → 44랩 중 0회.
- **충돌 지도 부풀림 rigid → circle 0.15 m**: 장애물 4개, seed 5개 각 240 s 기준 28랩·충돌 34회 → 65랩·충돌 11회.
- 아래 두 결과는 11/22 공지 이전 방식(gt 위치, 장애물 4개, 자동 복구, 헤어핀 구간 표)으로 잰 것입니다. 공지에 맞춰 구간 표를 없애고 곡률 규칙(`guide.max_curvature`)으로 바꿨으며, pf + 평가 방식(장애물 1/3/5, 첫 충돌까지 연속 랩)으로 다시 재는 중입니다.
- **장애물 노이즈 대비** (seed 3개, 각 240 s, guide + 헤어핀 구간 표): 장애물 위치 σ 0.10 m + 회전 무작위에서는 43랩 무충돌. 라이다 노이즈(거리 σ 3 cm, 빔 누락 2%, 허상 점 0.5%)에서는 충돌 5 → 11회로 늘어, 라이다 외톨이 점 제거 필터(`local_costmap.filter_isolated`, 기본 꺼짐)를 추가하고 학교 PC에서 확인 중.
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
