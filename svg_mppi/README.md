# svg_mppi — ForzaETH race stack용 SVG-MPPI 컨트롤러 (8조)

공식 SVG-MPPI 코드(kohonda/proj-svg_mppi, ICRA 2024)를 **수정 없이** 컴파일해서,
PP·TLN처럼 `/drive`로 주행 명령을 내는 ROS 2 노드로 쓰는 패키지입니다.

```
/scan  /car_state/odom  /global_waypoints_scaled   →  svg_mppi_node  →  /drive
```

| 파일 | 내용 |
|---|---|
| `src/svg_mppi_node.cpp` | 공식 ROS 1 노드 3개(reference_sdf_generator, local_costmap_generator, mppi_controller_ros)를 ROS 2 노드 하나로 옮긴 것 |
| `config/svg_mppi.yaml` | 공식 설정값 그대로 (thread_num만 6으로) |
| `launch/svg_mppi_launch.xml` | 실행 파일 (`mpc_mode:=svg_mppi` 또는 `forward_mppi`) |
| 알고리즘 본체 | `~/shared_dir/proj-svg_mppi/src/mppi_controller` (공식 코드, 빌드 때 그대로 컴파일) |

## 설치 (컨테이너 안)
```bash
cd ~/shared_dir && git clone https://github.com/kohonda/proj-svg_mppi.git   # 공식 코드 (race_stack 밖에 둘 것)
# svg_mppi 폴더를 ~/forza_ws/race_stack/controller/ 안에 넣기 (VS Code 드래그)
cd ~/forza_ws/race_stack
colcon build --packages-select svg_mppi
source install/setup.bash
```

## 실행
```bash
ros2 launch stack_master base_system_launch.xml map_name:=berlin pose_source:=gt   # T1
ros2 launch svg_mppi svg_mppi_launch.xml                                            # T2 (PP/TLN 대신)
ros2 launch race_manager race_manager_launch.xml map_name:=berlin seed:=2026 num_obstacles:=0   # T3
```
- 일반 MPPI와 비교: `ros2 launch svg_mppi svg_mppi_launch.xml mpc_mode:=forward_mppi`
- **CPU가 부족할 때(계산 시간 > 25 ms)**: 가벼운 설정
  `ros2 launch svg_mppi svg_mppi_launch.xml params:=$(ros2 pkg prefix svg_mppi)/share/svg_mppi/config/svg_mppi_light.yaml`
  (샘플 8000→3000, 기울기 샘플 100→50, 레이스라인 지도 20 m→12 m, 스레드 4)
- 파라미터 1회 변경: `config/svg_mppi.yaml` 수정 후 다시 실행 (빌드 불필요, `--symlink-install`이 아니면 빌드 1회)

## 확인용 토픽
| 토픽 | 내용 |
|---|---|
| `/mppi/best_path` (Path) | 선택된 예측 궤적 — RViz에서 Add → By topic |
| `/mppi/local_costmap` (Map) | 라이다로 만든 충돌 지도 (차 기준) |
| `/mppi/candidate_paths` | 샘플 궤적 100개 (`visualize_candidates:=true`일 때) |
| `/mppi/calculation_time` | 한 주기 계산 시간 [ms] — 25 ms 안이어야 함 |
| `/mppi/collision_rate` | 샘플 중 충돌 비용이 있는 비율 |

## 공식 코드와 다른 점
- ROS 1 → ROS 2, 노드 3개 → 1개 (계산 내용은 동일하게 옮김)
- 레이스라인을 CSV 대신 스택의 `/global_waypoints_scaled`에서 받음 → `speed_scaling.yaml` 배율이 그대로 적용
- 위치를 TF 대신 `/car_state/odom`에서 받음 → `pose_source:=gt|pf`가 그대로 적용
- 라이다 점 처리에 PCL을 쓰지 않음 (공식 설정에서 꺼져 있던 필터들은 생략)
- `drive_publish_enabled` 스위치 추가 (TLN과 동일)
- 레이스라인 지도(reference SDF)는 공식처럼 제어와 **별도 스레드**에서 갱신
- 로봇 프레임은 `/car_state/odom`의 child_frame_id를 자동 사용 (우리 스택: `car_state/base_link`)
- 25 ms를 넘기면 로그에 단계별 시간이 찍힘: `copy_sdf / costmap / solve / ref_sdf thread`

## 추가 기능 (11/17): 기준 경로 확인용

- `/mppi/reference_path` (nav_msgs/Path): MPPI가 따라가려는 앞쪽 8 m 기준 경로. RViz에서 Add → By topic으로 추가.
- 터미널 경고 `reference jump: waypoint A -> B`: 최근접 waypoint가 3 m 넘게 튀면 출력.
- `reference_sdf.local_search` (기본 false = 공식 코드 방식): true면 직전 위치 주변만 검색해서 벽 건너편 구간으로 튀지 않게 함.
  실행 중 전환: `ros2 param set /svg_mppi reference_sdf.local_search true`

## 추가 기능 (11/19)

### 1) 주행 중 파라미터 변경
`ros2 param set /svg_mppi <이름> <값>` 으로 주행 중에 바꿀 수 있음 (실수형은 `5.0`처럼 소수점 필요).

| 종류 | 예 | 반영 방식 |
|---|---|---|
| 노드 파라미터 | `reference_sdf.reference_speed_scale`, `reference_sdf.num_waypoints`, `reference_speed`, `constant_speed_mode`, `guide.*` | 다음 주기부터 즉시 |
| solver 파라미터 | `svg_mppi.lambda`, `svg_mppi.steer_cov`, `svg_mppi.sample_batch_num`, `common.prediction_step_size`, `common.q_dist` … | 새 solver 를 백그라운드에서 만든 뒤 제어 주기 사이에 교체 (solver 는 항상 1개, 교체 순간 웜스타트 초기화) |

```bash
ros2 param set /svg_mppi reference_sdf.reference_speed_scale 1.2   # 전체 속도 20% 올리기
ros2 param set /svg_mppi svg_mppi.lambda 5.0                        # solver 재생성 로그: "new solver built ..."
```

### 2) 장애물 순서를 고려한 가이드 기준 경로 (`guide.*`, 기본 꺼짐)
교수님 피드백(연속 장애물에서 SVG 선택 유도)에 대한 구현. 공식 solver 는 그대로이고 **기준 경로(reference SDF)만** 바꾼다.
1. 라이다 점 중 정적 지도(`/map`)의 벽에서 `guide.static_margin` 이상 떨어진 점 = 장애물 (0.1 m 격자, `guide.memory` 초 기억)
2. 레이스라인 기준 Frenet 좌표(s, d)로 바꿔 앞쪽 `guide.lookahead` m 안의 장애물 열 전체를 본다
3. DP(동적 계획법)로 "각 장애물을 어느 쪽으로 지나갈지" 순서를 한 번에 정한다
   비용 = 레이스라인 이탈 + 가로 기울기 + 직전 계획과의 차이(좌/우가 매 주기 뒤집히지 않게) + 장애물/벽 여유
4. 그만큼 옆으로 옮긴 레이스라인으로 reference SDF 를 만든다 → SVG 의 guide 입자와 샘플이 그 방향 모드로 모인다

장애물이 없으면 원래 레이스라인을 그대로 쓴다 (= 공식과 같은 동작). `/drive` 는 여전히 SVG-MPPI 하나만 낸다.
```bash
ros2 param set /svg_mppi guide.enable true
```
RViz: `/mppi/guide_path` (DP 결과), `/mppi/guide_obstacles` (인식한 장애물 점), `/mppi/reference_path`.
- 같은 칸이 스캔 3번 이상 보여야 장애물로 인정 (벽 오인식 방지), 라이다 점은 스캔 시각의 자세로 변환
- `guide.w_side`: 직전 계획과 반대쪽으로 같은 장애물을 지나가는 비용 (좌/우 결정이 주기마다 뒤집히지 않게)

### 3) 충돌 지도 부풀림 방식 (`local_costmap.inflation_mode`) — 기본값 circle 0.15 (11/19 변경)
- `circle` (기본): 방향과 무관하게 반지름 `local_costmap.circle_radius` = 0.15 m (시뮬 차 반폭 0.10 + 여유 0.05)로 부풀림
- `rigid` (공식): 판단 시점 차 방향의 차체 상자(앞 0.47 / 뒤 0.14 / 좌우 0.15)로 부풀림. 헤어핀처럼 예측 구간 안에서 크게 도는 곳에서는 통로가 실제보다 좁게 계산됨
- 근거 (장애물 4개, seed 2026/11/7/42/100, 각 240 s): rigid 28랩·충돌 34회 → circle 0.15 65랩·충돌 11회. 반지름 0.20/0.25/0.30 은 헤어핀 꼭짓점 장애물(seed 2026 2랩)에서 막힘
- 시작 로그에 `inflation=circle r=0.15` 로 표시. 시작할 때 정해짐 (주행 중 변경 아님)
- 실차로 옮길 때는 차 크기에 맞게 반지름을 다시 정할 것

### 4) 기본값 변경: `svg_mppi.lambda` 3.0 → 1.0
berlin 헤어핀(s≈24)에서 λ 3.0이면 좌회전(좁은 통로)과 막다른 주머니(넓은 공간) 샘플이 가중 평균에서 섞여 주머니로 감.
λ 1.0: 장애물 0 기준 44랩 주머니 진입 0회 (λ 3.0: 39랩 중 13회).

### 5) 진단 기록 (`debug.dump_dir`)
경로를 주면 조향이 급변하거나 충돌률이 급상승한 순간의 후보 궤적·guide·충돌 지도·기준 경로를 `flip_*.json` 으로 저장 (`svg_mppi_exp/analysis/flip_plot.py` 로 그림). 비우면 꺼짐.
`/mppi/guide_offset`: 가이드가 레이스라인에서 가장 많이 옮긴 거리 [m] (장애물 없으면 0).
