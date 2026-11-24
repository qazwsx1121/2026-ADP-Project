# 학교 PC(우분투)에서 pf 위치 추정 안정화 검증하기 — Claude Code 인수인계

> **학교 PC의 Claude Code에게**: 개인 PC의 Claude Code 세션에서 넘겨받는 문서입니다. 처음부터 읽고 **0장 규칙**을 지키면서 **4장 절차**를 진행한 뒤, **7장 형식**으로 결과를 보고하세요.
> 자세한 원인·수정 내용은 [`race_stack_changes/README.md`](race_stack_changes/README.md)에 있습니다.

터미널 표시: **[우분투]** = 학교 PC 호스트 터미널, **[컨테이너]** = `misys@` 프롬프트

---

## 0. 작업 규칙 (꼭 지킬 것)

- 답변은 **한국어**로 합니다.
- **git commit / push 를 하지 않습니다.** 사용자가 자기 계정으로 직접 커밋합니다(평가에서 각자 커밋 기록을 봄). 바꾼 파일과 커밋 방법만 알려 줍니다.
- 공식 SVG-MPPI 코드(`proj-svg_mppi/`)는 수정하지 않습니다. 고칠 수 있는 것은 연결 노드(`svg_mppi/`)와 실험 도구, 수업 스택의 파일(`race_stack_changes/`로 관리)입니다.
- **F1Tenth Gym(gym_bridge)과 race_manager 코드는 수정 금지**입니다(평가 규칙).
- **트랙 구간·좌표·waypoint 번호별로 파라미터를 바꾸면 큰 감점**입니다. 레이스라인 곡률 같은 일반 규칙만 씁니다.
- 주행 중 GT(실제 위치, 장애물 위치·크기)를 쓰면 감점입니다. 평가는 `pose_source:=pf`로 합니다. GT는 실험 비교와 학습에만 씁니다. berlin 지도는 GT가 아닙니다.
- 컨테이너와 이미지 `misys:forza_full`은 지우지 않습니다. 팀원 저장소 `byeongdsu/ADP-project-experiments`는 읽기만 합니다.
- 요청받지 않은 파일은 만들지 않습니다.
- 명령을 안내할 때는 어느 터미널인지([우분투] / [컨테이너]) 밝힙니다.
- 실험 전에 다른 실험이 돌고 있지 않은지 확인합니다. 프로세스를 찾거나 끌 때는 `pgrep -fa "[r]un_exp.sh"`처럼 첫 글자를 `[]`로 감쌉니다. 이렇게 해야 명령 자신이 검색에 걸리지 않습니다.
- 사용자가 같은 PC로 다른 무거운 일(게임 등)을 하면 계산 시간이 2배까지 늘어 결과가 왜곡됩니다. 이상하면 물어봅니다.

## 1. 배경

- 숭실대 2026-2 「자율주행 프로그래밍」 8조입니다. ForzaETH race stack(ROS 2 Humble, F1Tenth Gym)의 컨트롤러를 SVG-MPPI 하나로 바꿔 berlin 맵에서 주행합니다.
- 최종 평가는 12/16입니다. 장애물 1/3/5개마다 최대 3분 또는 10바퀴를 달리고, 첫 충돌 전까지의 **연속 랩 수**와 최고 랩 타임 순위를 합산합니다. 같은 비공개 seed를 씁니다.
- 위치 추정은 EKF(`/early_fusion/odom`)와 particle filter를 거쳐 `/car_state/odom`으로 나옵니다. EKF 입력은 `/car_state/odom_GT`의 속도와 IMU입니다. 이 위치를 컨트롤러가 받습니다.

## 2. 지금까지 한 일 (11/24, 개인 PC: Windows + WSL2, 6코어)

**문제**: `pose_source:=pf`로 달리면 1바퀴 안에 벽에 부딪혔습니다. 파티클 필터 위치가 1.5~11 m씩 순간이동했습니다.

**원인**
1. **WSL2의 시스템 시계 점프 (주원인)**: 아무것도 안 돌릴 때도 시스템 시계가 약 32초마다 2.94초씩 앞으로 튀었습니다. 같은 순간 monotonic 시계는 10 ms만 흘렀습니다. 모든 노드가 시스템 시계로 메시지 시각을 찍기 때문에, EKF는 그 간격만큼 외삽해 6~13 m 튀었습니다.
2. **EKF의 실제 멈춤**: CPU가 바쁘면 EKF가 약 0.35초씩 실제로 멈춥니다. 이때는 pf든 gt든 `/car_state/odom`이 같이 멈춥니다.
3. **초기 위치 메시지 유실**: 실험 도구의 `/initialpose`가 가끔 사라져, 파티클 필터가 14 m 떨어진 곳에서 시작했습니다.

**수정 (v6, main에 합쳐짐 — PR #9)**
- `race_stack_changes/particle_filter/.../particle_filter.py`: `apply_to_pc.sh` 4단계가 수업 스택에 덮어쓰고 빌드합니다. 원본은 `svg_mppi_exp/backup/race_stack_orig/`에 보관합니다.
  - 이동량을 EKF 위치 차이가 아니라 EKF 속도 × 메시지 간격으로 계산합니다(`odom_motion: velocity`).
  - 시계 점프를 감지합니다. 메시지 시각 차이가 실제 수신 간격(monotonic)보다 1.0 s 넘게 크면 수신 간격을 씁니다. 로그는 `system clock jump #N`입니다.
  - 0.5 s를 넘는 공백은 파티클마다 다른 경과 시간을 줘서 퍼뜨립니다. 로그는 `odometry gap #N`입니다.
- `svg_mppi_node`: 위치의 나이를 steady 시계로 재고, EKF 멈춤은 `pose_latency_max` 0.4 s까지 예측합니다. 레퍼런스 SDF 스레드는 2개입니다(`reference_sdf.threads`).
- `svg_mppi_exp/tools/init_pf_pose.py`: 초기 위치를 보낸 뒤 파티클 필터가 1 m 안에 들어왔는지 확인하고, 아니면 최대 5번 다시 보냅니다.

**개인 PC 결과** (pf, 장애물 3개, 평가 방식, 설정 G = 원형 부풀림 0.15 + guide + 곡률 0.65)

| 버전 | seed 7 | 11 | 2026 | 42 | 5 |
|---|---|---|---|---|---|
| 수정 전 | 모든 run이 1바퀴 안에 충돌 | | | | |
| v5 | 8바퀴, 0.18 m | 9바퀴 무충돌, 0.18 m | 3바퀴, 0.17 m | 0, 0.17 m | 4바퀴, 0.17 m |
| v6 (현재) | 9바퀴 무충돌, 0.18 m | 0 (A), 0.13 m | 0 (B), 1.74 m | — | 1바퀴, 0.16 m |

(숫자 = 연속 랩, 위치 오차 평균. 개인 PC는 "9바퀴 무충돌"이면 3분 시간 제한으로 끝난 것입니다.)
- v5/v6 충돌 9번 중 7번은 위치 오차 0.1~0.2 m 상태에서 났습니다. seed 2026과 42는 gt로 돌려도 같은 곳에서 충돌했습니다. → **컨트롤러 쪽 문제**입니다(6장).
- **(A)** race_manager 버그입니다. 첫 바퀴 장애물 생성이 출발 후 약 2.3초 늦게 끝나서, 장애물이 차 바로 위에 생겼습니다(생성 0.04초 뒤 충돌).
- **(B)** 헤어핀 입구에서 MPPI 계산이 50~120 ms(주기 25 ms)로 늘었습니다. CPU를 못 받은 파티클 필터가 방향이 0.25 rad 뒤처진 뒤 위치를 잃었습니다.

## 3. 이번 검증의 목적

1. 우분투(가상화 없음)와 8코어 PC에서 pf 위치 오차가 모든 run에서 작게 유지되는지 확인합니다.
2. 시계 점프가 없는지 확인합니다. 로그에 `system clock jump`가 0번이어야 합니다.
3. CPU 여유가 있을 때 (B)처럼 파티클 필터가 위치를 잃는 일이 사라지는지 봅니다.
4. 같은 seed를 gt로도 돌려 충돌 원인(컨트롤러 / pf / race_manager 버그)을 나눕니다.

## 4. 검증 절차

### 4-0. Claude Code가 우분투 호스트에서 돌 때
- 컨테이너 안 명령은 `docker exec <컨테이너 이름> bash -c '...'`로 실행합니다. 이름은 `docker ps`로 확인합니다(꺼져 있으면 사용자에게 켜 달라고 합니다).
- 공유 폴더는 호스트 `~/shared_dir`와 컨테이너 `/home/misys/shared_dir`입니다(`dkrun.sh` 기준). 다르면 `docker inspect -f '{{json .Mounts}}' <이름>`으로 확인합니다.
- `run_exp.sh`, `eval_sweep.sh`는 ROS 환경을 스스로 불러옵니다(source). 직접 `ros2` 명령을 쓸 때는 `source /opt/ros/humble/setup.bash; source ~/forza_ws/race_stack/install/setup.bash`가 필요합니다.
- 실험은 오래 걸리므로 백그라운드로 돌리고 끝나면 결과를 봅니다.

### 4-1. 시계 점프 확인 — [우분투], 70초
```bash
python3 -c "import time;t,m=time.time(),time.monotonic();time.sleep(70);print('realtime',round(time.time()-t,2),'monotonic',round(time.monotonic()-m,2))"
```
두 값의 차이가 0.1 s 안쪽이면 점프가 없는 것입니다. 개인 PC(WSL2)에서는 2.9 s 이상 차이 났습니다.

### 4-2. 최신 코드 적용 — [컨테이너]
```bash
cd ~/shared_dir/2026-ADP-Project && git checkout main && git pull
bash ~/shared_dir/2026-ADP-Project/apply_to_pc.sh
```
- 출력에 `[4/4] race_stack 에 반영: particle_filter`와 `빌드 완료`가 나와야 합니다.
- 저장소가 아직 없으면 `cd ~/shared_dir && git clone https://github.com/qazwsx1121/2026-ADP-Project.git`을 먼저 실행합니다.
- 적용됐는지 확인합니다. 결과가 2 이상이어야 합니다.
  ```bash
  grep -c odom_clock_jump ~/forza_ws/race_stack/install/particle_filter/lib/python3.10/site-packages/particle_filter/particle_filter.py
  ```

### 4-3. pf 주행 — [컨테이너], 최대 약 18분
개인 PC와 같은 조건입니다. 장애물 3개, seed 5개, 패키지 기본 `svg_mppi_light.yaml`에서 만든 설정 G를 씁니다.
```bash
bash ~/shared_dir/svg_mppi_exp/tools/eval_sweep.sh cfgs=G obs=3 seeds="7 11 2026 42 5" tag=pf
```
결과 폴더는 `~/shared_dir/svg_mppi_exp/runs/<시각>_pfG_o3_s<seed>`입니다.

### 4-4. gt 비교 주행 — [컨테이너], 최대 약 18분
```bash
bash ~/shared_dir/svg_mppi_exp/tools/eval_sweep.sh cfgs=G obs=3 seeds="7 11 2026 42 5" tag=gt pose=gt
```

### 4-5. (시간이 되면) 장애물 1/5개
```bash
bash ~/shared_dir/svg_mppi_exp/tools/eval_sweep.sh cfgs=G obs="1 5" seeds="7 11 2026 42 5" tag=pf
```

## 5. 결과 확인 방법 — [컨테이너]

```bash
cd ~/shared_dir/svg_mppi_exp
python3 analysis/summarize.py runs/*_pfG_o3_s* runs/*_gtG_o3_s*
for d in runs/*_pfG_o3_s*; do echo "$d | 시계점프 $(grep -c 'system clock jump' $d/base.log) | 공백 $(grep -c 'odometry gap' $d/base.log) | 가중치리셋 $(grep -c 'Weight sum invalid' $d/base.log) | $(tail -1 $d/init_pose.log)"; done
```
- `summarize.py` 열
  - `streak`: 첫 충돌 전 연속 랩 수
  - `end`: 끝난 이유. crash / time / laps
  - `loc_mean/p95/max`: 파티클 필터 위치 오차 [m]
  - `calc_mean/p95`, `overrun%`: MPPI 계산 시간 [ms]과 25 ms 초과 비율
- `trace.csv` 열: `t,x,y,yaw,v,steer_cmd,speed_cmd,calc_ms,coll_rate,cost,collision,idx,s,d,best_end_x,best_end_y,guide_offset,est_x,est_y,est_yaw,loc_err,loc_yaw_err,ekf_x,ekf_y,ekf_v`
  - x, y, yaw는 실제(GT) 위치, est_*는 파티클 필터 위치입니다. s, d는 레이스라인 기준 좌표이고 guide_offset은 장애물 회피 오프셋입니다.
- 로그 파일
  - `base.log`: 시뮬, EKF, particle_filter
  - `mppi.log`: 우리 노드. `guide paused`, `pose is ... old` 등
  - `race.log`: 점수와 장애물 생성
  - `events.csv`, `init_pose.log`


**race_manager 버그(A) 찾기**: 장애물이 생긴 지 0.6초 안에 충돌한 run을 찾습니다.
```bash
for D in runs/*_pfG_o3_s* runs/*_gtG_o3_s*; do python3 - "$D" <<'EOF'
import re, sys
last = None
for line in open(sys.argv[1] + "/race.log", errors="ignore"):
    m = re.search(r"\[(1\d{9}\.\d+)\] \[(obstacle_spawner|scorer)\]: (.*)", line)
    if not m: continue
    t, who, msg = float(m.group(1)), m.group(2), m.group(3)
    if who == "obstacle_spawner" and re.match(r"lap \d+ \[", msg): last = t
    elif who == "scorer" and "collision #" in msg and last and t - last < 0.6:
        print(sys.argv[1], f"장애물 생성 {t - last:.2f} s 뒤 충돌 (race_manager 버그)")
EOF
done
```

**판단 기준**
- **pf 안정**: 모든 pf run에서 `loc_mean` ≤ 0.25 m이고 `loc_max` < 1 m입니다.
- **발산 run**(`loc_max` ≥ 1 m)이 있으면 원인을 봅니다.
  - `trace.csv`에서 `loc_err`이 커지기 시작한 시각을 찾습니다.
  - 그 직전 `calc_ms`가 치솟았는지(CPU 부족)와 `loc_yaw_err`(방향 오차)를 봅니다.
  - 그 시각 근처의 `base.log` particle_filter 경고를 봅니다.
- **충돌 분류**: 각 충돌을 다음 중 하나로 나눕니다.
  - (1) 위치 오차가 작고 gt도 같은 곳에서 충돌 → 컨트롤러
  - (2) race_manager 버그 (A)
  - (3) pf 발산
  - (4) 그 밖
- `system clock jump`가 0번이 아니면 학교 PC에도 시계 점프가 있다는 뜻이니 꼭 보고합니다.

## 6. 알려진 남은 문제 (다음 과제 후보 — 고치기 전에 사용자와 상의)

1. **guide 곡률 규칙**(`guide.max_curvature` 0.65): 장애물을 피하던 guide가 S자 헤어핀 입구(s≈21.5, 곡률 0.69)에서 꺼져 바로 장애물에 부딪힙니다(seed 42, 5, 7). 헤어핀 직후(s≈29) 장애물도 늦게 피합니다. gt에서도 재현됩니다. 후보 해결책은 "회피 중이면 곡률 규칙으로 끄지 않기" 같은 일반 규칙입니다.
2. S자 헤어핀(s≈23~26)에서 저속으로 벽에 닿습니다(seed 2026, gt에서도 재현).
3. 파티클 필터의 CPU 부족(B): 후보는 파티클 수 줄이기, MPPI 노드 우선순위 낮추기(`nice`)입니다.
4. race_manager 첫 바퀴 장애물 생성 지연(A): 수정 금지이므로 조교님께 알릴지 사용자와 상의합니다.
5. `init_pf_pose.py`는 시작 위치를 GT에서 읽습니다. 실험용으로는 괜찮지만, 최종 데모에서는 GT 없는 초기화(예: 출발 위치를 레이스라인 시작점에서 정하기)가 필요합니다.
6. 다시 시도하지 말 것: berlin S자 막다른 공간 문제는 λ 1.0으로 해결했습니다. 모델 맞추기(steer/lf/lr), 긴 예측 구간, 헤어핀 감속은 모두 더 나빠졌습니다.

## 7. 보고 형식

사용자에게 아래 표와 결론을 한국어로 보고합니다.

| pose | seed | 연속 랩 | 끝난 이유 / 위치(s) | 최고 랩 | 위치 오차 평균 / 최대 | 시계 점프 | 충돌 분류 |
|---|---|---|---|---|---|---|---|

- 결론 1: pf가 안정적인가(개인 PC 결과와 비교).
- 결론 2: 남은 충돌 원인별 개수와 다음에 할 일 제안(6장 후보 중).
- 4-1의 시계 점프 측정값.
