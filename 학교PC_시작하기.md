# 학교 PC(평가 PC 사양)에서 학습 시작하기

준비물은 `svg_mppi_bundle_20261119.zip` 하나입니다. 내 PC의 WSL `~/shared_dir/`에 있고, Windows 탐색기에서는 `\\wsl$\Ubuntu-22.04\home\user1\shared_dir\` (또는 Windows `다운로드` 폴더에 복사본)로 열 수 있습니다. 여기서 USB나 드라이브로 복사해 가면 됩니다.

묶음에 들어 있는 것
- `svg_mppi/`: 연결 노드 패키지 (λ 1.0, 원형 부풀림 0.15가 기본값)
- `proj-svg_mppi/`: 공식 SVG-MPPI 코드 (수정 없음)
- `svg_mppi_exp/`: 실험·학습 도구(`tools/`), 설정(`configs/`), 분석(`analysis/`), Optuna(`pylib/`), 보고서(`report/`)

터미널 표시: **[WSL]** = `user1@` 프롬프트, **[컨테이너]** = `misys@` 프롬프트

---

## 1. 환경 확인 (5분)

수업 컨테이너(`misys:forza_full`)가 있어야 합니다.

```bash
# [WSL]
docker images | grep forza
docker ps -a
```

컨테이너에 들어가서 확인합니다.

```bash
# [컨테이너]
ls ~/forza_ws/race_stack/stack_master/maps/berlin
ls ~/shared_dir
```

컨테이너나 이미지가 없으면 9번을 보세요.

## 2. 압축 풀기

zip을 컨테이너의 `~/shared_dir`에 넣습니다(VS Code 탐색기로 드래그하거나 WSL의 공유 폴더로 복사). 컨테이너에는 `unzip`이 없어서 파이썬으로 풉니다.

```bash
# [컨테이너]
cd ~/shared_dir && python3 -m zipfile -e svg_mppi_bundle_20261119.zip .
chmod +x ~/shared_dir/svg_mppi_exp/tools/*.sh
```

`~/shared_dir/svg_mppi`, `proj-svg_mppi`, `svg_mppi_exp`가 생깁니다. 이미 `proj-svg_mppi`가 있으면 덮어써도 됩니다(공식 코드 그대로).

## 3. 빌드 (1분)

```bash
# [컨테이너]
source /opt/ros/humble/setup.bash
source ~/forza_ws/race_stack/install/setup.bash
cp -r ~/shared_dir/svg_mppi ~/forza_ws/race_stack/
cd ~/forza_ws/race_stack && colcon build --packages-select svg_mppi && source install/setup.bash
```

`svg_mppi`는 반드시 `race_stack` 바로 아래에 있어야 합니다(`controller/` 안에 넣으면 못 찾음).

## 4. 동작 확인 (2분)

```bash
# [컨테이너]
C=~/shared_dir/svg_mppi_exp/configs
bash ~/shared_dir/svg_mppi_exp/tools/run_exp.sh check dur=60 obs=0 params=$C/school_light_t8.yaml
```

마지막에 `laps=… lap times …`가 나오면 정상입니다. 시작 로그(`svg_mppi_exp/runs/*_check/mppi.log`)에 `inflation=circle r=0.15`가 있어야 합니다.

## 5. 학습에 쓸 설정 정하기 (8분)

평가 PC가 원본 샘플 수(8000)를 감당하는지 잽니다.

```bash
# [컨테이너]
C=~/shared_dir/svg_mppi_exp/configs
bash ~/shared_dir/svg_mppi_exp/tools/batch2.sh 120 "bench_light params=$C/school_light_t8.yaml obs=0" "bench_o8 params=$C/school_orig_t8.yaml obs=0" "bench_o12 params=$C/school_orig_t12.yaml obs=0"
cd ~/shared_dir/svg_mppi_exp/runs && python3 ../analysis/summarize.py *_bench_*
```

| 설정 파일 | 샘플 | 스레드 |
|---|---|---|
| `school_light_t8.yaml` | 3000 | 8 |
| `school_orig_t8.yaml` | 8000 | 8 |
| `school_orig_t12.yaml` | 8000 | 12 |

**고르는 기준:** `calc_mean` 20 ms 이하이고 `overrun%` 15% 이하인 것 중 샘플이 많은 것. 원본 두 개가 모두 기준을 넘으면 `school_light_t8.yaml`을 씁니다. 내 PC(6코어)에서는 light가 20 ms, 원본이 43 ms였습니다.

## 6. 학습 시작

```bash
# [컨테이너]  <고른 설정> 을 5번에서 고른 파일 이름으로
C=~/shared_dir/svg_mppi_exp/configs
bash ~/shared_dir/svg_mppi_exp/tools/train.sh berlin_v1 trials=60 laps=4 seed=100 params=$C/<고른 설정>.yaml
```

- trial 하나 = 파라미터 표 하나를 넣고 4랩(랩마다 장애물 배치가 바뀜) 달린 결과로 점수를 냅니다. 점수 = 평균 랩타임 + 10 × 충돌 + 15 × 복구.
- trial 1개에 약 1.5분 → 60개면 1.5~2시간. 밤새 돌리려면 `trials=300`.
- 첫 trial은 현재 최고 설정(비교 기준), 처음 8개는 무작위 탐색입니다.
- 학습하는 값(트랙 전체에 같은 값, 구간·좌표별 값 없음): 전체 속도 배율, guide 를 쉬는 곡률 기준(`guide.max_curvature`), 장애물 근처 속도, guide 여유 거리·유지 비용, λ (`configs/tune_space.yaml`). 11/22 공지: 구간별 하드코딩은 대폭 감점.
- 진행 확인(다른 컨테이너 터미널): `tail -f ~/shared_dir/svg_mppi_exp/tuning/berlin_v1_trials.csv`
- 멈추기: Ctrl-C. 같은 이름(`berlin_v1`)으로 다시 실행하면 이어서 학습합니다(`tuning/berlin_v1.db`).
- GUI 없이 돕니다. **학습 중에는 같은 PC에서 게임 등 무거운 작업을 하지 마세요.** 계산 시간이 2배가 되어 학습 결과가 틀어집니다.
- 충돌하거나 한 곳에서 15초 넘게 막히면 실험 도구가 차를 장애물 너머로 옮깁니다(평가 때 사람이 하는 일).

## 7. 학습 결과 검증 (약 45분)

학습에 쓰지 않은 seed로, 학습한 표(T)와 기준 설정(B)을 비교합니다.

```bash
# [컨테이너]
C=~/shared_dir/svg_mppi_exp/configs
bash ~/shared_dir/svg_mppi_exp/tools/validate.sh ~/shared_dir/svg_mppi_exp/tuning/berlin_v1_best_schedule.yaml params=$C/<고른 설정>.yaml
```

seed 2026, 11, 7, 42, 5 × 240초 × (T, B). 마지막에 `vT`(학습 표)와 `vB`(기준)의 랩·클린 랩·충돌 합계가 나옵니다. T가 B보다 충돌이 적고 랩이 많아야 학습이 의미가 있습니다.

학습한 표를 주행 중에 적용하는 방법(평가 때):

```bash
# [컨테이너] 시뮬과 svg_mppi 를 띄운 뒤
python3 ~/shared_dir/svg_mppi_exp/tools/param_scheduler.py --ros-args -p table:=$HOME/shared_dir/svg_mppi_exp/tuning/berlin_v1_best_schedule.yaml
```

## 8. 결과 가져오기

```bash
# [컨테이너]
cd ~/shared_dir && python3 -m zipfile -c school_results.zip svg_mppi_exp/tuning svg_mppi_exp/runs
```

`runs`가 너무 크면 `svg_mppi_exp/tuning`만 가져와도 됩니다(db, trials.csv, best_schedule.yaml이 핵심).

## 9. 문제가 생기면

| 증상 | 해결 |
|---|---|
| 컨테이너나 이미지가 없음 | 수업 이미지 `misys:forza_full`이 필요합니다. 조교님께 문의하거나, 내 PC에서 `docker save`로 이미지 파일(약 10 GB)을 만들어 가져가야 합니다. |
| `ros2: command not found` | 3번의 `source` 두 줄 |
| `Package 'svg_mppi' not found` | 3번 다시 (`race_stack` 바로 아래에 복사) |
| `ModuleNotFoundError: optuna` | `~/shared_dir/svg_mppi_exp/pylib`가 있는지 확인. `train.sh`가 자동으로 경로를 넣습니다. |
| 공식 코드를 찾을 수 없다는 CMake 오류 | `~/shared_dir/proj-svg_mppi/src/mppi_controller`가 있는지 확인 |
| 토픽이 안 보임 | 터미널마다 `ROS_DOMAIN_ID`가 같은지 확인 |
| 계산 시간이 이상하게 큼 | 다른 프로그램 종료. CPU 확인: `for i in 1 2 3 4 5 6 7 8; do python3 -c "sum(i*i for i in range(10**7))" & done; time wait` |
