# 학교 PC: 업데이트 적용과 노이즈 실험

준비물: `svg_mppi_update_20261120.zip` (Windows `다운로드` 폴더, 또는 WSL `~/shared_dir/`)

이번 업데이트에 들어 있는 것
- 장애물 노이즈 / 라이다 노이즈를 켜고 실험하는 도구(`obstacle_noise.py`, `scan_noise.py`, `run_exp.sh` 옵션)
- 노이즈 실험 한 줄 실행 스크립트(`noise_sweep.sh`)
- 연결 노드: 라이다 외톨이 점(허상 점) 제거 필터 `local_costmap.filter_isolated` (기본 꺼짐)
- `train.sh`, `validate.sh`에 노이즈 옵션 추가, 옵션 값에 공백이 있으면 잘리던 버그 수정

터미널 표시: **[WSL]** = `user1@`, **[컨테이너]** = `misys@`

---

## 0. 반드시 학습이 끝난 뒤에 적용

지금 돌고 있는 학습(`train.sh`)은 `run_exp.sh`를 쓰고 있습니다. 실행 중인 bash 스크립트 파일을 덮어쓰면 **학습이 중간에 깨질 수 있습니다.** 학습이 끝났는지 먼저 확인하세요.

```bash
# [컨테이너] 아무것도 안 나오면 끝난 것
ps -eo args | grep -E "[t]rain.sh|[r]un_exp.sh"
```

학습 결과(`~/shared_dir/svg_mppi_exp/tuning/`)는 업데이트해도 지워지지 않습니다.

## 1. 압축 풀기와 빌드 (2분)

zip을 컨테이너의 `~/shared_dir`에 넣습니다.

```bash
# [컨테이너]
cd ~/shared_dir && python3 -m zipfile -e svg_mppi_update_20261120.zip .
chmod +x ~/shared_dir/svg_mppi_exp/tools/*.sh
source /opt/ros/humble/setup.bash
cp -r ~/shared_dir/svg_mppi ~/forza_ws/race_stack/
cd ~/forza_ws/race_stack && colcon build --packages-select svg_mppi && source install/setup.bash
```

## 2. 1분 확인 (노이즈 두 종류 + 필터)

```bash
# [컨테이너]  <고른 설정> = 학습 때 고른 school_*.yaml
C=~/shared_dir/svg_mppi_exp/configs
bash ~/shared_dir/svg_mppi_exp/tools/noise_sweep.sh cfgs=GF levels=N4 seeds=11 dur=60 tag=chk base=$C/<고른 설정>.yaml
```

`laps=…`가 나오고 `ERROR`가 없으면 정상입니다. 장애물 노이즈 노드가 장애물을 못 내면 `ERROR: obstacle_noise did not publish obstacles`로 멈춥니다.

## 3. 노이즈 실험 (기본 약 2시간 15분)

```bash
# [컨테이너]
C=~/shared_dir/svg_mppi_exp/configs
bash ~/shared_dir/svg_mppi_exp/tools/noise_sweep.sh base=$C/<고른 설정>.yaml
```

기본은 G, GF × N2, N3, N4 × seed 5개(2026 11 7 42 5) × 240초 = 30회입니다. 마지막에 조건별 합계 표가 나옵니다.

| 설정 | 내용 |
|---|---|
| C | 원형 0.15, guide 끔 (패키지 기본값) |
| G | C + guide 켬, 레이스라인 곡률이 0.65 1/m 보다 큰 곳에서는 guide 쉼 (구간·좌표별 설정 없음) |
| CF / GF | C / G + 라이다 외톨이 점 제거 필터 |

| 노이즈 | 내용 |
|---|---|
| N0 | 없음 |
| N2 | 장애물 위치 σ 0.10 m + 회전 무작위 (배치마다 새로) |
| N3 | 라이다 거리 σ 3 cm + 빔 누락 2% + 허상 점 0.5% (컨트롤러가 보는 스캔에만, 실제 충돌 판정은 그대로) |
| N4 | N2 + N3 |

- C까지 비교하려면 `cfgs="C G CF GF"` (시간 2배), 노이즈 없는 기준도 보려면 `levels="N0 N2 N3 N4"`.
- 결과 폴더: `svg_mppi_exp/runs/*_ns<설정><노이즈>_s<seed>` (전광판 PNG 포함)
- 합계만 다시 보기: `cd ~/shared_dir/svg_mppi_exp/analysis && python3 group_summary.py nsGN2 nsGN3 nsGN4 nsGFN2 nsGFN3 nsGFN4`

## 4. 내 PC에서 미리 본 결과 (seed 2026·11·7, 240초씩, light 설정)

| 설정 | 노이즈 없음 | 장애물 흔들기 N2 | 라이다 N3 | 둘 다 N4 |
|---|---|---|---|---|
| C | 충돌 9 / 37랩 | 10 / 42 | 23 / 37 | 22 / 35 |
| F | 5 / 42 | **0 / 43** | 11 / 40 | 7 / 39 |

장애물 위치 노이즈에는 강하고, 라이다 노이즈에 약합니다. F(당시 헤어핀 구간 표, 현재는 곡률 규칙 G로 대체)가 C보다 강합니다. 필터(GF)는 아직 시험하지 않았습니다. 학교에서 처음 확인하는 것입니다.

## 5. 노이즈를 켜고 학습하기 (3번 결과를 본 뒤)

필터가 도움이 되면(GF가 G보다 충돌이 적으면) 필터를 켠 설정으로, 아니면 그대로 학습합니다.

```bash
# [컨테이너]  필터를 쓸 때는 params 를 configs/generated/GF_<고른 설정 이름>_k0.65.yaml 로
C=~/shared_dir/svg_mppi_exp/configs
bash ~/shared_dir/svg_mppi_exp/tools/train.sh berlin_noise trials=60 laps=4 seed=100 \
  params=$C/<고른 설정>.yaml noise="onoise=0.10 orot=1 snoise=0.03 sdrop=0.02 sghost=0.005"
```

검증도 같은 노이즈로 합니다.

```bash
# [컨테이너]
bash ~/shared_dir/svg_mppi_exp/tools/validate.sh ~/shared_dir/svg_mppi_exp/tuning/berlin_noise_best_schedule.yaml \
  params=$C/<고른 설정>.yaml noise="onoise=0.10 orot=1 snoise=0.03 sdrop=0.02 sghost=0.005"
```

## 6. 결과 가져오기

```bash
# [컨테이너]
cd ~/shared_dir && python3 -m zipfile -c school_results_noise.zip svg_mppi_exp/tuning svg_mppi_exp/runs
```
