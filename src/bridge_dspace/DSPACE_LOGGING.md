# dSPACE 측정 로깅 규격 (ControlDesk) — 상위 PC 로그와 합치기 위한 최소 조건

**목적:** dSPACE에서 찍은 「MPC 출력 str vs 실제 str」·「수신 목표속도 vs 실제 속도」를
PC의 rosbag(스테이트·ref·인지)과 **같은 시간축**에 올려, 어느 구간이 GPS/차선/회피였는지
대조해 본다. 병합·그래프는 `src/adas_mgm/tools/dspace_merge.py`가 한다.

> 담당: 손상민(dSPACE 측 변수 선정·측정) ↔ 김윤기(병합 분석).
> **시험 전에 이 표대로 측정 변수를 잡아 둘 것** — 나중에 추가할 수 없다.

## 1. 로깅할 변수

이름은 자유다 (도구가 이름으로 자동 추정하고, 안 되면 `--map`으로 지정한다).
**무엇을 찍느냐가 중요하다.**

| # | 변수 | 어디서 | 왜 필요한가 | 등급 |
|---|---|---|---|---|
| 1 | **시간** | 모델 시간 [s] | 로그의 시간축 (0부터여도 됨) | 필수 |
| 2 | **counter** | `0x100` TARGET_HEADER byte0-1 (u16) | **PC 로그와 틱 단위로 정확히 맞추는 열쇠** (§3) | 필수 |
| 3 | **v_ref (수신)** | `0x100` byte4-5 ÷1000 | 목표속도. counter가 없을 때의 예비 동기 신호도 됨 | 필수 |
| 4 | **v 실제** | 차속 추정/엔코더 | 목표 대비 실제 속도 | 필수 |
| 5 | **str (MPC 출력)** | MPC 결과 str_ref | 명령 조향각 | 필수 |
| 6 | **str 실제** | 조향 피드백(서보/포텐쇼) | 실현 조향각 | 필수 |
| 7 | **state (수신)** | `0x100` byte2 (u8) | 0=차선 1=GPS 2=회피 3=주차. **이것만 있으면 dSPACE 로그만으로도 구간 색칠이 된다** | 강력권장 |
| 8 | n_points (수신) | `0x100` byte3 | v3 유효 범위 1~3 확인 | 권장 |
| 9 | ref_point0 x, y | `0x101` byte0-3 ÷1000 | 명령 곡률 ↔ 실현율(CLAUDE.md §3 ③) 대조 | 권장 |
| 10 | watchdog 상태 | 내부 플래그 | counter 끊김 시 v_ref=0 동작 확인 | 권장 |
| 11 | 0x200~0x202 송신 여부 | 내부 | §5 참조 — PC가 지금 이 프레임을 **한 개도 못 받고 있다** | 권장 |

단위는 무엇이든 좋다(rad/deg 자동 판별). 다만 **str 두 개(5,6)는 같은 단위**여야 한다.

## 2. 측정 설정

| 항목 | 값 | 이유 |
|---|---|---|
| 샘플 주기 | **1~10 ms** | PC 루프가 10ms다. 그보다 성기면 조향 응답이 뭉개진다 |
| 로깅 시작 | **PC launch보다 먼저** (go 인가 전) | 앞뒤로 남으면 잘라내면 되지만, 모자라면 못 채운다 |
| 로깅 종료 | 차 정지 후 | 종점 정지·watchdog 동작까지 담기 |
| 내보내기 | **CSV** (쉼표, 열 이름 한 줄) | `.mat`·`.idf`는 도구가 못 읽는다 |
| 파일 이름 | `dspace_<run 시각>.csv` (예: `dspace_0816_203352.csv`) | run 디렉터리 이름과 맞추면 짝이 안 헷갈린다 |
| 놓을 곳 | `~/FMA_ws/drive_logs/run_<시각>/` | rosbag과 같은 폴더 |

앞에 메타데이터 줄(`#`, 장비 정보 등)이 붙어 있어도 된다 — 도구가 건너뛴다.

## 3. 왜 counter인가 (시계 맞추기)

PC와 dSPACE는 **서로 다른 시계**를 쓴다. 로그 두 개를 그냥 겹치면 수십 초가 어긋난다.
counter는 PC가 매 틱 +1 해서 `0x100`으로 보내는 값이라, **양쪽 로그에 같은 번호가 찍힌다**
→ 그 번호로 붙이면 시계 오차도, 클럭 드리프트도 원리적으로 없다 (실측 정렬 오차 0.3ms).

counter를 못 찍으면 도구는 **v_ref 파형 상호상관**으로 맞춘다 (자체 시험 오차 1ms,
클럭 드리프트 180ppm까지 복원). 다만 그 run에 속도 변화(출발·정지·감속)가 몇 번은 있어야
한다 — 등속 주행만 있으면 못 맞춘다. 그래서 counter가 **필수**다.

## 4. 병합 실행 (PC에서)

```bash
source ~/FMA_ws/install/setup.bash
cd ~/FMA_ws
# 열 이름 확인
python3 src/adas_mgm/tools/dspace_merge.py --list-columns drive_logs/run_0816_203352/dspace_0816_203352.csv
# 병합 (이름 자동 추정 실패한 것만 --map으로)
python3 src/adas_mgm/tools/dspace_merge.py drive_logs/run_0816_203352 \
    --dspace drive_logs/run_0816_203352/dspace_0816_203352.csv \
    --map "t=Time,counter=CAN_RX/counter,v_act=Vehicle/v_meas,str_cmd=MPC/str_ref,str_act=Steer/str_meas"
```

출력 `<run>/analysis_dspace/`:

| 파일 | 내용 |
|---|---|
| `report.txt` | 정렬 품질 + **스테이트 구간표** (구간별 목표/실제 속도, 명령/실제 조향, 실현율, 횡오차) |
| `merged.csv` | 10ms 격자에 PC·dSPACE 신호를 나란히 — 다른 도구로 파고들 때 쓰는 원자료 |
| `1_speed_cmd_vs_actual.png` | 목표 v_ref vs 실제 v, 배경 = 스테이트 |
| `2_steer_cmd_vs_actual.png` | PC 기하 δ → MPC 출력 str → 실제 str, 배경 = 스테이트 |
| `3_per_state_summary.png` | 스테이트별 체류 시간·평균 조향·**실현 이득** |
| `4_trajectory_by_state.png` | 궤적을 스테이트 색으로 |

## 5. ⚠ 지금 PC는 dSPACE 상태를 하나도 못 받고 있다

2026-08-16 실차 run 16개 전부에서 `/vehicle/vector` 메시지가 **0개**다 — 즉 PC는
`0x200`(VEH_POSE)·`0x201`(VEH_VEL)·`0x202`(VEH_COMMIT)를 한 프레임도 받지 못했다.
그래서 지금은 **실제 str·v를 아는 곳이 dSPACE 로그밖에 없다**. 확인 요청:

1. dSPACE 모델이 `0x200`~`0x202`를 실제로 **송신**하는가 (PROTOCOL.md RX 절).
2. 송신한다면 주기·ID·바이트 배치가 PROTOCOL.md와 같은가 (little-endian, f32).
3. `0x202`(커밋)까지 와야 PC가 퍼블리시한다 — 셋 중 하나만 빠져도 0개가 된다.

이게 살아나면 PC 단독으로도 실제 str/v를 로깅하게 되고, dSPACE 로그는 **교차 검증**과
MPC 내부 신호용으로 남는다. 그전까지는 이 문서의 로깅이 유일한 실측 경로다.

## 참조

- CAN 프레임 정의: `PROTOCOL.md` (같은 폴더)
- 계약 배경: `CLAUDE.md` §3 (특히 조향 실현율 10~55% 제약 ③)
- 회피 시험 절차: `src/adas_mgm/RUNBOOK_avoid_field_test.md`
