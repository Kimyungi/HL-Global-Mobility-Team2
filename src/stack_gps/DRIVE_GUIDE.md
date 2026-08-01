# GPS 웨이포인트 주행 가이드 — 기록부터 주행·분석까지

**담당: 김윤기** · 산출물 "GPS 단독 주행" (마감 8/2)

전체 체인:

```
FST(위치) → stack_gps(경로 변환) → adas_mgm(판단) → bridge_dspace(CAN) → dSPACE MPC → 모터
  GPS 파트 ────────────────────┘        │                │                └ 손상민 담당
                                        └ 로깅: rosbag ──┴─ candump
```

주행에는 dSPACE 쪽(MPC, 손상민)이 차량에 올라가 있어야 한다 — **일정 사전 조율 필수.**
GPS 파트 단독으로는 §1~§4(기록·검증)까지 진행 가능.

---

## §0. 준비물 체크리스트

- [ ] 베이스: EVK + 안테나 + 베이스 PC + 라디오 (확정 좌표 지점 — `tools/base_station/README.md` 맨 위)
- [ ] 차량: FST-UEF9P **지붕 중앙 금속면**에 고정, 차량 PC USB 연결 (`/dev/ttyRover` 확인)
- [ ] 차량 PC: 라디오 USB (`/dev/ttyRadio` 확인), 저장소 최신(`git pull`), colcon 빌드 완료
- [ ] estop: 물리 비상정지 수단 + stack_estop (라이다 `/scan` 필요)
- [ ] 보조배터리 저부하 자동꺼짐 확인 (10분 유지 테스트)

베이스 가동 (베이스 PC — 상세는 `tools/base_station/INDUSTRIAL_PC_SETUP.md`):

```bash
python3 rtcm_server.py --radio /dev/ttyRadio
```

차량 PC — 라디오 RTCM을 로컬 TCP로 중계 (운용 내내 켜둠):

```bash
python3 ~/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py \
    --port /dev/ttyRadio --tcp-port 2101
# 이후 모든 도구는 rtcm_host=127.0.0.1 로 이 중계에 붙는다
# (FST 포트는 한 프로세스만 쓸 수 있어서 이 구조가 표준)
```

## §1. 웨이포인트 새로 따기 (차량 탑재 기록)

**도보 트랙 재사용 금지** — 사람 걸음의 급커브·흔들림이 차량 곡률로 부적합.
**차에 FST를 실은 채, 수동(조종기)으로 코스를 저속 주행하며 기록**한다.

```bash
cd ~/FMA_ws/src/stack_gps/tools/waypoints
python3 record_waypoints.py --host 127.0.0.1 --name course_1 --spacing 0.3
```

요령:

1. 기록 전 화면에 `FIXED`가 뜨는지 확인 (FLOAT면 하늘 시야·베이스 확인)
2. **시작점에서 3초 정지 후 출발** — 첫 점이 안정적으로 찍히게
3. 주행할 속도보다 **느리게** (보행 속도), 조향은 부드럽게 — 기록된 곡률이 곧 주행 경로다
4. 코스가 폐곡선이면 시작점으로 돌아와 종료 → **시작·끝 좌표 차이 = 그날 기록 품질** (3cm 이내면 합격)
5. `FIX 아님 — 기록 일시정지` 경고가 뜨었다면 그 구간은 점이 비어 있다 — 다시 따기
6. 여러 코스는 `--name`을 바꿔 별도 파일로

기록 직후 품질 검사:

```bash
python3 live_view.py --csv ../../waypoints/waypoints_course_1_*.csv   # 궤적 모양 눈검사
# 점 간격 튐/꺾임이 보이면 재기록
```

⚠ 웨이포인트는 **현재 베이스 좌표에 묶인다.** 베이스를 옮기면(재측량하면) 전부 재기록.

## §2. 벤치 리허설 (차량·현장 불필요, 실내 OK)

바퀴 굴리기 전에 소프트웨어 체인 전체를 dSPACE 시뮬레이터로 검증:

```bash
source ~/FMA_ws/install/setup.bash
ros2 launch bridge_dspace loopback_test.launch.py     # 터미널1: CAN 루프백+시뮬레이터
ros2 launch adas_mgm mgm.launch.py                    # 터미널2: MGM
ros2 run stack_gps stack_gps_node --ros-args \
    -p waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_course_1_<...>.csv
                                                      # 터미널3 (실내: 빈 경로+quality 0 발행이 정상)
ros2 topic hz /adas/target_ref                        # 터미널4: ~100Hz 나오면 체인 관통
ros2 topic echo /vehicle/vector --once                # 시뮬레이터 회신 확인
```

## §3. 현장 정지 검증 (주행 전 필수 관문)

차를 **트랙 시작점 근처, 진행 방향으로** 세우고:

```bash
# 차량 PC (라디오 중계는 §0에서 이미 가동 중)
ros2 run stack_gps stack_gps_node --ros-args \
    -p waypoint_csv:=<코스 CSV> -p rtcm_host:=127.0.0.1 \
    -p error_log_csv:=$HOME/FMA_ws/drive_logs/static_check.csv
```

| # | 확인 | 합격 기준 |
|---|---|---|
| 1 | `ros2 topic echo /perception/gps_path --once` | `fix_quality: 4` |
| 2 | points[0] | 차량 전방 수 m 내, x>0 (전방) |
| 3 | `tools/waypoints/live_view.py` | 현 위치가 트랙 위, 횡오차 cm급 |
| 4 | 차를 옆으로 1m 이동 | 횡오차가 ~1m로 상승 (변환 생존 증거) |
| 5 | FST 안테나 잠깐 가림 | quality 0 + points 비움 발행 (안전 동작) |

## §4. 로깅 — 주행 전 반드시 시작

**모든 주행은 기록한다** (문제없던 주행의 로그가 문제 주행의 기준선이 된다):

```bash
cd ~/FMA_ws/src/stack_gps/tools/drive_log
./record_drive.sh run1 <코스CSV경로>        # 주행 끝나면 Ctrl-C
```

무엇이 남나 (`~/FMA_ws/drive_logs/run1_<시각>/`):

| 파일 | 내용 | 이 로그로 답할 질문 |
|---|---|---|
| `bag/` | 인지(`/perception/*`, `/scan`) · 판단(`/adas/target_ref`) · **하위 피드백(`/vehicle/vector`)** · `/tf` · `/rosout` | "그 순간 각 계층이 뭘 보고/판단/보고했나" |
| `candump-*.log` | **CAN 원시 프레임 전량** (0x100~0x114 TX, 0x200~0x202 RX) | "CAN에 실제로 뭐가 나갔나, counter/watchdog은 정상이었나" |
| `meta.txt` | 일시, git 해시, 베이스 좌표, 트랙명 | "그때 어떤 코드·좌표였나" |
| 트랙 CSV 사본 | 그 주행의 지도 | 재현 |
| stack_gps `error_log_csv` | 매 틱 횡오차 | "추종 오차가 어디서 컸나" |

## §5. 첫 주행 프로토콜 — 전체 명령 순서

**"출발 명령어"는 따로 없다.** MGM은 estop 신호가 없거나 오래되면(250ms) 무조건
정지(v_ref=0)를 유지한다 (CLAUDE.md §5.7 워치독). 따라서 **마지막에 켜는
stack_estop이 곧 출발 스위치**다 — 켜는 순간 estop이 해제 상태로 발행되기
시작하면서 차가 출발한다. 이 원리를 이용해 아래 순서로 "장전 → 발사"한다.

사전: 비상정지 담당 1명 + 차 옆 동행 1명 배치. dSPACE v_ref 상한 보행속도(~1m/s).
stack_lane은 켜지 않는다 (차선 신뢰도 없음 → MGM이 waypoint 스테이트 자동 진입).

```bash
# ═══ 장전 단계 — 이 순서대로 다 켜도 차는 움직이지 않는다 (estop 워치독) ═══

# [베이스 PC]
python3 rtcm_server.py --radio /dev/ttyRadio                    # ① 보정 송출

# [차량 PC — 터미널 나눠서]
python3 ~/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py \
    --port /dev/ttyRadio --tcp-port 2101                        # ② 라디오→로컬 중계

source ~/FMA_ws/install/setup.bash                              # (각 터미널마다)
ros2 run stack_gps stack_gps_node --ros-args \
    -p waypoint_csv:=<코스CSV> -p rtcm_host:=127.0.0.1 \
    -p error_log_csv:=$HOME/FMA_ws/drive_logs/run1_lateral.csv  # ③ 경로 발행

ros2 launch bridge_dspace bridge.launch.py                      # ④ CAN 브릿지
ros2 launch adas_mgm mgm.launch.py                              # ⑤ 판단 (10ms 루프)

ros2 topic echo /vehicle/vector --once                          # ⑥ dSPACE 살아있나 (회신 오면 OK)
ros2 topic echo /perception/gps_path --once | grep fix_quality  # ⑦ quality: 4 확인

cd ~/FMA_ws/src/stack_gps/tools/drive_log
./record_drive.sh run1 <코스CSV>                                # ⑧ 블랙박스 시작

# ═══ 출발 — 전원 준비 확인 후 이 한 줄이 곧 출발이다 ═══
ros2 run stack_estop stack_estop_node                           # ⑨ ★출발★

# ═══ 정지 방법 (급한 순서대로) ═══
#  1) 물리 비상정지 (항상 최우선)
#  2) ⑨ 터미널에서 Ctrl-C → 250ms 내 estop 워치독이 정지시킴 (소프트웨어 정지 레버)
#  3) 코스 끝 도달 시 자연 정지
```

- 첫 목표는 **"직선 10m 추종"** — 성공 시 곡선 → 전체 코스 → 속도 단계 상승
- 이상 시 정지시키더라도 **로깅(⑧)은 끄지 말 것** — 사고 순간이 제일 귀한 데이터
- FIX가 풀리면 stack_gps가 빈 경로+quality 0 발행 → MGM 정지 — **주행 중 차가 서면
  우선 FIX부터 의심**
- ⑨를 켰는데 출발 안 하면: stack_estop이 장애물을 보고 있는지(`ros2 topic echo
  /perception/estop`), 라이다 `/scan`이 나오는지 확인

## §6. 주행 후 분석

```bash
ros2 bag info ~/FMA_ws/drive_logs/run1_*/bag          # 토픽·메시지 수 개요
ros2 bag play ~/FMA_ws/drive_logs/run1_*/bag          # 재생 → RViz/live_view로 그날 상황 재현
```

문제 유형별로 볼 곳:

| 증상 | 1차로 볼 로그 | 보는 법 |
|---|---|---|
| 경로를 못 따라감 (횡오차 큼) | `error_log_csv`, `/vehicle/vector` vs `/perception/gps_path` | 오차가 GPS(입력)인지 제어(추종)인지 분리 — gps_fix 궤적을 트랙에 겹쳐 그리기 |
| 갑자기 정지 | `/rosout` → `/perception/estop` → `gps_path.fix_quality` 순서 | 정지 요구의 출처 계층 확인 |
| 차가 아예 안 움직임 | candump: 0x100 counter 증가? → `/vehicle/vector` 회신? | watchdog(30ms) 타임아웃 여부, CAN 단선 |
| 휘청거림/진동 | `/adas/target_ref` (판단 흔들림?) vs candump str_ref (제어 흔들림?) | 같은 시각축으로 비교 — 원인 계층 특정 |
| CAN 레벨 의심 | `candump-*.log` | `canplayer -I <log>` 재생, 또는 텍스트로 counter 연속성 검사 |

분석 후: 원인·조치를 커밋 메시지나 `drive_logs/<run>/notes.md`로 남기고, bag은 용량 크므로 git에 올리지 않는다 (`drive_logs/`는 gitignore).

## 트러블슈팅 (주행 특화)

| 증상 | 점검 |
|---|---|
| 기록 중 FIXED가 안 뜸 | 베이스 라디오 B/s → 차량 중계(127.0.0.1) B/s → FST 하늘 시야 순 |
| gps_path가 빈 배열 | fix stale(1.5s) — FST 연결/중계 확인. quality 4인데 비면 waypoint_csv 경로 오타 |
| 첫 점이 차량 뒤(x<0) | 차량이 트랙 진행 방향과 반대로 서 있음 — 방향 맞춰 재배치 |
| target_ref는 나오는데 바퀴 무반응 | bridge↔dSPACE: candump로 TX 확인 → 손상민 쪽 수신 확인 |
| 곡선에서 크게 이탈 | 기록 속도가 너무 빨랐거나 spacing 과대 — 코스 재기록(더 느리게, spacing 0.2) |
