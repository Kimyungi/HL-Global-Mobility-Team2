# GPS 웨이포인트 주행 가이드

**담당: 김윤기** · 산출물 "GPS 단독 주행" (8/2)

```
FST(위치) → stack_gps(경로) → adas_mgm(판단) → bridge(CAN) → dSPACE MPC(손상민) → 모터
```

- 주행하려면 dSPACE(손상민)가 차량에 있어야 한다. GPS 파트 단독으로는 PART A까지 가능.
- 모든 ros2 터미널은 워크스페이스가 source돼 있어야 한다 (차량 PC `.bashrc`에 등록돼
  있으면 자동 — 안 되면 `source ~/FMA_ws/install/setup.bash`).

## 터미널 지도 — 이 문서의 모든 명령은 아래 이름으로 부른다

| 이름 | 어디서 | 역할 | 켜는 때 | 끄는 때 |
|---|---|---|---|---|
| **B1** | 베이스 PC | 보정 송출 (라디오) | 현장 도착 직후 | 철수할 때 |
| **V1** | 차량 PC | 라디오→로컬 중계 | 현장 도착 직후 | 철수할 때 |
| **V2** | 차량 PC | stack_gps (경로 발행) | 도착 후 상시 | 철수할 때 (⚠ 코스 기록 중엔 꺼야 함) |
| **V3** | 차량 PC | CAN 브릿지 | 도착 후 상시 | 철수할 때 |
| **V4** | 차량 PC | MGM (판단) | 도착 후 상시 | 철수할 때 |
| **V5** | 차량 PC | 주행 로깅 (블랙박스) | 매 주행 직전 | 매 주행 끝 (Ctrl-C) |
| **V6** | 차량 PC | stack_estop = **출발 스위치** | 출발 순간 | 세우고 싶을 때 (Ctrl-C) |
| (임시) | 아무 데나 | `ros2 topic echo` 등 확인용 | 필요할 때 | 확인 후 닫아도 됨 |

"상시"는 켜두고 잊는 것. 주행을 여러 번 반복해도 **V5, V6만 반복해서 켜고 끈다.**

---

# PART A — 준비 (해당될 때만, 한 번씩)

## A1. 벤치 리허설 — 최초 1회, 실내 OK (코드 크게 바뀌면 다시)

dSPACE 시뮬레이터로 소프트웨어 체인 관통 확인. 임시 터미널 4개, 끝나면 전부 Ctrl-C 후 닫는다.

```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0

# 임시1
ros2 launch bridge_dspace loopback_test.launch.py
# 임시2
ros2 launch adas_mgm mgm.launch.py
# 임시3  (실내라 fix 없음 → 빈 경로 발행이 정상)
ros2 run stack_gps stack_gps_node --ros-args -p waypoint_csv:=<아무 코스 CSV>
# 임시4 — 합격 판정
ros2 topic hz /adas/target_ref          # ~100Hz 나오면 체인 관통
ros2 topic echo /vehicle/vector --once  # 시뮬레이터 회신 확인
```

## A2. 코스 웨이포인트 기록 — 코스당 1회 (베이스를 재측량하면 전부 다시)

전제: B1·V1 가동 중 (아래 PART B의 B1·V1과 동일 명령). **V2(stack_gps)는 꺼둘 것**
— FST 포트는 한 프로세스만 쓸 수 있다.

```bash
# 임시 터미널 (차량 PC) — 차에 FST 실은 채 수동 저속 주행하며 기록
cd ~/FMA_ws/src/stack_gps/tools/waypoints
python3 record_waypoints.py --host 127.0.0.1 --name course_1 --spacing 0.3
# 기록 끝나면 Ctrl-C → 터미널 닫아도 됨
```

요령: 화면에 FIXED 확인 후 출발 / 시작점 3초 정지 / 주행 속도보다 느리게, 조향 부드럽게 /
폐곡선이면 시작점 복귀 후 종료 (시작·끝 차이 3cm 이내 = 합격) / "FIX 아님" 경고가 떴던
run은 버리고 다시.

기록 후 품질 눈검사 (임시 터미널): `python3 live_view.py --csv ../../waypoints/waypoints_course_1_*.csv`

## A3. 현장 정지 검증 — 현장 첫날 1회 (이후 이상할 때만)

전제: B1·V1·V2 가동 (PART B 순서로 켠 상태). 차를 **트랙 시작점, 진행 방향으로** 세우고
임시 터미널에서:

| # | 명령/행동 | 합격 |
|---|---|---|
| 1 | `ros2 topic echo /perception/gps_path --once` | `fix_quality: 4` |
| 2 | 위 출력의 points[0] | 전방 수 m 내, x>0 |
| 3 | `python3 live_view.py` | 현 위치가 트랙 위, 횡오차 cm급 |
| 4 | 차를 옆으로 1m 이동 | 횡오차 ~1m로 상승 |
| 5 | FST 안테나 잠깐 가림 | quality 0 + 빈 points (안전 동작) |

---

# PART B — 주행 날 운용

## B-1단계: 상시 터미널 켜기 (현장 도착 시 1회 — 이후 하루 종일 유지)

```bash
# B1 [베이스 PC] — 안테나는 확정 좌표 지점에!
cd ~/FMA_ws/src/stack_gps/tools/base_station
python3 rtcm_server.py --radio /dev/ttyRadio

# V1 [차량 PC]
python3 ~/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py \
    --port /dev/ttyRadio --tcp-port 2101

# V2 [차량 PC]
# 1. V2 재시작 (새 코드 반영 — 기존 stack_gps 터미널 Ctrl-C 후)
ros2 run stack_gps stack_gps_node --ros-args \
    -p waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_course_1_20260801_170519.csv \
    -p rtcm_host:=127.0.0.1 \
    -p error_log_csv:=$HOME/FMA_ws/drive_logs/lateral_$(date +%m%d_%H%M).csv

# V3 [차량 PC]
ros2 launch bridge_dspace bridge.launch.py

# V4 [차량 PC]
ros2 launch adas_mgm mgm.launch.py
```

켜고 나서 확인 2줄 (임시 터미널 — 닫아도 됨):

```bash
ros2 topic echo /vehicle/vector --once                          # dSPACE 회신 = 연결 OK
ros2 topic echo /perception/gps_path --once | grep fix_quality  # 4 = RTK FIXED
```

**IMU 헤딩 융합 (2026-08-01 도입, 08-03 부호 실측 완료):** V2 상태 로그에 2초마다
`IMU:150Hz 정렬 +123°`가 나온다. 전원 인가 직후에는 `미정렬(직진 주행 필요)` —
**첫 주행에서 몇 초 직진하면 COG로 자동 정렬**되고, 이후 헤딩 소스가 `융합`으로
바뀌며 정지·저속에서도 절대 헤딩이 유지된다. 첫 출발만 "트랙 위 진행방향 정렬"
필요(접선 폴백). HFI-A9의 yaw 부호(시계+)는 노드 기본값에 반영돼 있어(`imu_yaw_sign=-1.0`)
따로 만질 것 없음. IMU를 뗐거나 문제면 `-p imu_port:=off`로 기존 COG/접선 동작 그대로.

**여기까지는 전부 켜도 차가 움직이지 않는다** — estop 신호가 없으면 MGM이 정지를
유지하기 때문 (§5.7 워치독). 이게 "장전만 된" 안전 상태다.

## B-2단계: 주행 루프 (run마다 반복 — 이 두 터미널만 만진다)

```bash
# V5 [차량 PC] — 블랙박스 시작
cd ~/FMA_ws/src/stack_gps/tools/drive_log
./record_drive.sh run1 $HOME/FMA_ws/src/stack_gps/waypoints/waypoints_course_1_20260801_170519.csv


# V6 [차량 PC] — ★ 이 줄이 곧 출발이다 ★  (비상정지 담당·동행 준비 확인 후!)
# GPS 단독 주행 (라이다 없음 — 현재 구성):
python3 ~/FMA_ws/src/stack_gps/tools/drive_log/manual_go.py
# 정식 구성 (라이다 장착 시) — 위 대신 이것. 둘 동시 실행 절대 금지(신호 충돌):
# ros2 run stack_estop stack_estop_node
```

⚠ manual_go는 돌발 장애물 자동 정지가 없다 — 물리 비상정지 담당 배치·저속·개활지 필수.

**세우는 법** (급한 순서): ① 물리 비상정지 ② V6에서 Ctrl-C (250ms 내 정지) ③ 코스 완주 자연 정지

run 종료 처리: V6 Ctrl-C(이미 세웠으면 생략) → V5 Ctrl-C(bag 마감 몇 초 대기) → 끝.
다음 run은 V5·V6만 다시 켠다. 코스를 바꿀 때만 V2를 Ctrl-C 후 새 CSV로 재시작.

첫 주행 규칙: dSPACE v_ref 상한 보행속도(~1m/s) / 첫 목표 "직선 10m" → 곡선 → 전체
코스 → 속도 상승 / 사고·이상 순간에도 **V5는 끄지 말 것** / V6를 켰는데 출발 안 하면:
manual_go 구성은 `/perception/gps_path` fix_quality·`/adas/target_ref` 발행 확인,
stack_estop 구성은 차 앞 장애물(estop이 보고 있음)과 `/scan` 유무 확인.

---

# PART C — 주행 후 분석

로그 위치: `~/FMA_ws/drive_logs/<run>_<시각>/` (bag + candump CAN 원시 + meta + 트랙 사본)

```bash
ros2 bag info  ~/FMA_ws/drive_logs/run1_*/bag
ros2 bag play  ~/FMA_ws/drive_logs/run1_*/bag    # RViz/live_view로 그날 재현
```

| 증상 | 볼 로그 | 요점 |
|---|---|---|
| 경로 이탈 (횡오차 큼) | error_log_csv + `/perception/gps_fix` vs 트랙 | 오차가 GPS(입력)인지 제어(추종)인지 분리 |
| 갑자기 정지 | `/rosout` → `/perception/estop` → `gps_path.fix_quality` | 정지 요구의 출처 계층 |
| 아예 안 움직임 | candump 0x100 counter → `/vehicle/vector` | watchdog/CAN 단선 |
| 휘청거림 | `/adas/target_ref` vs candump str_ref | 판단이 흔들리나 제어가 흔들리나 |

분석 결론은 `drive_logs/<run>/notes.md`로 남긴다. bag은 git에 올리지 않는다(gitignore 됨).

## 트러블슈팅

| 증상 | 점검 |
|---|---|
| 기록/주행 중 FIXED 안 뜸 | B1 통계 B/s → V1 통계 B/s → FST 하늘 시야 순서로 상류부터 |
| gps_path가 빈 배열 | quality 0이면 fix stale(FST/중계 확인), 4인데 비면 CSV 경로 오타 |
| 첫 점이 차량 뒤(x<0) | 차가 트랙 진행 방향과 반대 — 돌려 세우기 |
| target_ref 나오는데 바퀴 무반응 | candump로 CAN TX 확인 → dSPACE 쪽(손상민) 수신 확인 |
| 곡선에서 크게 이탈 | 기록이 너무 빨랐음 — 더 느리게, --spacing 0.2로 재기록 |
| 상태 로그 `IMU:없음` | /dev/ttyUSB_IMU 존재·USB 연결 확인 (udev가 자동 명명) |
| 융합 정렬 후에도 선회에서 이탈 | IMU 부호/장착 의심 — `tools/imu_sign_check.py`로 재판정. HFI-A9 실측(08-03): yaw 시계+ → 노드 기본 `imu_yaw_sign=-1.0` 반영됨. **IMU 재장착·교체 때만 재실행** (수평면 회전 장착은 융합이 자동 흡수, 상하 뒤집힘만 부호가 바뀜). 로그의 imu_yaw_deg는 원시값이라 좌회전 시 감소가 정상, 융합 heading_deg는 증가가 정상 |
| IMU CRC오류 다수 | USB 케이블·전원 노이즈 — 케이블 교체 |
