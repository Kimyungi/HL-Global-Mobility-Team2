# RUN_BOOK_HALLA_FINAL

현재 상태 머신: **스테이트 v09.17** ([명세·상태 점검](docs/STATE_V09_17.md)).

실차에 연결하는 **이 PC** 기준입니다. 모든 명령은 아래 작업 폴더의 `scripts/v2`를 사용합니다.
출력은 **정상 실행 시의 주요 부분을 발췌한 예시**이며, PID·날짜·수신량은 달라집니다.
각 명령 앞에 출력되는 공통 설치 점검 메시지는 예시에서 생략했습니다.

## 1. GPS 연결 코드

**터미널 1**에서 실행합니다.

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
scripts/v2 gps-start
```

정상 수신 예시:

```text
GPS 수신 상태를 1초마다 표시합니다. Ctrl-C: 표시만 종료, GPS 연결 유지.
이후 scripts/v2 prepare … 실행 / GPS 완전 종료: scripts/v2 gps-off
RTCM    560 B/s | FIXED (FIX=4) | 누적 11200 B | GPS PID=12345
RTCM    580 B/s | FIXED (FIX=4) | 누적 11780 B | GPS PID=12345
```

`FIXED (FIX=4)`를 확인합니다. **Ctrl-C는 표시만 종료**하며 GPS/RTCM 연결은 유지됩니다.
이후 주행 런처를 종료해도 GPS 연결은 유지됩니다.

## 2. 모든 센서 초기화 및 연결 확인 코드

현재 구조에서는 **3번 런처가 라이다 4대·차선 카메라 등 주행 센서를 초기화**하고,
1번에서 연결한 GPS를 재사용합니다. 별도의 전체 센서 초기화 명령은 없습니다.
현재 halla_0919 코스에서는 zone [3]의 신호등/GPS 구간 진입에 따라 감지 모듈이 활성화됩니다.

**터미널 2 — 런처 실행 전 장치 연결·접근 권한 확인:**

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
for device in /dev/ttyRover /dev/ttyRadio \
              /dev/lidar_front /dev/lidar_rear /dev/lidar_left /dev/lidar_right; do
  if [ -c "$device" ] && [ -r "$device" ] && [ -w "$device" ]; then
    printf 'OK: %s\n' "$device"
  else
    printf 'CHECK: %s (연결 또는 접근 권한 확인 필요)\n' "$device"
  fi
done
```

정상 예시입니다. 이 결과는 장치 접근 확인이며 실제 데이터 수신 확인은 아래에서 수행합니다.

```text
OK: /dev/ttyRover
OK: /dev/ttyRadio
OK: /dev/lidar_front
OK: /dev/lidar_rear
OK: /dev/lidar_left
OK: /dev/lidar_right
```

라이다 링크가 없거나 권한 문제가 있는 경우에만, **주행 런처가 종료된 상태**에서 복구합니다.
GPS 연결은 유지됩니다.

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
sudo /usr/bin/python3 scripts/v2_recover_lidars.py --apply
```

복구 성공 시 마지막 출력:

```text
LIDAR_LINKS_READY: four identities, links and access permissions verified. Restart the integrated launch and verify all four scan topics before go.
```

이어서 **터미널 2**에서 수신 상태 화면을 열어 둡니다. **3번을 실행한 뒤** 아래 값이 갱신됩니다.

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
scripts/v2 state
```

라이다 4대·차선 카메라·GPS FIXED가 모두 준비된 경우의 주요 필드 예시:

```yaml
camera_available: true
gps_fixed_ready: true
lidar_ready: true
lidar_missing_topics: []
start_ready: true
go_authorized: false
revised_v2: true
traffic_zone_active: false
```

`lidar_ready`는 필수 라이다 4대의 유효한 최신 스캔, `camera_available`은 실제 영상 수신,
`gps_fixed_ready`는 최신 FIXED GPS와 경로 준비를 나타냅니다.
출발 준비 조건은 **라이다 4대 정상 AND (카메라 영상 수신 OR GPS FIXED 준비)**입니다.
차선 검출·실제 이동 가능 여부는 별도로 판단합니다.
확인 후 터미널 2에서 Ctrl-C를 누르면 상태 표시만 종료됩니다.

## 3. 주행 런쳐 코드 — halla_0919 전체 코스 (PR #119)

초기 차량 전방을 선택한 경로의 진행 방향에 맞춥니다. v2는 첫 유효한 RTK FIXED
위치에서 가장 가까운 웨이포인트의 진행 방향과 신선한 IMU yaw로 초기 헤딩을
한 번 정렬합니다. 초기 정렬을 위한 별도 직진 주행은 필요하지 않습니다.
`웨이포인트 초기 헤딩 정렬: idx …, heading …°` 로그를 확인합니다.
이후에는 IMU 회전량을 추적하고 주행 중 GPS COG로 보정합니다. 이 초기 방향은
실측 방위가 아니라 차량 배치에 대한 가정입니다. 주행 중 IMU 재연결이나 정렬
손실이 발생하면 경로 방향으로 강제 복귀하지 않고 기존 COG 재정렬을 기다립니다.

### 3-1. 실행 시점

매번 새로운 주행을 시작할 때 실행합니다. 1번에서 GPS FIXED를 확인하고,
2번의 장치 점검을 끝낸 뒤 **터미널 1**에서 GPS 상태 표시를 Ctrl+C로 종료합니다.
GPS/RTCM 연결은 유지됩니다. 차량은 선택할 경로의 진행 방향으로 배치합니다.
이미 주행 런처가 실행 중이면 아래 5번 순서로 종료한 뒤 실행합니다.

### 3-2. 시작 경로와 속도를 질문받아 실행 — 기본 사용법

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
scripts/v2 prepare REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX
```

명령 실행 후 같은 터미널에 **시작 경로 번호 → 일반 주행 목표속도(m/s)** 순서로 입력합니다.
각 값을 입력하고 Enter를 누릅니다. 다음은 경로 01, 일반 주행 2m/s를 선택한 예시입니다.

```text
시작 CSV 선택: halla_0919_path_01.csv ~ 07.csv
시작 경로 번호 (기본값 없음): 01
일반 주행 목표속도 v_base [m/s] (기본값 없음): 2.0
[v2] 일반 주행 목표속도: 2.0 m/s
[v2] 시작 경로: 01 / 종료 경로: 07
[v2 drive] general driving v_base: 2 m/s
```

입력한 값은 **이번 런처 실행에만 적용**됩니다. 다음 실행에서는 다시 질문하며,
빈 입력으로 이전 속도나 0.5m/s를 재사용하지 않습니다. 0·음수·NaN·무한대는 거부합니다.
`v_base`는 출발 순간의 속도가 아니라 **일반 주행 구간의 목표속도**입니다.
회피 목표속도는 별도 설정인 1m/s이며, T 주차는 주차 모듈의 목표속도를 사용하되
절댓값이 `v_base`를 넘지 않도록 제한됩니다.

입력을 마치면 센서·CAN·RViz가 실행됩니다. **이 명령만으로 출발 인가가 나가지는 않습니다.**
터미널 1은 켜 두고, 터미널 2의 `scripts/v2 state`에서 준비 상태를 확인한 뒤
4번의 `scripts/v2 go`를 실행합니다.

### 3-3. 질문 없이 경로와 속도를 직접 지정하는 방법

값을 미리 정했거나 자동 실행할 때는 아래처럼 두 값을 명시합니다.
`01`과 `2.0`을 이번 주행에 사용할 경로와 속도로 바꿉니다.

```bash
scripts/v2 prepare \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  start_waypoint:=01 end_waypoint:=07 v_base:=2.0
```

경로만 지정하면 속도를 묻고, 속도만 지정하면 경로를 묻습니다.
터미널 입력이 없는 자동 실행에서는 `start_waypoint`와 `v_base`를 둘 다 지정해야 합니다.
종료 경로를 06으로 선택하려면 `end_waypoint:=06`을 추가합니다.
`prepare`와 `drive`에는 같은 입력 절차가 적용됩니다.

### 3-4. 실제 적용되는 CSV와 주행 순서

현재 `prepare/drive`는 PR #119의 `halla_0919` 경로를 사용합니다.

- 통합 원본: `src/stack_gps/waypoints/halla_0919.csv` (901점, 확인용)
- 실행 CSV: `halla_0919_path_01.csv`, `halla_0919_path_02.csv`,
  `halla_0919_path_03.csv`, `halla_0919_path_04.csv`, `halla_0919_path_05.csv`,
  `halla_0919_path_06.csv`, `halla_0919_path_07.csv` (모두 같은 waypoints 폴더)
- 주차 후진 후보: `src/stack_parking/config/parking_ref_01.csv`, `parking_ref_02.csv`.
  PR #120의 최신 `parking_ref_01.csv`, `parking_ref_02.csv`만 주차 후보로 사용합니다.
  PR #116의 `parking_waypoint*.csv`는 보관용이며 v2 실행에서 제외합니다.
  별도 `parking` 프로파일은 실행 전에 거부됩니다.

시작 경로는 01~07 중 선택하며, 생략하면 터미널에서 질문합니다. 기본 종료는 07,
06에서 시작하면 06입니다. 전체 순서는 `01 또는 02 → 03 → 04 → 05 → 06 또는 07`입니다.
03에서 시작하면 `03 → 04 → 05 → 07`만 실행합니다. 06과 07은 대체 출구입니다.
03 주차 완료·전진 복귀 후 04로 진행합니다. 일반 주행 목표속도 `v_base`는 매 실행마다 직접 입력합니다. 숨은 0.5m/s 기본값은 없습니다.

03 끝점과 주차 후보 시작점 차이는 약 0.080m입니다. CSV 좌표는 PR 원문을 유지하며,
기존 전진 종점 도달 허용거리 0.14m 이내에서 경로 로딩을 허용합니다.
turn zone [3]은 PR #119의 기존 YAML 동작 범위를 유지합니다. CSV의 zone_id/inside_zone과
실행 중 신호등·GPS 전용 구역은 동일 개념이 아닙니다. 03은 YAML 기준 idx 28~116이며
CSV 표기는 idx 75~116입니다. 05(idx 276~316), 06(idx 0~25), 07(idx 0~36)의 YAML 신호등 구역도 유지합니다.
커브 끝 +1.5m까지의 주차 후보 검사/RViz 표시는 유지합니다.
변경 사항은 런처를 Ctrl+C로 종료 후 다시 실행해야 적용됩니다.

센서·상위 제어·RViz와 실제 CAN 송신을 시작하고, 4번 출발 인가를 기다립니다.
**이 터미널은 실행 상태로 둡니다.**

정상 런처 시작 예시:

```text
[v2 drive] route: 01 -> 03 -> 04 -> 05 -> 07
[v2 drive] avoid planner: Waypoint_Avoid_PR103; backend=stack_avoid.waypoint_planner; zone_only=true
[v2 drive] logs: /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main/drive_logs/v2_20260916_150000_123456; waiting for explicit go
```

이 메시지는 런처 시작을 뜻합니다. 센서 준비 완료는 **2번의 상태 화면**에서 확인합니다.
GPS 좌표 원점은 선택한 시작 CSV의 첫 위도·경도이며 주차·회피도 같은 원점을 사용합니다.
경로 CSV와 Zone은 이 PC의 `src/stack_gps/waypoints/halla_route_sequence.yaml`을 기준으로 읽습니다.
세션별 경로 목록 `route_selected.yaml`과 로그는 위 작업 폴더의 `drive_logs/v2_날짜_시간/` 아래에 생성됩니다.

ESTOP은 상위 제어에서 진입하고 실제 정차가 연속 6초 유지된 후 후방이 확인되면 1m 후진합니다.
후진 후 실제 정차를 확인해야 복귀하며, 회복 입력 이상 시 정지를 유지합니다.

## 4. 주행 인가 코드

센서 준비를 확인한 뒤 **터미널 2**에서 상태 표시를 Ctrl-C로 종료하고 실행합니다.

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
scripts/v2 go
```

카메라·GPS·라이다가 모두 준비된 상태에서 인가에 성공한 예시:

```text
카메라 영상=True | GPS FIXED=True | MGM=True
라이다 정상=True | 미수신/무효=[]
출발 인가 완료 (MGM 확인). 실제 이동은 선택된 경로와 제어 조건에 따릅니다.
```

마지막 메시지가 MGM의 인가 확인입니다. 실제 이동은 현재 상태와 유효한 주행 경로에 따라 결정됩니다.

## 5. 주행 종료 및 다음 주행의 경로·속도 변경

1. **터미널 2**에서 `scripts/v2 stop`을 실행해 출발 인가를 해제하고 차량 정차를 확인합니다.
2. **터미널 1**에서 Ctrl+C로 주행 런처를 종료하고 종료 메시지를 확인합니다.
3. 다음 주행은 3-2의 `scripts/v2 prepare ...` 명령을 다시 실행해 경로와 속도를 새로 입력합니다.
4. 센서 준비 확인 후 4번의 `scripts/v2 go`로 새 주행을 인가합니다.

`go`를 다시 보내는 것만으로 경로·속도가 바뀌지는 않습니다. 실행 중인 런처의
경로·속도를 바꾸려면 위 순서로 재시작합니다. **Ctrl+Z는 종료가 아니라 일시정지**이므로
런처 종료에 사용하지 않습니다. 이전 프로세스가 남아 통신 자원을 점유할 수 있습니다.
GPS 연결도 완전히 종료하려면 주행 런처 종료 후 `scripts/v2 gps-off`를 실행합니다.

## PR #117 장애물 회피 전용 런처

손상민의 PR #117 (`48a63ba4c63cdbee79cbea5de15aad7ea18e82ac`)에서
`obstacle_waypoint.csv`를 원문 그대로 가져왔다. 116점, 약 28.3m이며
경로 01의 idx=20 (`state=4`, 출발점에서 약 4.92m)에서 회피 진입한다.
단일 경로 종료 뒤 다음 CSV로 넘어가지 않는다. 주차 구역·주차 미션은 없다.
GPS와 웨이포인트 회피 플래너는 같은 CSV와 좌표 원점을 사용한다.
기본 차선/GPS 전환 정책과 회피 후 GPS 복귀 조건은 기존 v2와 같다.

기존 주행 런처를 종료한 뒤 별도 터미널에서 실행:

```bash
scripts/v2 obstacle REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX
```

경로 01은 자동 선택된다. 준비 확인 후 `scripts/v2 go`로 출발 인가하며,
`scripts/v2 stop`으로 인가를 해제한다. 초기 헤딩은 웨이포인트 방향을 사용하므로
차량을 경로 진행 방향(-135도)에 맞춰 놓고 시작한다.
`scripts/v2 prepare`는 halla_0919 전체 코스를 사용한다. `scripts/v2 obstacle`만 PR #117 단일 회피 시험 경로를 유지한다.
새 런처도 차량 마커 TF/갱신 수정과 누적 SLAM 지도 제외가 적용된 통합 RViz를 쓴다.

2026-09-17 회피 preview는 1.5m로 설정한다. 생성된 경로와 전방 1.5m 원의
교점으로 계산한다. 진입·복귀 길이는 기존 2.5m를 유지한다.

### 2026-09-17 소형 RC 주차 시험: 내부 정지 조건 변경

사용자 지시에 따라 선택 이후 주차 내부의 전·후방 장애물/선택 경로 충돌 정지,
입력 나이·센서 간 시각 차이 정지, 로컬 소유권·경로 식별 변경·시간 역행·역방향 이동
FAULT를 제거했다. 기존 시작점 및 추종 거리·방향 검사 제거도 유지한다.
선택 전에는 LiDAR로 후보를 고르고, 선택 이후에는 그 경로와 최신 수신 위치·차속을 쓴다.
T 어댑터의 reference/preparation_stamp는 이 모드에서 입력 취득 시각이 아니라
재계산 시각이다. 첫 유효 위치·차속이 없거나 값이 비유한 경우에는 제어점을 만들지 않는다.
상위 MGM의 ESTOP·운전자 정지·CAN 및 최종 출력 제어는 유지한다.
초기 선택·방향 전환 정차, 주차 완료 판정/10초 대기, 복귀 완료 정지는 유지한다.
요청 목록에서 제거 대상이 아니었던 경로 끝의 `path_end_without_rear_wall`도 유지한다.
변경은 런처 재실행부터 적용하며, 실행 중인 차량에 출발/해제 명령은 보내지 않는다.

### Zone 판정 우선순위 — 2026-09-18

현재 차량 station에 해당하는 zone이 있으면 preview에만 걸린 다른 GPS zone을
추가 활성화하지 않는다. 정지·회피·가속 구간도 현재 station 판정을 우선한다.
현재 station에 zone이 없을 때만 preview의 GPS zone 선행 진입을 허용한다.
실제로 현재 station에서 겹치는 zone들은 그대로 전달하며, preview 제어점 위치는 바꾸지 않는다.
동일 경로 설정 안에서 같은 zone_id를 중복 정의하는 것은 기존처럼 설정 오류로 처리한다.
