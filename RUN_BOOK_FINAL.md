# RUN_BOOK_FINAL

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
신호등 모듈은 새 한라대 맵의 **zone [3] 안에서만 켜집니다**. 같은 구간에서 GPS 단독주행을 사용하며, 구간 밖에서는 신호등 모듈 OFF가 정상입니다.

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

## 3. 주행 런쳐 코드 — v2_main 전체 코스

**터미널 1**에서 GPS 상태 표시만 Ctrl-C로 종료한 뒤 실행합니다.

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
scripts/v2 prepare \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  start_waypoint:=01 end_waypoint:=07 \
  parking_enabled:=true parking_zone_entry_active:=true t_reference_enabled:=true \
  avoidance_enabled:=true avoid_zone_only:=true \
  waypoint_avoid:=true avoid_v2_enabled:=false \
  zone_enter_confirm_samples:=5 zone_exit_confirm_samples:=5 \
  traffic_enabled:=true rviz:=true v_base:=2.0
```

현재 `integration/v2_main`의 새 한라대 CSV와 전체 미션을 실행합니다.
주행 순서는 시작부터 끝까지 **01 → 03 → 04 → 05 → 07**이며, 03의 T 주차·04의 회피/평행 주차·zone [3]의 Traffic/GPS 단독주행을 포함합니다.
02는 다른 시작 분기, 06은 다른 종료 분기이므로 01~07을 모두 순서대로 주행하는 구성은 아닙니다.
경로를 바꾸려면 `start_waypoint`와 `end_waypoint`를 수정합니다.
센서·상위 제어·RViz와 실제 CAN 송신을 시작하고, 4번 출발 인가를 기다립니다.
**이 터미널은 실행 상태로 둡니다.**

정상 런처 시작 예시:

```text
[v2 drive] route: 01 -> 03 -> 04 -> 05 -> 07
[v2 drive] avoid planner: Waypoint_Avoid_PR103; backend=stack_avoid.waypoint_planner; zone_only=true
[v2 drive] logs: /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main/drive_logs/v2_20260916_150000_123456; waiting for explicit go
```

이 메시지는 런처 시작을 뜻합니다. 센서 준비 완료는 **2번의 상태 화면**에서 확인합니다.
CSV 파일은 `waypoints_halla_20260916_path_XX.csv`입니다.
경로 CSV와 Zone은 이 PC의 `src/stack_gps/waypoints/halla_route_sequence.yaml`을 기준으로 읽습니다.
세션별 경로 목록 `route_selected.yaml`과 로그는 위 작업 폴더의 `drive_logs/v2_날짜_시간/` 아래에 생성됩니다.

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
