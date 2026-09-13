# Integration v2 통합 자율주행 시작 — 한라대학교

> 2026-09-13 주차 변경: [즉시 주차 진입](../../docs/MGM_PARKING_ENTRY.md).
> Zone 진입 즉시 PARKING이며 탐색 중 현재 CSV의 GPS를 추종한다. ready 후 주차 제어로 인계한다. 미완료 종점은 실패 처리 후 다음 CSV로 자동 전환한다(단일 CSV는 FINISH).

> **현재 설정:** 일반 주행 회피·LiDAR E-stop ON, Mission ACTIVE의 탐색·실행 중 둘 다 제외.
> 신호등 노출 보정은 `-2`이며 `traffic_exposure_compensation:=0`으로 기본 자동 노출 수준을 선택할 수 있다.
> 노출 변경은 launch 재시작에 적용하며 차선 카메라 노출에는 영향을 주지 않는다.

2026-09-12 · `integration/v2_main` · 기준 `8697bcb` + 연속 경로 확장·PR #86 데이터 (미커밋) · **실차 검증 전 런북**

차선·GPS·회피·긴급정지·신호등/정지선·주차를 v2 통합 launch 하나로 실행하는 절차다.
기존 main의 `~/FMA_ws/install`과 v2의 `install_v2`를 분리한다.
전용 진입점은 `./scripts/v2 vehicle`, 실제 launch는 `REAL_VEHICLE_integration_v2.launch.py`다.

첨부한 `RUNBOOK_full_operation_20260904.md`의 준비→베이스→RTCM→launch→점검→go→종료
순서를 따르고, 상태/주차/Reference/보정 조건은
[6차 명세](../../docs/MGM_MBD_STATE_MACHINE_SPEC.md)를 적용했다.
문서 작성 시 센서·차량·CAN은 실행하지 않았다. 아래 하드웨어 명령은 현장 담당자가 실행한다.

> 물리 비상정지를 누른 상태에서 준비한다. 운전자와 주변 감시자가 준비하고 출발 점검을
> 마칠 때까지 해제하지 않는다. 기존 main launch, 주차 단독 launch, MBD/회피 시험 launch,
> 별도 센서 드라이버/CAN bridge를 함께 실행하지 않는다. v1/v2는 동일 하드웨어를 사용한다.

## 0. 코스 파일과 현재 준비 상태

손상민(`sonsm0318`)의 초기 PR #84와 최신 [PR #86 / 커밋 28ba409](https://github.com/Kimyungi/HL-Global-Mobility-Team2/pull/86)를
확인하고 최신 CSV·Zone·미션 표식을 v2에 반영했다. PR #86은 확인 시점에 OPEN이며 기존 main에는 아직 반영되지 않았다.
아래 CSV와 같은 번호의 YAML을 짝지어 **(01 또는 02)→03→04→05→(06 또는 07)** 순서로 실행한다.

| 번호 | CSV (`src/stack_gps/waypoints/` 기준) | 점 수 | 대응 YAML / 기능 |
|---|---|---:|---|
| 01 | `waypoints_halla_reference_path_01.csv` | 27 | `zones_halla_reference_path_01.yaml` — Zone/주차 없음 |
| 02 | `waypoints_halla_reference_path_02.csv` | 32 | `zones_halla_reference_path_02.yaml` — 전 구간 GPS-only |
| 03 | `waypoints_halla_reference_path_03.csv` | 180 | `zones_halla_reference_path_03.yaml` — GPS-only idx 28~116, T자 idx 145 |
| 04 | `waypoints_halla_reference_path_04.csv` | 196 | `zones_halla_reference_path_04.yaml` — GPS-only 2구간, 평행 주차 idx 163 |
| 05 | `waypoints_halla_reference_path_05.csv` | 317 | `zones_halla_reference_path_05.yaml` — GPS-only 3구간, 신호 예상 idx 300, 주차 없음 |
| 06 | `waypoints_halla_reference_path_06.csv` | 81 | `zones_halla_reference_path_06.yaml` — GPS-only 1구간, 주차 없음 |
| 07 | `waypoints_halla_reference_path_07.csv` | 79 | `zones_halla_reference_path_07.yaml` — GPS-only 1구간, 주차 없음 |

[전체 배치 그림](../stack_gps/waypoints/halla_reference_mission.png),
[시나리오 메타데이터](../stack_gps/waypoints/halla_reference_mission.yaml),
[Zone 꼭짓점](../stack_gps/waypoints/halla_reference_zone_corners.csv)을 함께 확인한다.
전체 메타데이터 YAML은 경로별 운용 `zones_file`로 대신 전달하지 않는다.

**연속 경로 설정:** [halla_route_sequence.yaml](../stack_gps/waypoints/halla_route_sequence.yaml).
MGM이 현재 CSV 종점과 해당 경로의 모든 Mission 성공 또는 CSV 종점 미완료 실패, 실제 정지를 확인하면 다음 CSV를 요청한다.
GPS가 다음 파일을 적용하고 새 fix의 유효 reference가 확인되면 기존 go 인가로 재개한다.
중간 경로에서는 FINISH를 latch하지 않으며 선택한 마지막 06 또는 07만 전체 종료다.

**이번 지정 run은 01→03→04→05→07**이다. `route_start_id:=01 route_end_id:=07`로 지정한다.
출발이 다른 경우 02, 복귀가 다른 경우 06을 명시한다. 두 인자의 기본값은 미설정이며
기동 시 모두 지정해야 한다. 01→02 또는 06→07 연결 구간은 만들지 않는다.
출구 판별 모델은 있지만 현재 v2에는 06/07 선택 runtime이 연결돼 있지 않으므로 이번 run은
수동 지정한 종료 경로를 유지한다. 주행 중 선택 변경은 지원하지 않으며 재기동한다.

01/02→03, 03→04, 05→06/07은 접속점이 일치한다. 최신 04→05는 업로드 문서대로
평행 주차 후 복귀를 위한 불연속이다. 04 끝점 `(-62.335,-72.648)`과 05 시작점
`(-64.335,-70.648)`을 직선으로 연결하지 않는다. ACTIVE Parking 중 종점 관측을 기억하고
주차 완료 응답 뒤 GPS 경로를 넘긴다. 실제 복귀 위치와 이 인계 조건 충족은 현장에서 확인한다.

각 항목의 `completion: endpoint_and_missions`는 종점 + 해당 Zone 파일의 모든 Mission 성공 또는 CSV 종점 미완료 실패다.
주차가 있는 03/04는 Mission 성공 또는 CSV 종점 미완료 실패를 확인한다. Mission 완료 자체가 CSV 종료 조건인 구성은
`missions_complete`를 명시할 수 있으며, Mission이 하나 이상 있어야 한다. CSV 종점 미완료 실패는 성공과 별도 종료로 기록하고 종점 전환을 허용한다. 다른 취소는 완료가 아니다.
ACTIVE Parking 중 지나간 종점을 기억하므로 완료 뒤 옛 종점으로 되돌아갈 필요는 없다.
인계 시 실제 정지 및 안전 조건은 유지한다. 기존 go 인가로 자동 재개하지만 무정차 연결을 뜻하지 않는다.

이 7개 CSV는 ENU 기준경로를 lat/lon과 함께 담으며 `quality` 열이 없다. 전 점이 RTK
실주행으로 기록·검증됐다고 해석하지 않는다. 노드는 lat/lon을 읽고 추가 CSV 열의
`zone_id/parking_mode/drive_mode`로 직접 상태를 전환하지 않으므로 대응 YAML이 필요하다.
02도 YAML의 GPS-only 구간으로 제어되며 별도의 `gps_only:=true` 강제값은 필요 없다.
CSV의 새 `state` 열은 0=일반, 1=T자 주차, 2=평행 주차, 3=신호 예상 위치다.
1·2는 대응 YAML 주차점과 일치한다. 05 idx 300의 state=3은 위치 표식이며
신호 정지는 기존 `stack_traffic`의 적색·정지선 입력과 MGM Signal 조건으로 판단한다.
이 코드를 legacy MGM state byte 또는 MissionType으로 그대로 대입하지 않는다.

03의 T자 주차점은 `37.30389680, 127.90720308`(idx 145), 04의 평행 주차점은
`37.30357615, 127.90682074`(idx 163)이다. main의 `parking_zone_span_m=1.0`으로 구간화되지만
**v2 탐색을 충분히 일찍 시작하는 위치라는 검증은 아직 없다**. Search→ready 이동거리와
종점까지 여유를 측정한다. 별도 선행 trigger를 정의하면 기존 parking_points를 중복 남기지 않고
그 측정 구간으로 대체하며, 위치/폭을 임의로 늘리지 않는다. 모든 업로드 YAML의 stop/avoid 구간은 비어 있다.

한라대의 기존 2026-08-19 303점 CSV는 별도 과거 코스다. 그 CSV에 위 번호별 YAML을 붙이지 않는다.

## 1. V0 — v2 폴더와 모델 준비, 빌드

현재 PC의 v2 폴더를 사용한다. 다른 PC라면 기존 main 폴더를 그대로 둔 채 별도 폴더로
`integration/v2_main`을 checkout하고 아래 `FMA_V2_WS` 한 줄을 그 위치로 바꾼다.

```bash
export FMA_V2_WS="$HOME/Desktop/HL-Global-Mobility-Team2-v2_main"
cd "$FMA_V2_WS" || exit 1
git branch --show-current
git status --short --branch
```

브랜치는 `integration/v2_main`이어야 한다. 원래 `feat/state-machine` 폴더의 미커밋 파일을
덮거나 기존 `~/FMA_ws`의 symlink/설치 경로를 바꾸지 않는다.

Git에 없는 모델은 기존에 사용하던 파일을 준비한다. 다음은 **이 PC에 이미 있는 모델**을
v2에 없는 경우에만 복사한다. 다른 PC에서는 `FMA_EXISTING_WS`를 실제 모델 보관 위치로 바꾼다.
모델을 다른 학습 결과로 임의 교체하지 않는다. 호모그래피는 v2 checkout의 파일을 사용하되
현재 카메라 장착 상태에서 측정한 값인지 확인한다.

```bash
(
  set -e
  FMA_EXISTING_WS="$HOME/FMA_ws"
  mkdir -p "$FMA_V2_WS/src/stack_lane/models"
  for relative in src/stack_lane/models/yolopv2.pt src/stack_traffic/models/yolov8n.pt; do
    if [ ! -s "$FMA_V2_WS/$relative" ]; then
      test -s "$FMA_EXISTING_WS/$relative"
      cp -- "$FMA_EXISTING_WS/$relative" "$FMA_V2_WS/$relative"
    fi
  done
  test -s "$FMA_V2_WS/src/stack_traffic/models/stopline_yolov8s_seg.pt"
  test -s "$FMA_V2_WS/src/stack_lane/config/homography.json"
  cd "$FMA_V2_WS"
  ./scripts/v2 build
  ./scripts/v2 check
)
```

`V2_INSTALL_READY`를 확인한다. 옵션 없는 기존 `colcon build`나 기존 `install/setup.bash`는
이 런북에서 사용하지 않는다. 최초 빌드/코드 변경 후에는 `./scripts/v2 test`도 통과시킨다.
이 검사는 하드웨어 없는 core/Python/ROS 합성 시험이다. `check`는 센서 연결 검사가 아니다.

## 2. 차량 PC의 각 새 터미널 — 공통 준비 블록

V0·V1·V2·V3·V4·M에서 아래 블록을 **각각 한 번** 실행한다. 앞 터미널의 변수나 함수는
새 터미널로 전달되지 않는다. `v2ros`는 조회/점검 명령에도 v2 메시지 설치를 적용하는 함수다.
함수 내부의 환경은 호출마다 초기화하며 기존 main overlay를 물려받지 않는다.

```bash
export FMA_V2_WS="$HOME/Desktop/HL-Global-Mobility-Team2-v2_main"
cd "$FMA_V2_WS" || exit 1
v2ros() {
  env -i HOME="$HOME" USER="${USER:-}" LANG="${LANG:-C.UTF-8}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    FMA_V2_WORKSPACE="$FMA_V2_WS" ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 \
    /bin/bash --noprofile --norc -c '
      set -e
      source /opt/ros/humble/setup.bash
      source "$FMA_V2_WORKSPACE/install_v2/local_setup.bash"
      exec "$@"
    ' v2ros "$@"
}
./scripts/v2 check
```

차량 launch와 조회/go는 ROS domain 0이다. `scripts/v2 bench`의 domain 175 및
`/integration_v2/*` 토픽은 하드웨어 없는 별도 벤치이므로 실차 절차에 섞지 않는다.

## 3. V0 — 기존 실행 종료와 차량 사전점검

```bash
v2ros ros2 node list
```

기존 MGM/CAN/인지/센서 노드가 보이면 해당 실행 터미널에서 정상 종료한다.
`stack_parking parking.launch.py`, `multi_lidar_drivers.launch.py`를 따로 시작하지 않는다.
이번 구성은 `parking_enabled:=true`가 4-LiDAR와 융합/주차를 함께 시작한다.

물리 비상정지와 후진·출차 공간을 확인하고 두 카메라를 USB2(HIGH)로 연결한다.

| 용도 | 장치 |
|---|---|
| 차선 OAK-D | `14442C105157D3D200` |
| 신호등/정지선 OAK-D | `14442C10B167CFD200` |
| 전방 / 후방 / 측방 LiDAR | a1 / a2 / b1 / b2 |

```bash
v2ros ros2 run stack_traffic stack_traffic_ml_preflight --require-xpu
"$FMA_V2_WS/src/bridge_dspace/tools/can_setup/install.sh" --check
v2ros /usr/bin/python3 "$FMA_V2_WS/src/multi_lidar_fusion/tools/check_sensors.py" --no-ros
```

`ML_RUNTIME_READY`, CAN FD의 MTU 72 및 점검 통과, 센서 전 항목 통과를 확인한다.
`--no-ros` 센서 점검도 실제 장치를 확인하므로 **통합 launch 전에 끝낸다**.
이 절차는 드라이버·모델·CAN 설정 오류를 `go --force`로 넘기기 위한 절차가 아니다.

## 4. B1 — 한라대학교 베이스 좌표 확인과 송출

베이스 PC는 차량 v2 overlay가 필요 없다. 다음 `FMA_BASE_WS`는 **베이스 PC의** 도구 위치다.
안테나 설치 위치와 높이를 등록 당시와 일치시킨 뒤 현재 수신기 좌표를 먼저 읽는다.

```bash
FMA_BASE_WS="$HOME/FMA_ws"
cd "$FMA_BASE_WS/src/stack_gps/tools/base_station" || exit 1
python3 read_base_position.py --port /dev/ttyF9P --baud 115200
```

등록 기준은 `halla_20260819`: 위도 **37.303844970**, 경도 **127.907281838**,
WGS84 타원체고 **183.7405m**다. 1층 야외의 등록 위치/높이를 재현한다.

기준경로 메타데이터의 ENU 원점 `37.3041743, 127.9075328`은 **코스의 좌표 원점**이며
위 베이스 수신기의 좌표와 다르다. 이 원점을 `setup_base.py`에 넣지 않는다.
기준경로는 기존 한라대 웨이포인트 idx 0을 원점으로 삼았지만 현재 물리 배치와의 일치는 현장 검증 대상이다.

다른 장소의 설정이 남아 있을 때만 현재 좌표를 기록한 후 아래 **이 장소의 값**으로 복원한다.
이 명령은 F9P 설정을 쓴다. 같은 위치를 재사용하면서 새 측량값이나 `--svin`으로 바꾸지 않는다.

```bash
python3 setup_base.py --lat 37.303844970 --lon 127.907281838 --height 183.7405
python3 read_base_position.py --port /dev/ttyF9P --baud 115200
```

`TMODE=FIXED`와 설정값 일치, 설정 도구의 `fixType=5 (TIME)`를 확인한 뒤 송출한다.
안테나 위치가 등록 지점과 다르면 이 숫자를 넣는 것만으로 코스와 일치하지 않는다.
[BASE_MOVE](../stack_gps/tools/base_station/BASE_MOVE.md),
[새 지점 측량](../stack_gps/tools/base_station/BASE_SURVEY.md)을 따른다.

```bash
ls -l /dev/ttyF9P /dev/ttyF9P_uart2 /dev/ttyRadio
python3 rtcm_server.py --radio /dev/ttyRadio
```

B1은 계속 켜 둔다. 약 10초 통계의 RTCM B/s가 0보다 커야 한다.

## 5. V1 — 차량 라디오 → RTCM TCP 중계

차량 PC의 새 터미널에서 §2 공통 준비를 한 뒤 실행한다.

```bash
v2ros /usr/bin/python3 "$FMA_V2_WS/src/stack_gps/tools/base_station/rtcm_server.py" \
  --port /dev/ttyRadio --tcp-port 2101
```

V1도 계속 켜 둔다. `RTCM` 수신이 지속되는지 확인하며 RTK가 안정화될 시간을 둔다.
차량 GPS는 통합 launch에서 `127.0.0.1:2101`로 접속한다.
`rtcm_client_inject.py`, 웨이포인트 기록기, 별도 GPS 노드를 함께 띄워 `/dev/ttyRover`를
중복 점유하지 않는다. B1의 `--radio`는 송출, V1의 `--port`는 수신 역할이다.

## 6. V2 — 코스·보정값 확인 후 통합 시작

먼저 §2 공통 준비를 실행한다. 이번 run은 01→03→04→05→07 순서다. 03은 T자, 04는 평행 주차점이 있다.
측정한 Zone 파일을 쓰려면 manifest의 해당 `zones_file`을 바꾸고 재기동한다. 단일 전역 Zone 파일로 덮지 않는다.

손상민의 최신 업로드에는 Zone 확인 수와 탐색 제한값이 없다. 주차 구간 폭 1.0m는 탐색 거리 제한이 아니다.
사용자 지정으로 `zone_enter_confirm_samples` / `zone_exit_confirm_samples`는 독립 GNSS
**5회 / 5회**를 사용한다. params.yaml 및 통합 launch 기본값도 동일하며, 아래 블록에서 다시 묻지 않는다.
이는 선택한 설정이며 실차 검증 인증값은 아니다.
주차는 `parking_zone_entry_active=true`로 stable Zone 진입 즉시 PARKING 상태가 된다.
탐색 중 GPS 목표점과 v_base로 주행하며 시간·거리 제한값은 입력하지 않는다. Zone 이탈로 종료하지 않고,
주차 완료 또는 현재 CSV 종점에서 일반 주행으로 복귀한다. 종점 미완료는 reason 10으로 기록한다.
종점 실패는 done 대기 조건을 남기지 않는다. 경로 순서가 설정되면 실제 정지와 새 CSV 응답 후 자동 재출발하고,
단일 CSV이면 FINISH다. ready 이전 GPS 상실은 탐색을 정지하고, ready 이후 Parking 경로 상실은 주차 제어를 유지한 채 정지한다.
상세 기준은 [주차 상태 / GPS 탐색 정책](../../docs/MGM_PARKING_ENTRY.md)을 따른다.

입력 파일/보정값 검사 실패 시 아래 괄호 안의 절차는 launch 전에 종료된다.
명령 마지막에서 CAN TX가 활성화되며 `wait_go=true`로 출발 인가 전 목표속도는 0이다.

```bash
(
  set -e
  cd "$FMA_V2_WS"
  FMA_SEQUENCE="$FMA_V2_WS/src/stack_gps/waypoints/halla_route_sequence.yaml"
  FMA_ROUTE_START=01
  FMA_ROUTE_END=07
  FMA_ZONE_ENTER=5
  FMA_ZONE_EXIT=5
  FMA_LANE_WEIGHTS="$FMA_V2_WS/src/stack_lane/models/yolopv2.pt"
  FMA_HOMOGRAPHY="$FMA_V2_WS/src/stack_lane/config/homography.json"
  test -s "$FMA_LANE_WEIGHTS"
  test -s "$FMA_HOMOGRAPHY"
  v2ros /usr/bin/python3 - "$FMA_SEQUENCE" \
    "$FMA_ZONE_ENTER" "$FMA_ZONE_EXIT" \
    "$FMA_ROUTE_START" "$FMA_ROUTE_END" <<'PYCODE'
import math, sys
from stack_gps.route_plan import RoutePlan
plan = RoutePlan(sys.argv[1], sys.argv[4], sys.argv[5])
assert [r.id for r in plan.files] == [sys.argv[4], '03', '04', '05', sys.argv[5]]
for value in sys.argv[2:4]:
    assert value.isdecimal() and 0 < int(value) <= 2147483647, 'Zone 확인 수는 양수 정수'
for route in plan.files:
    print(route.id, len(route.points), route.csv, route.zones, 'completion:', route.completion)
print('Zone 확인:', sys.argv[2:4], '주차 탐색: 현재 CSV의 GPS 추종, ready 후 주차 제어, 종료는 done/CSV 종점')
PYCODE
  ./scripts/v2 vehicle \
    REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
    route_sequence_file:="$FMA_SEQUENCE" \
    route_start_id:="$FMA_ROUTE_START" route_end_id:="$FMA_ROUTE_END" \
    parking_enabled:=true \
    t_parking_zone_ranges:="[0]" \
    parallel_parking_zone_ranges:="[0]" \
    zone_enter_confirm_samples:="$FMA_ZONE_ENTER" \
    zone_exit_confirm_samples:="$FMA_ZONE_EXIT" \
    parking_zone_entry_active:=true \
    escape_after_cycles:=0 \
    avoidance_enabled:=true avoid_zone_only:=false \
    lane_weights:="$FMA_LANE_WEIGHTS" \
    homography_path:="$FMA_HOMOGRAPHY" \
    usb_speed:=high camera_fps:=10 \
    traffic_enabled:=true traffic_depth_enabled:=false \
    traffic_exposure_compensation:=-2 \
    traffic_yolo_image_size:=320 \
    traffic_yolo_inference_interval:=2 \
    traffic_red_phase_yolo_inference_interval:=3 \
    traffic_stopline_yolo_image_size:=320 \
    traffic_require_stop_gate:=false traffic_stop_y_ratio:=0.0
)
```

`[0]`는 추가적인 **인덱스 인자만 비운다**. `zones_file`의 `parking_points` 또는 명시적
MISSION_ZONE은 그대로 사용한다. 주차 파일과 다른 인덱스 구간을 겹쳐 지정하지 않는다.
이 런북은 회피 제한 구간을 추측하지 않고 기존 기본 `avoid_zone_only=false`를 명시한다.
검증한 회피 구간만 허용하는 운용은 구간 파일을 먼저 준비한 뒤 해당 인자를 true로 바꾼다.

`traffic_require_stop_gate=false`, `traffic_stop_y_ratio=0.0`이어도 **v2 신호 정지는 동작한다**.
정상 진입은 `red_active && stopline_detected`, 해제는 `green_active && !red_active`다.
옛 y-ratio 정지 게이트를 새로 보정할 필요는 없다. 1.5m 소실 seed와 앞범퍼 목표 1.0m가
현재 장착에서 맞는지, 실제 정차 위치가 정지선 전 `0 < d <= 1.0m`인지 현장에서 확인한다.
RGB depth의 optical-Z는 범퍼 거리로 자동 치환하지 않는다.

현재 `fixed_speed_enabled=true`에서 비정지 속도 크기는 `v_base=1.0m/s`이며 회피에도 적용한다.
정지 제어는 유지한다. 과거 `v_avoid=0.6m/s` 설정이 현재 고정 속도를 낮추지는 않는다.
V2 시작 로그의 실제 `drive_logs/v2_...` 경로를 기록한다.

## 7. V3 — 출발 전 토픽과 상태 확인

§2 공통 준비 후 기동이 안정될 때까지 기다린다. 아래 `timeout`의 종료 코드 124는
측정 시간 만료이며 `average rate`가 한 번도 나오지 않았다면 입력 미수신이다.

```bash
v2ros ros2 node list
v2ros ros2 topic info /adas/target_ref --verbose
for topic in /perception/lane_path /perception/gps_path /perception/avoid \
  /perception/estop /perception/traffic_stop /perception/parking \
  /lidar/a1/scan /lidar/a2/scan /lidar/b1/scan /lidar/b2/scan \
  /unified_lidar/scan /parking/slam_pose /adas/target_ref /vehicle/vector; do
  v2ros timeout 12 ros2 topic hz "$topic"
done
```

`/adas/target_ref` publisher는 MGM 하나여야 한다. 인지/scan은 각 설정 주기로 지속 수신되고
`/adas/target_ref`, `/vehicle/vector`는 약 100Hz여야 한다. 4-LiDAR 구성은 `/scan` 대신
전방 `/lidar/a1/scan`을 회피·긴급정지가 공유한다. 필수 입력 미수신, GPS/Traffic 노드의
반복 재시작, 최종 reference publisher 중복이면 출발하지 않는다.

```bash
v2ros timeout 15 ros2 topic echo /perception/gps_path --field fix_quality
v2ros timeout 5 ros2 topic echo /perception/gps_path --once
v2ros timeout 5 ros2 topic echo /perception/estop --once
v2ros timeout 5 ros2 topic echo /perception/parking --once
v2ros timeout 5 ros2 topic echo /adas/mgm_state --once
v2ros timeout 5 ros2 topic echo /adas/target_ref --once
v2ros ros2 param get /mgm_node backend
v2ros ros2 param get /mgm_node base_state_machine_enabled
v2ros ros2 param get /mgm_node escape_after_cycles
```

| 점검 | 출발 전 기대값 / 해석 |
|---|---|
| GPS | `fix_quality=4` 지속, 올바른 코스/실제 위치/index, `reference_stamp` 실제 fix 갱신 |
| Estop | 장애물 없을 때 `estop=false`, `scan_valid=true`; 후방 corridor UNKNOWN은 현재 정상 |
| MGM backend | `core`, `base_state_machine_enabled=true`, `escape_after_cycles=0` |
| Top / 최종 명령 | `top=0`(ENABLE), `v_ref=0`; go 대기로 EXTERNAL bit 8이 있는 것은 정상 |
| Reference | 현재 선택 소스의 `selected_reference_valid/fresh=true`; 모든 provider를 동시에 요구하지 않음 |
| Parking 준비 전 | `mission=0`, 요청 inactive, 이전 세션의 done/ref가 현재 실행으로 인정되지 않음 |
| 탐색 설정 | `parking_zone_entry_active=true`, `parking_calibration_state=3`(NOT_REQUIRED), Zone 확인 5/5 |
| Zone | 현재 raw/stable 소속과 ID를 확인. `ZONE_CONTEXT_UNAVAILABLE` bit 256이면 go로 풀리지 않음 |

**Mission 시작 위치 밖에서 enable한다.** ENABLE 중 소비된 Mission entry는 go에서 재생하지 않는다.
주차 구간 안에서 켰다면 요청이 자동 생성된다고 기대하지 말고, 확인된 exit 후 새 entry가
생기는 경로로 준비한다. 명시적 MISSION_ZONE만 사용하는 경우 legacy `parking_zone=false`여도
Mission Zone 안일 수 있으므로 `/adas/mgm_state.zones`를 기준으로 본다.

## 8. V3 — 신호등과 정지선 확인

물리 비상정지를 유지하고 실제 적색+정지선을 카메라에 보인다.

```bash
v2ros ros2 topic echo /perception/traffic_stop
```

적색 확정과 안정 정지선에서 `red_active=true`, `green_active=false`,
`stopline_detected=true`, `fail_safe_stop=false`를 확인한다. 같은 신호등을 초록으로
바꾸면 `red_active=false`, `green_active=true`여야 한다. 확인 후 echo만 Ctrl-C로 끝낸다.
적색에서 끝까지 정지선 미검출, 초록 미해제, fail_safe_stop 지속이면 출발하지 않는다.
`stop_required` 하나만 보고 정상 전이나 정지 기능 활성 여부를 판정하지 않는다.

## 9. V4 — 출발 인가 / M — 주행 중 상태

V4 새 터미널에서 §2 공통 준비를 한 뒤, 사람이 앞 단계 점검을 마친 경우에만 물리
비상정지를 해제하고 실행한다. 운전자는 계속 비상정지에 손을 둔다.

```bash
./scripts/v2 go --require-traffic --ros-args -r /scan:=/lidar/a1/scan
```

**remap은 필수다.** 기존 go 도구의 `/scan` 점검을 실제 전방 a1 입력으로 연결한다.
점검 자체를 생략하는 `--force`/`--skip-*`로 대체하지 않는다.
go 도구는 수신/RTK 점검이며 v2 Mission/Zone 보정과 모든 Reference freshness를
검증하는 도구가 아니므로 §7의 상태 점검도 필요하다. `[OK]` 뒤 출발 인가 완료를 확인한다.

M 터미널은 §2 공통 준비 후 계속 켜 둔다.

```bash
./scripts/v2 state
```

| 상황 | v2에서 볼 동작 |
|---|---|
| 일반 주행 | LINE/GPS_BACKUP, stable GPS-only에서는 GPS_ONLY_NAV |
| 회피 | Avoid ACTIVE→CLEAR_CONFIRM; 소실 200틱 확인과 LINE 복귀 hold 300틱은 같은 시점부터 계산 |
| 신호 | 회피 경로와 Signal 속도 제약이 함께 동작. 적색 소실만으로 풀리지 않음 |
| Mission entry | stable MISSION_ZONE entry로 IDLE(0)→ACTIVE(1), 요청 ID latch, PARKING=3 |
| 준비 | ACTIVE 안에서 PREPARE 명령으로 준비, ready→ACTIVATE ack/유효 경로까지 0 속도 |
| ready / 실행 | ACTIVE 유지, 현재 요청의 fresh ready 이후 ACTIVATE 전송; 실행 ack/ref부터 이동 |
| Zone exit | 주차 상태 유지. 정상 복귀는 주차 완료 또는 현재 CSV 종점 |
| 주차 중 정지 | 일반 Avoidance·LiDAR AUTO_ESTOP·Signal은 마스킹. Parking path_blocked/ref gate와 외부/운전자/CAN 정지 유지 |
| 완료 | 현재 요청 done 후 Mission ID 완료 기억, 현재 Zone에 따라 nav_reselect |
| 경로 이상 | 선택 경로 invalid/stale면 양·음 목표속도 모두 0, Mission/Avoid 소유권 유지 |
| Recovery | 현재 OFF, rear UNKNOWN. 일반 Parking 후진과 자동 Escape Recovery를 구분 |
| 종점 | 중간 CSV는 필수 Mission 종료·실제 정지·새 reference 확인 뒤 자동 전환. 마지막 07만 FINISH latch, go만으로 풀리지 않음 |

`TargetRef.state`의 LANE/WAYPOINT/AVOID/PARKING/TRAFFIC byte는 위 병행 상태의 요약이다.
그 byte만으로 Mission 준비·신호·Safety를 판정하지 않는다. Parking의 0.20m 벽 도착 판정은
후방 통로 전체가 안전하다는 인증이 아니며, `rear_clear`를 수동 true로 만들지 않는다.

## 10. 정지·취소·종료

위험 시 물리 비상정지를 먼저 누른다. 기존 임시 정지 명령은 다음과 같다.

```bash
./scripts/v2 stop
```

operator stop은 Mission 요청을 유지한다. 현재 Zone 탐색에는 timeout이 없으며 경과 시간은 관측 기록으로 남긴다.
정지한 상태에서 Mission만 명시적으로 취소해야 할 때는 다음을 사용한다.

```bash
v2ros ros2 topic pub --once /operator/cancel_mission std_msgs/msg/Bool '{data: true}'
```

정상 종료: 차량 정지·물리 비상정지 → V2 Ctrl-C 한 번으로 CAN zero guard 완료 →
V1 종료 → B1 종료 순서다. `kill -9`, CAN 케이블 분리, PC 전원 차단을 정상 정지 수단으로
사용하지 않는다. 다른 경로를 선택할 때도 이 절차를 거쳐 새 launch로 시작한다.
`start_session`은 FINISH와 Mission 성공/실패 기억을 지우므로 단순 오류 해제용으로 반복 발행하지 않는다.

## 11. 시험 후 기록

V2 콘솔에 나온 **이번 run의 실제 절대경로**를 입력한다. 자동으로 가장 최근 디렉터리를
고르면 다른 시험과 섞일 수 있다. §2 공통 준비가 된 터미널에서 실행한다.

```bash
(
  set -e
  read -r -p '이번 drive_logs/v2_... run 절대경로: ' FMA_RUN
  test -d "$FMA_RUN"
  v2ros ros2 bag info "$FMA_RUN/rosbag"
  "$FMA_V2_WS/install_v2/adas_mgm/lib/adas_mgm/core_replay" \
    "$FMA_RUN/mgm_snapshots.bin" "$FMA_RUN/replay.csv"
  /usr/bin/python3 "$FMA_V2_WS/src/adas_mgm/tools/analyze_mission_calibration.py" \
    "$FMA_RUN/mission_events.csv" \
    --json "$FMA_RUN/mission_calibration.json" \
    --markdown "$FMA_RUN/mission_calibration.md"
)
```

`mission_events.csv`, `zone_observations.csv`, `transitions.csv`, `mgm_jitter.csv`, `lateral.csv`,
`vehicle_vector.csv`를 보관한다. 함께 기록할 정보는 경로/Zone 파일명과 commit, 베이스 설치,
사용한 manifest와 경로별 Zone 파일, Zone 확인 5/5·탐색 정책·속도 설정, 모델/호모그래피, 물리 앞범퍼 정차 위치다. raw dump는 **v15**이며
구버전 replay/generated v1.88로 현재 병행 상태의 parity를 주장하지 않는다.

현재 기본 bag에는 a1/unified scan과 MGM/Mission 진단이 포함되지만 a2/b1/b2 raw scan 및
operator go/stop/start_session 전체를 모두 기록하지는 않는다. 그 분석까지 필요하면 시험 전에
기록 구성을 별도로 정한다. 이 런북은 기록되지 않은 토픽을 사후 복원할 수 있다고 가정하지 않는다.
bag 재생은 차량 launch를 종료한 별도 domain에서만 수행한다. 운영 domain에 TargetRef/명령 토픽을
그대로 replay하지 않는다. Mission 분석 도구는 관측 통계를 만들며 운영 제한값을 자동 결정하지 않는다.

## 12. 멈추거나 시작되지 않을 때

### 라이다 4대가 Unknown error로 종료되며 /dev/lidar_*가 없을 때

2026-09-12 현장에서는 CP2102 네 대가 ttyUSB0~3으로 열거됐지만 설치된 udev의
USB 경로 `usb-0:2...`와 현재 `usb-0:4...`가 달라 네 링크/접근 권한이 적용되지 않았다.
USB 서술자 serial은 네 대 모두 0001이라 위치를 구분하지 못한다. ttyUSB 번호로 앞뒤를 추측하지 않는다.

물리 비상정지를 누르고 V2 통합 launch를 Ctrl-C로 정상 종료한 뒤 실행한다. B1/V1 중계는 유지해도 된다.

```bash
cd "$HOME/Desktop/HL-Global-Mobility-Team2-v2_main" || exit 1
sudo /usr/bin/python3 scripts/v2_recover_lidars.py --apply
ls -l /dev/lidar_front /dev/lidar_rear /dev/lidar_left /dev/lidar_right
```

도구는 저장소 SDK의 GET_DEVICE_INFO 조회만 보내 등록된 내부 일련번호로 앞/뒤/좌/우를 확인한다.
네 번호가 모두 일치할 때 기존 규칙을 백업하고 현재 USB ID_PATH로 갱신한다. 해당 tty 장치 네 개만
udev 재적용하며 CAN/GPS/IMU나 스캔/차량 launch를 시작하지 않는다. `LIDAR_LINKS_READY`는
이름/권한 확인 결과다. 센서 위치 자체를 바꿨다면 등록 일련번호의 장착 위치부터 다시 확인한다.
번호 미확인/중복/응답 없음/사용 중 포트는 중단하며 임의 위치를 지정하지 않는다.
관리자 비밀번호는 sudo를 실행한 현장 터미널에 입력한다.
복구 도구는 Python 표준 라이브러리만 사용하므로 root 계정의 pyserial 설치는 필요 없다.
이전 도구에서 `No module named 'serial'`이 났다면 수정된 도구로 같은 명령을 다시 실행한다.

완료 후 §6의 같은 통합 launch를 다시 실행하고 §7에서 네 scan·회피·Estop 입력을 재확인한다.
이름 복구만으로 스캔 정상이나 출발 허가가 보장되지는 않는다. go는 점검 완료 뒤 별도로 실행한다.

2026-09-12 링크 복구 후 추가 확인: 뒤/좌/우는 약 9Hz로 수신됐으나 a1은 exit -6으로 종료됐다.
실제 launch 파라미터에 a1만 기존 9K/16bit가 남아 있었다. `stack_parking/parking.launch.py`가
`multi_lidar_fusion/multi_lidar_drivers.launch.py`를 호출하던 연결을
`lidar_fusion_v2/drivers.launch.py`로 수정해 네 대 모두 기존 v2 4K/8bit 프로필을 사용한다.
중첩 launch에서 생성되는 실제 네 Node의 파라미터/토픽까지 확인하는 시험을 포함해 20개 검사를 통과했다.
현재 symlink install에는 반영됐으며 실행 중 프로세스는 바뀌지 않으므로 V2 정상 종료 후 재시작한다.
수정 후 실제 a1 수신과 GPS FIXED를 확인한 뒤 출발한다.

| 증상 | 우선 확인 |
|---|---|
| go가 LiDAR 미수신 | §9 `/scan:=/lidar/a1/scan` remap 및 실제 a1 주기 |
| FIXED인데 코스가 통째로 어긋남 | 베이스 좌표·물리 설치 위치/높이·CSV의 짝 |
| Zone bit 256 | 확인 표본 수가 0/잘못된 값인지 확인. go는 calibration 설정을 바꾸지 않음 |
| Mission 취소 reason 10 | 주차 미완료 상태로 현재 CSV 종점 도달. 실패 기록 후 기존 다음 CSV 인계 |
| 주차 준비 대기 | PARKING 상태에서 현재 요청 ready/ACTIVATE ack/유효 reference 확인 |
| 과거 취소 reason 7/9 | 즉시 진입 false인 과거 준비 모드의 속도 상실/Zone 이탈 |
| Parking이 시작 안 됨 | 해당 코스의 stable Mission Zone entry, 완료 기억, PREPARE/ready/현재 request ID |
| ACTIVE에서 정지 | 현재 request의 ACTIVATE ack, Parking reference_stamp/validity, path_blocked, 외부/CAN 입력 |
| Signal 접근/정지에서 멈춤 | 신호 상태와 bit 32/64, traffic 및 실제 차속 수신 |
| 같은 경로 재출발 불가 | Top FINISH/완료 기억; 새 시험의 정상 재기동/세션 절차 확인 |
| Reference bit 4 | 선택 source의 실제 generation/freshness; 발행 Hz만으로 정상 판단하지 않음 |

참조: [v2 사용법](../../docs/INTEGRATION_V2.md),
[v2 반영 목록](../../docs/INTEGRATION_V2_SCOPE.md),
[6차 검증 보고](../../docs/MGM_STABILIZATION_REPORT.md),
[기존 운영 런북](RUNBOOK_full_operation_20260904.md),
[GPS 설정 가이드](../stack_gps/tools/waypoints/README.md).

## 연속 경로 상태 확인

§2의 `v2ros` 함수를 준비한 터미널에서 확인한다.

```bash
v2ros ros2 topic echo /adas/mgm_state --field route
v2ros ros2 topic echo /perception/gps_path --field route
```

`index`는 선택한 5개 경로의 순번 0~4다. 이번 설정은 01/03/04/05/07에 대응한다.
`route_id`는 GPS 출력의 실제 CSV 번호다. 한라대는 `connecting=false`이며 본 CSV만 주행한다.


| phase | 의미 / 다음 단계 |
|---|---|
| RUNNING=1 | 선택된 본 CSV 주행. 잘못된 종점 기동은 skip 방지 정지 |
| WAIT_MISSION=2 | 종점 관측 후 필수 Mission 성공 또는 CSV 종점 미완료 실패 대기. ACTIVE Parking 제어는 유지 |
| WAIT_STOP=3 | 실제 정지 및 외부/신호 정지 해제 대기 |
| WAIT_ACK=4 | 요청한 다음 index, 요청 번호, 새 GNSS reference의 응답 대기. 정지 유지 |
| FINISHED=5 | 선택한 마지막 06/07 종료. Top FINISH |
| FAULT=6 | 순서/구성/producer 불일치. 원인을 확인하고 명시적으로 새 session 시작 |

`SAFE_STOP_ROUTE_SEQUENCE=512`는 이 인계/오류 정지다. 다른 정지 reason도 함께 확인한다.
현재 파일은 `/perception/gps_path.route`에서 확인한다. `waypoint_csv` 파라미터는 시작 설정이다.
`mark_zone`을 사용하면 `--track`/`--out`에 해당 CSV/Zone을 명시하고 재기동한다.
GPS가 재시작해도 현재 sequence를 자동 처음부터 다시 실행하지 않는다. `/operator/start_session`
이벤트만 완료 기억을 지우고 선택한 시작 경로(이번 run은 01) 적용을 요청한다. `/operator/go`는 경로를 리셋하지 않는다.
단일 CSV 단위시험은 `route_sequence_file` 없이 기존 `waypoint_csv`/`zones_file`로 실행하며,
그때는 해당 파일 종점에서 기존대로 FINISH다. 상세 계약은 [경로 전환 명세](../../docs/MGM_ROUTE_SEQUENCE.md)를 따른다.
