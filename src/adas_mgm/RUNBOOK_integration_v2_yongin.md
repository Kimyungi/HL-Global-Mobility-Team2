# Integration v2 통합 자율주행 시작 — 용인 운전면허시험장 Course A

> **1/10 무인 RC 시험 후진:** E-stop 연속 1000틱(정상 주기 10초) 뒤 −0.8m/s로 최대
> 162틱(1.62초, 명령상 약 1.3m) 후진한다. `escape_require_rear_clear=false`로 후방
> CLEAR를 요구하지 않으며, 주차 ACTIVE에서는 후진 복구를 실행하지 않는다.
> [현재 후진 설정과 종료 조건](../../docs/RC_REVERSE_RECOVERY.md)을 적용한다.

> 2026-09-13 주차 변경: [즉시 주차 진입](../../docs/MGM_PARKING_ENTRY.md).
> Zone 진입 즉시 PARKING이며 탐색 중 현재 CSV의 GPS를 추종한다. ready 후 주차 제어로 인계한다. 미완료 종점은 실패 처리 후 다음 CSV로 자동 전환한다(단일 CSV는 FINISH).

> **현재 설정:** 일반 주행 회피·LiDAR E-stop ON, Mission ACTIVE의 탐색·실행 중 둘 다 제외.
> 신호등 노출 보정은 `-2`이며 `traffic_exposure_compensation:=0`으로 기본 자동 노출 수준을 선택할 수 있다.
> 노출 변경은 launch 재시작에 적용하며 차선 카메라 노출에는 영향을 주지 않는다.

2026-09-12 · `integration/v2_main` · 기준 `8697bcb` + 연속 경로 확장 (미커밋) · **실차 검증 전 런북**

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

손상민(`sangmin <sonsm0928@gmail.com>`)의
[커밋 9d81b53](https://github.com/Kimyungi/HL-Global-Mobility-Team2/commit/9d81b53)을 확인했다.

- 운용 검토 대상: `src/stack_gps/waypoints/waypoints_yongin_license_course_a_20260905_131357.csv`
- 2,141개 행, idx 0~2140, **모든 quality=4**. 첫 점 `37.2889723, 127.1076515`,
  마지막 점 `37.2887765, 127.1076170`이다. 실제 현장에서 진행 방향과 경로 일치를 확인한다.
- `/home/sangmin/FMA_ws`의 `..._131322.csv`는 로컬에 있는 43점 파일이다.
  업로드된 위 2,141점 파일과 구별하여 이 런북에서는 사용하지 않는다.
- 현재 main/v2의 업로드 자료에는 **용인용 zones YAML, T자/평행 Mission trigger가 없다**.
  `zones_wonju_...yaml`이나 한라대 YAML을 용인에 재사용하지 않는다.

**GPS CSV는 준비됐지만 주차를 포함한 전체 Mission 운행 설정은 아직 별도 확정이 필요하다.**
이 문서의 전체 주행 블록은 현장에서 검증한 용인 Zone 파일의 경로를 입력받으며, 파일이 없거나
Mission이 비어 있으면 시작하지 않는다. 임의 주차 인덱스 120/140, 260/285를 채워 넣지 않았다.
주차를 제외한 별도 시험을 했더라도 전체 Mission 통합 검증이 완료된 것으로 기록하지 않는다.

용인 Zone 파일은 다음 메타데이터 형식을 사용한다. 아래 빈 예시는 **실행 가능한 Mission 설정이 아니다**.
`track`은 정확히 이 CSV 이름을 가리켜야 한다. 검증한 start/end 위경도 또는 idx 구간과
Mission ID/type을 [설정 예시](../stack_gps/config/mission_zones.example.yaml)에 맞게 추가한다.

```yaml
track: waypoints_yongin_license_course_a_20260905_131357.csv
stop_points: []
avoid_zones: []
gps_only_zones: []
parking_points: []
zones: []
```

`parking_points`는 주차점+모드를 기존 폭으로 구간화한다. 명시적 `zones`의 MISSION_ZONE은
Search 시작 위치를 직접 정의할 수 있다. v2에서는 Search→ready 시간/거리를 측정해 trigger를
정하고, 둘로 같은 Mission을 중복 정의하지 않는다. GPS-only와 회피 제한 구간도 측정한 경우에만 추가한다.

`BASE_LOCATIONS.md`의 용인 코스 “기록 예정” 문구는 위 CSV 업로드보다 이전 기록이다.
이번에는 실제 업로드 CSV를 기준으로 썼다. CSV 안에 base ID/안테나 설치 증빙은 없으므로
베이스 등록값과 기록 당시의 물리 설치가 일치하는지는 현장 기록으로 확인한다.

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

## 4. B1 — 용인 운전면허시험장 Course A 베이스 좌표 확인과 송출

베이스 PC는 차량 v2 overlay가 필요 없다. 다음 `FMA_BASE_WS`는 **베이스 PC의** 도구 위치다.
안테나 설치 위치와 높이를 등록 당시와 일치시킨 뒤 현재 수신기 좌표를 먼저 읽는다.

```bash
FMA_BASE_WS="$HOME/FMA_ws"
cd "$FMA_BASE_WS/src/stack_gps/tools/base_station" || exit 1
python3 read_base_position.py --port /dev/ttyF9P --baud 115200
```

등록 기준은 `yongin_license_20260905`: 위도 **37.288898139**, 경도 **127.107505461**,
WGS84 타원체고 **114.4403m**다. 2026-09-05 VRS FIXED 1,200표본/20분 측량 기록이며
설정 정확도는 0.0200m다. 높이는 해발고도가 아니다.

[용인 베이스 현장 가이드](../stack_gps/tools/base_station/FINAL_BASE_GPS_SETTING_IN_YONGIN.md)와
[좌표 레지스트리](../stack_gps/tools/base_station/BASE_LOCATIONS.md)를 확인한다.
등록 문서에도 설치 사진/바닥 표시/삼각대 높이 보완이 남아 있으므로, 숫자만 일치한다고
현재 안테나 설치가 재현된 것으로 판단하지 않는다.

다른 장소의 설정이 남아 있을 때만 현재 좌표를 기록한 후 아래 **이 장소의 값**으로 복원한다.
이 명령은 F9P 설정을 쓴다. 같은 위치를 재사용하면서 새 측량값이나 `--svin`으로 바꾸지 않는다.

```bash
python3 setup_base.py --lat 37.288898139 --lon 127.107505461 --height 114.4403
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

먼저 §2 공통 준비를 실행한다. 업로드된 2,141점 CSV를 고정 사용하고, §0에서 준비한 용인 Zone YAML의 절대경로를 입력한다.

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
  FMA_COURSE="$FMA_V2_WS/src/stack_gps/waypoints/waypoints_yongin_license_course_a_20260905_131357.csv"
  read -r -p '현장에서 검증한 용인 Course A Zone YAML 절대경로: ' FMA_ZONES
  test -n "$FMA_ZONES" && test -r "$FMA_ZONES"
  FMA_ZONE_ENTER=5
  FMA_ZONE_EXIT=5
  FMA_LANE_WEIGHTS="$FMA_V2_WS/src/stack_lane/models/yolopv2.pt"
  FMA_HOMOGRAPHY="$FMA_V2_WS/src/stack_lane/config/homography.json"
  test -s "$FMA_LANE_WEIGHTS"
  test -s "$FMA_HOMOGRAPHY"
  v2ros /usr/bin/python3 - "$FMA_COURSE" "$FMA_ZONES" \
    "$FMA_ZONE_ENTER" "$FMA_ZONE_EXIT" <<'PY'
import csv, math, sys
from pathlib import Path
import yaml
from stack_gps.path_engine import load_waypoints_csv
course, zones = map(Path, sys.argv[1:3])
points = load_waypoints_csv(str(course))
assert len(points) >= 10 and all(math.isfinite(v) for p in points for v in p)
config = yaml.safe_load(zones.read_text())
assert isinstance(config, dict), 'Zone 파일은 YAML mapping이어야 합니다'
assert config.get('track') == course.name, 'Zone track과 선택 CSV가 다릅니다'
for key in ('stop_points', 'avoid_zones', 'gps_only_zones', 'parking_points', 'zones'):
    assert isinstance(config.get(key, []), list), f'{key}는 list여야 합니다'
for value in sys.argv[3:5]:
    assert value.isdecimal() and 0 < int(value) <= 2147483647, 'Zone 확인 수는 양수 정수'
assert course.name == 'waypoints_yongin_license_course_a_20260905_131357.csv'
assert len(points) == 2141, '업로드 기준과 다른 CSV입니다. 수정 이력/코스를 다시 확인하세요'
with course.open(newline='') as stream:
    assert all(row['quality'] == '4' for row in csv.DictReader(stream))
mission_zones = [z for z in config.get('zones', []) if isinstance(z, dict)
                 and z.get('zone_type') in ('MISSION_ZONE', 2)]
assert config.get('parking_points') or mission_zones, '주차 Mission이 없습니다. Zone 확정 후 시작하세요'
print('course:', course, 'loaded points:', len(points))
print('zones:', zones)
print('GPS-only:', config.get('gps_only_zones', []))
print('parking points:', config.get('parking_points', []))
print('explicit zones:', config.get('zones', []))
print('Zone 확인:', sys.argv[3:5], '주차 탐색: 현재 CSV의 GPS 추종, ready 후 주차 제어, 종료는 done/CSV 종점')
PY
  ./scripts/v2 vehicle \
    REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
    waypoint_csv:="$FMA_COURSE" \
    zones_file:="$FMA_ZONES" \
    parking_enabled:=true \
    t_parking_zone_ranges:="[0]" \
    parallel_parking_zone_ranges:="[0]" \
    zone_enter_confirm_samples:="$FMA_ZONE_ENTER" \
    zone_exit_confirm_samples:="$FMA_ZONE_EXIT" \
    parking_zone_entry_active:=true \
    escape_after_cycles:=1000 \
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
| MGM backend | `core`, `base_state_machine_enabled=true`, `escape_after_cycles=1000` |
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
| 종점 | FINISH latch, go만으로 풀리지 않음. 경로 변경은 정상 종료·재기동 후 수행 |

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
Zone 확인 5/5·탐색 정책·속도 설정, 모델/호모그래피, 물리 앞범퍼 정차 위치다. raw dump는 **v15**이며
구버전 replay/generated v1.88로 현재 병행 상태의 parity를 주장하지 않는다.

현재 기본 bag에는 a1/unified scan과 MGM/Mission 진단이 포함되지만 a2/b1/b2 raw scan 및
operator go/stop/start_session 전체를 모두 기록하지는 않는다. 그 분석까지 필요하면 시험 전에
기록 구성을 별도로 정한다. 이 런북은 기록되지 않은 토픽을 사후 복원할 수 있다고 가정하지 않는다.
bag 재생은 차량 launch를 종료한 별도 domain에서만 수행한다. 운영 domain에 TargetRef/명령 토픽을
그대로 replay하지 않는다. Mission 분석 도구는 관측 통계를 만들며 운영 제한값을 자동 결정하지 않는다.

## 12. 멈추거나 시작되지 않을 때

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

용인은 현재 업로드된 Course A 단일 CSV를 사용하므로 위 명령은 연속 경로를 켜지 않는다.
추후 분할 CSV가 확정되면 [경로 전환 명세](../../docs/MGM_ROUTE_SEQUENCE.md)의 `route_sequence_file`로
순서와 경로별 Zone/전환 조건을 지정한다. 한라대 manifest를 용인에 사용하지 않는다.
