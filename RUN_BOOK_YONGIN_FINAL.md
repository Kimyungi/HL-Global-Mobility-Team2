# RUN_BOOK_YONGIN_FINAL

현재 상태 머신: **스테이트 v09.17** ([명세·상태 점검](docs/STATE_V09_17.md)).

2026-09-20 · `integration/v2_main` · 이 PC 기준 용인 전체 주행 런북입니다.
한라 런북과 동일하게 **GPS 연결 → 센서 연결 확인 → 주행 런처 → GO → 종료** 순서로 설명합니다.
모든 실행 명령은 `/home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main`의 `scripts/v2`를 사용합니다.
기본 경로는 **저장된 `src/stack_gps/waypoints/yongin_no_parking.csv`**입니다.
`prepare/drive`에서 `course`를 생략해도 `yongin_no_parking`이 선택되며,
실행 시 경로 01~07로 분리하여 사용합니다. 주차 미션은 수행하지 않습니다.
문서의 하드웨어 명령은 작성 중 실행하지 않았습니다.

## 1. GPS 연결 코드

**터미널 1**에서 실행합니다.

베이스는 용인 등록 위치에서 RTCM을 송출 중이어야 합니다.
등록값은 `yongin_license_20260905`: 위도 `37.288898139`, 경도 `127.107505461`,
WGS84 타원체고 `114.4403m`입니다. 현재 안테나 설치 위치가 같은지는 현장에서 확인합니다.
베이스 설치·설정·송출 절차는 [용인 베이스 가이드](src/stack_gps/tools/base_station/FINAL_BASE_GPS_SETTING_IN_YONGIN.md)를 따릅니다.

차량 PC:

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
scripts/v2 gps-start
```

정상 수신 출력 예시입니다. 수신량·PID는 실행마다 다릅니다.

```text
GPS 수신 상태를 1초마다 표시합니다. Ctrl-C: 표시만 종료, GPS 연결 유지.
RTCM    560 B/s | FIXED (FIX=4) | 누적 11200 B | GPS PID=12345
```

`FIXED (FIX=4)`와 RTCM 수신을 확인합니다. `gps-start`는 차량 라디오 중계도 관리합니다.
별도 차량 `rtcm_server.py`나 GPS 기록기를 중복 실행하지 않습니다.
현재 설정은 [persistent_gps.yaml](src/stack_gps/config/persistent_gps.yaml)의
`/dev/ttyRover`, `/dev/ttyRadio`, `127.0.0.1:2101`입니다.

**Ctrl+C는 상태 표시만 닫습니다. GPS/RTCM 서비스는 계속 연결됩니다.**

```bash
scripts/v2 gps-status
```

GPS를 완전히 끊었다가 다시 연결하려면, 먼저 주행을 정지하고 주행 런처를 종료한 뒤 실행합니다.

```bash
scripts/v2 gps-off
scripts/v2 gps-start
```

## 2. 모든 센서 초기화 및 연결 확인 코드

3번 런처가 라이다 4대·카메라·IMU 등 주행 센서를 초기화하고, 1번 GPS 서비스를 재사용합니다.
별도 차량 GPS나 RTCM 중계기를 중복 실행하지 않습니다.

**터미널 2 — 런처 실행 전 장치 확인:**

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
for device in /dev/ttyRover /dev/ttyRadio /dev/ttyUSB_IMU \
              /dev/lidar_front /dev/lidar_rear /dev/lidar_left /dev/lidar_right; do
  if [ -c "$device" ] && [ -r "$device" ] && [ -w "$device" ]; then
    printf 'OK: %s\n' "$device"
  else
    printf 'CHECK: %s\n' "$device"
  fi
done
```

장치 접근 확인과 실제 데이터 수신은 별개입니다. 라이다 링크가 없을 때의 복구 명령은
주행 런처를 종료한 상태에서만 실행합니다.

```bash
sudo /usr/bin/python3 scripts/v2_recover_lidars.py --apply
```

**터미널 2 — 상태 표시를 열어 두고, 3번 런처 실행 후 수신을 확인합니다:**

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
scripts/v2 state
```

| 필드 | 출발 전 확인 |
|---|---|
| `revised_v2` | `true` |
| `lidar_ready`, `lidar_missing_topics` | `true`, `[]` |
| `gps_fixed_ready` | `true`: GPS FIXED와 유효 경로 필수 |
| `camera_available` | 마지막 미션용 원본 영상 수신 상태 |
| `start_ready` | `true` |
| `go_authorized` | GO 전 `false`, 인가 후 `true` |
| `selected_reference_valid`, `selected_reference_fresh` | 실제 사용할 참조의 유효성·신선도 |
| `/adas/target_ref.v_ref`, MGM의 `reference_motion_blocked`, `active_safe_stop_reasons` | GO 전 0 속도는 정상; GO 후 정차 원인은 상태와 함께 확인 |

현재 revised v2 준비 조건은 **라이다 4대 정상 AND GPS FIXED·유효 경로**입니다.
카메라 영상만으로 GPS 준비 조건을 대신할 수 없습니다.
준비 완료와 실제 이동에 필요한 GPS 헤딩·선택 참조·주차 피드백 유효성은 별도입니다.
상태 표시에서 Ctrl+C를 누르면 표시만 종료됩니다.

## 3. 주행 런쳐 코드 — 용인 전체 코스

차량을 선택할 CSV의 진행 방향으로 배치합니다. 최초 RTK FIXED와 신선한 IMU yaw로
가까운 웨이포인트 방향에 초기 헤딩을 정렬합니다. 이후 IMU 회전량과 GPS COG를 사용합니다.
`웨이포인트 초기 헤딩 정렬: idx …` 로그를 확인합니다.

### 3-1. 실행 시점

1번의 FIXED 수신과 2번 장치 확인을 마친 뒤 터미널 1에서 GPS 표시를 Ctrl+C로 닫습니다.
GPS 연결은 유지됩니다. 이미 주행 런처가 실행 중이면 5번 순서로 정지·종료한 뒤 실행합니다.
아래 두 방법 중 하나만 사용합니다. `prepare`와 `drive`는 같은 용인 런처입니다.

### 3-2. 시작 경로와 속도를 질문받아 실행 — 기본 사용법

```bash
scripts/v2 prepare course:=yongin_no_parking REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX
```

시작 경로 번호와 일반 속도를 차례로 입력합니다. 예: `01`, `2.0`.
값은 매 실행마다 지정하며 이전 값이나 숨은 기본 속도를 재사용하지 않습니다.

### 3-3. 질문 없이 경로와 속도를 직접 지정하는 방법

```bash
scripts/v2 prepare \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  course:=yongin_no_parking start_waypoint:=01 v_base:=2.0
```

다른 출발 위치는 `start_waypoint:=02`로 지정합니다. `2.0`은 일반 주행 2m/s의 예시값입니다.
03~05 중간 시작도 선택할 수 있지만, 처음부터 전체 코스를 수행하는 시작은 01 또는 02입니다.
06 또는 07에서 시작하면 같은 번호의 단일 종료 경로만 실행합니다.

05를 포함하는 실행은 06·07을 모두 사전 로드하여 YOLO 판단으로 선택합니다.
`end_waypoint:=06` 또는 `07`을 지정해도 이 동적 분기를 고정 선택으로 덮어쓰지 않습니다.
일반 전체 주행 명령에서는 `end_waypoint`를 생략하면 됩니다.

이 명령은 센서·MGM·CAN·RViz를 실행하고 **GO를 기다립니다**. 터미널을 켜 둡니다.
런처의 `selected start CSV`, `general driving v_base`, `route`, `logs` 출력을 확인합니다.
경로 목록에는 분기 사전 로딩 때문에 06·07이 함께 표시될 수 있습니다.

### 3-3a. 기본 no-parking 경로 확인

기본값도 no-parking이며, 아래 명령은 코스를 명시해 실행합니다.

```bash
scripts/v2 prepare \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  course:=yongin_no_parking start_waypoint:=01 v_base:=2.0
```

- 원본: `src/stack_gps/waypoints/yongin_no_parking.csv` (현재 2497개 행).
- 매 prepare에서 저장된 원본을 읽고 해당 실행 로그 폴더의 `no_parking_route/`에
  경로 01~07, zone 파일, 원본 스냅샷·SHA256을 생성합니다. 편집기의 저장 전 내용은 반영되지 않습니다.
- 01 또는 02 출발, 03 → 04 → 05, 마지막 미션의 06/07 선택은 유지합니다.
- 주차 state=1/2가 없는 경로이며 T·평행 주차 참조 실행을 끕니다.
- 신호등 [3], 회피 [5], 출구 [2]·state=3은 CSV의 현재 좌표·구역을 따라 연결합니다.
- **경로 04 원본 idx 622~828의 zone [6] 안에서만 ESTOP 감지 활성화**.
  구역 진입만으로 정지하지 않고, 장애물 감지 시 한 번 정지합니다.
- `course` 생략 시에도 `yongin_no_parking.csv`가 선택됩니다.
- 기존 주차 포함 코스는 `course:=yongin`을 명시했을 때만 선택됩니다.

런처의 `course: yongin_no_parking`과 `selected start CSV`를 확인합니다.
아래 주행 순서와 구간 표도 저장된 no-parking CSV 기준입니다.

### 3-4. 실제 적용되는 CSV와 주행 순서

```text
CMD에서 출발 01 또는 02, 일반 주행 속도 지정
  → GO → state=5 위치에서 실제 정차 후 3초 대기
  → 03 합류
  → 신호등 [3] → 웨이포인트 [1] → 회피 [5] (저장된 CSV 기준)
  → 회피 [5] 이탈 시 GPS 복귀 → [1] → 신호등 [3] → [1]
  → 04: 신호등 [3] → [1] → ESTOP 감지 [6] → [1]
  → [6]에서 장애물 감지 시 정지, 소실 확인 후 재출발 (주차 없음)
  → 05: [1] → 출구 검출 [2]에서 계속 주행
  → state=3 지점에서 정차 → 실제 정차 후 3초 판단
  → Left: 06 / Right: 07 / 무검출·동률: 06
  → 선택한 종료 경로 종점에서 정차·FINISH
```

우회전은 별도 우회전 상태가 아니라 zone [1]의 CSV 경로 추종으로 수행합니다.
06과 07은 대안 경로이며 연속 주행하지 않습니다.

| 용도 | 현재 파일 |
|---|---|
| 원본 웨이포인트 | `src/stack_gps/waypoints/yongin_no_parking.csv` |
| 전체 순서 | 실행 로그 폴더의 `no_parking_route/route_sequence.yaml` |
| 실행 웨이포인트 | `no_parking_route/yongin_no_parking_01.csv` ~ `07.csv` |
| 경로별 구역 | `no_parking_route/zones_no_parking_01.yaml` ~ `07.yaml` |
| 실제 선택 경로 목록 | 실행 로그 폴더의 `route_selected.yaml` |
| 주차 미션 | 비활성화 |
| 출구 YOLO | `src/stack_exit_decision/models/exit_decision_yolo26n.pt` |

GPS·회피는 선택한 시작 CSV의 첫 위도·경도를 공통 원점으로 사용합니다.
CSV를 변경하면 저장 후 런처를 다시 실행해야 새 실행 스냅샷에 반영됩니다.

### 3-5. 용인 구간별 동작과 속도

| 경로 | CSV 기준 구간·마커 |
|---|---|
| 01 / 02 | state=5: 각각 idx 90 / 88. 현재 YAML 정지점과 연결되어 실제 정차 후 3초 대기 |
| 03 | [3] 160~177 → [1] 178~222, 270~336 → [5] 337~462 → [1] 463~557 → [3] 558~581 → [1] 582~671 |
| 04 | [3] 298~314 → [1] 315~362 → [6] 622~828 → [1] 959~984 |
| 05 | [1] 10~83 → [2] 84~103 → state=3: 92 |
| 06 / 07 | [1] 0~24 → 각 경로 종점 |

| 상황 | 목표속도·정책 |
|---|---|
| 일반 주행 / [1] | CMD에서 지정한 `v_base` |
| 신호등 [3] | 최대 1m/s. 더 낮은 세션 속도·정지 명령은 유지 |
| 회피 | 기본 1m/s; 회피 종료 후 일반 속도로 복귀 |
| ESTOP 감지 [6] | 감지 대기는 일반 주행 속도, 장애물로 ESTOP 진입 시 0 |
| 출구 [2] 접근 | 일반 주행 속도로 계속 주행; state=3 도달 시 목표 0 |

zone [1]·[3]은 현재 station 또는 허용된 preview로 진입을 관측합니다.
현재 위치가 다른 특별 구간에 있으면 그 구간이 우선합니다. 일반 zone 확정은
독립 GPS 관측 5회씩 사용합니다. 회피 [5] 진입·이탈은 새 유효 위치 1회 기준이며,
ESTOP [6]과 출구 [2]는 preview가 아닌 현재 위치로 판정합니다.
주차 미션·출구 미션 진입 및 state=3 정차는 현재 위치 기준입니다.

#### 신호등·정지선

신호등 카메라 노출 보정은 `traffic_exposure_compensation=-9`입니다. 변경은 런처 재시작 후 적용되며 라인 카메라는 그대로입니다.

신호등 박스 검출 기준은 최초 **`confidence_threshold=0.65`**, 추적 유지 **`tracking_confidence_threshold=0.59`**입니다.
현재 모델은 수동 라벨 396장으로 추가 학습한 `src/stack_traffic/models/yolov8n.pt`이며,
SHA256은 `0e60f7f6996132fcced5f40d54e2311bf4cc63c6e54341d8921ecb0bb930a2e5`입니다.

적색 근거는 **HSV 적색 OR YOLO 신뢰도 ≥0.70 OR 유효 template 박스의 표시 점수 1.00**입니다.
template은 소수 둘째 자리 반올림 표시 기준이므로 0.999도 포함합니다. 두 점수는 다른 지표입니다.
최근 5프레임 중 유효 적색 관측이 3회 이상이면 확정하며, 2회 이하로 떨어지면 적색을 해제합니다.
YOLO 근거의 중간 프레임 유지·템플릿 투표에는 현재 추적 유효성 조건을 적용합니다.
정상 영상의 미검출은 적색 0표이며, 카메라 수신 실패는 정상적인 적색 해제 관측이 아닙니다.
사진 검토 화면의 ‘적색 미충족=초록불’ 표시는 실제 녹색 점등 확인을 의미하지 않습니다.

- zone [3]에서 검출·최대 1m/s 제한을 적용합니다. 이탈 후 다른 제한이 없으면 `v_base`로 복귀합니다.
- 신호등 bbox YOLO와 HSV 색상 판단, 정지선 segmentation을 사용합니다.
- 정지선 신규 검출 신뢰도는 0.30, 연속 추적 후보는 0.20입니다.
- 적색 확정 후 정지선 판단을 시작합니다. 적색+정지선 → 접근 → 정지선 소실 후 추정 이동거리로 정차합니다.
- 적색 해제로 재출발하며 초록 필수 확인 방식은 아닙니다. 적색·정지선 미검출만으로 정차를 보장하지 않습니다.
- 용인 런처는 한라대 정지선 단독 시험 옵션 `halla_stopline_test_enabled=false`입니다.
- 신호등 카메라는 일반 구간에서도 유지하고 판단만 구역 제한합니다. 출구 미션에도 카메라를 유지하고, 출구 YOLO는 라인 카메라의 `/perception/lane_image_raw` 원본 영상을 구독합니다. 출구 분기를 사용할 때는 `lane_enabled:=true`가 필요합니다.

#### 회피

현재 위치의 CSV `zone_id=5`, `inside_zone=1` 구간에서 회피 상태를 켭니다.
장애물이 없어도 구역 안에서는 회피 모듈이 GPS 웨이포인트를 따라갑니다.
유효한 새 GPS 관측에서 구역 밖으로 확인되면 즉시 회피 상태를 끄고,
진행 중인 회피 경로와 장애물 추적을 해제하여 GPS 주행으로 돌아갑니다.
GPS가 유효하지 않으면 구역 이탈로 처리하지 않습니다.

state=4, 경로 복귀 오차, 다음 [1] 진입, 회피 완료 신호는 더 이상 v2 회피의
진입·종료 조건이 아닙니다. [5]가 없는 CSV에서는 회피가 활성화되지 않습니다.
구역 재진입 시 다시 활성화됩니다. 라인 주행은 사용하지 않습니다.

#### 기존 주차 코스 참고 — 기본 no-parking에서는 실행하지 않음

state=1은 T자, state=2는 평행 주차입니다. 후보 선택·준비 응답을 기다리는 동안
정차할 수 있으며, 미완료 상태로 주행 CSV 종점에 도달했다고 주차를 건너뛰지 않습니다.
후진 경로 끝 도달 후 실제 정차 확인도 주차 성공 조건입니다.
완료 후 3초 정차하고 탈출하며, 탈출 완료와 다음 CSV 인계 때문에 별도 정차 시간을 추가하지 않습니다.
후방 거리 성공 판정의 로컬 보류 차이는 부록 A를 참고합니다.

#### 출구 선택

zone [2]에서는 `APPROACH=6`으로 검출하면서 주행합니다. idx 92의 state=3에
도달하면 `STOPPING=1`, 실제 정차 후 `JUDGING=2`로 3초간 새 관측을 집계합니다.
접근 중 관측은 정차 후 표수에 포함하지 않습니다.

| 신호 패턴: 왼쪽부터 | 클래스 | 선택 |
|---|---|---|
| 파랑–빨강–빨강 | `blue_red_red` / 1 | Left → 06 |
| 빨강–파랑–빨강 | `red_blue_red` / 0 | Right → 07 |
| 무검출·동률 | fallback | 06 |

신뢰도 0.5 이상, 프레임별 최고 신뢰도 클래스의 다수결입니다.
`SELECTED=3 → WAIT_ROUTE=4 → DONE=5`로 새 GPS 경로 응답을 확인하고 출발합니다.
자세한 계약: [LAST_MISSION_STATE.md](docs/LAST_MISSION_STATE.md).

#### 상위 ESTOP

과거 “독립 E-stop 삭제·상위 ESTOP 추가 예정” 요구는 현재 구현에 반영됐습니다.
독립 E-stop과 옛 시간 기반 후진을 실행하지 않으며 상위 MGM이 전이를 결정합니다.
현재는 **후진 회복 없이 정지 유지**하는 스테이션 방식입니다.

- CSV zone [6] (`inside_zone=1`)에서 현재 위치로 감지 활성화합니다. 구역 없는 경로는 비활성화입니다. state=6은 사용하지 않습니다. 구역 이탈은 감지를 끄지만 이미 정지한 ESTOP을 해제하지 않습니다.
- 전방 LiDAR의 차체 앞끝부터 **3m × 차폭 0.62m** 사각형에서 한 장애물의 폭 **18cm 이상**을
  새 유효 스캔 3회 확인하면 ESTOP에 진입합니다. 좌·우 센서 거리로는 진입하지 않습니다.
- 영역이 비었음을 새 유효 스캔 3회 확인할 때까지 목표 0으로 정지합니다.
  진입 후에는 작은 잔여 장애물도 빈 영역으로 취급하지 않습니다.
- 무효·미수신·0.35초 초과 스캔은 소실 확인으로 사용하지 않습니다.
- 소실 확인 후 이전 상태로 복귀하고 해당 스테이션 이탈 전 재진입을 금지합니다.
- 과거 estop_recovery 실행기는 런처에서 실행하지 않으며, 옛 후진·done 명령도 사용하지 않습니다.

[현재 ESTOP 계약](docs/ESTOP_STATION_20260919.md)을 따릅니다.

## 4. 주행 인가 코드

**터미널 2**에서 준비 상태 확인 후 상태 표시를 Ctrl+C로 닫고 실행합니다.

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
scripts/v2 go
```

정상 인가 시 주요 출력 예시:

```text
카메라 영상=True | GPS FIXED=True | MGM=True
라이다 정상=True | 미수신/무효=[]
출발 인가 완료 (MGM 확인). 실제 이동은 선택된 경로와 제어 조건에 따릅니다.
```

현재 go 도구에는 옛 `/scan` remap이 필요하지 않습니다. 신호등 판단 메시지는 zone [3]에서만
발행하므로 일반 출발 명령에 `--require-traffic`을 붙이지 않습니다.
주행 중 상태 확인은 `scripts/v2 state`로 합니다.

## 5. 주행 종료 및 다음 주행의 경로·속도 변경

터미널 2:

```bash
scripts/v2 stop
```

실제 정차를 확인한 뒤 터미널 1에서 Ctrl+C로 주행 런처를 종료합니다.
GPS는 유지됩니다. 다음 주행은 `prepare`를 다시 실행해 경로·속도를 정하고 `go`합니다.
`go`만으로 경로·속도가 바뀌거나 FINISH가 초기화되지는 않습니다. Ctrl+Z는 종료가 아닙니다.
GPS까지 종료하려면 주행 런처 종료 후 `scripts/v2 gps-off`를 실행합니다.

## 부록 A. 코드 변경 후 빌드와 보류 변경

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
git branch --show-current
git status --short
scripts/v2 build
scripts/v2 check
```

브랜치는 `integration/v2_main`, 설치는 이 폴더의 `install_v2`입니다.
`V2_INSTALL_READY`는 설치 확인이며 센서 수신 확인은 아닙니다.
소프트웨어 회귀시험이 필요하면 `scripts/v2 test`를 실행합니다.

**이 PC의 보류 변경:** 후방 주차 기준을 0.50m에서 0.20m로 줄이고 최근접 장애물로
성공 판정하는 수정은 사용자 지시로 커밋에서 제외했습니다. 현재 로컬 파일에는 남아 있어
이 PC의 실행은 커밋본과 다를 수 있습니다. 런북 작성 과정에서 이 값을 되돌리거나 승인하지 않았습니다.

```bash
git diff -- src/stack_parking/stack_parking/t_reference_parking.py
rg -n 'rear_stop:|reverse_speed:|forward_speed:|preview:' \
  src/stack_parking/stack_parking/t_reference_parking.py
```

## 부록 B. 상세 조회·로그

상세 ROS 조회는 기존 main overlay를 읽지 않은 새 터미널에서 준비합니다.

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
source /opt/ros/humble/setup.bash
source install_v2/local_setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 topic echo /perception/gps_path --field route --once
ros2 topic echo /adas/mgm_state --once
ros2 topic echo /adas/target_ref --once
```

`route.waypoint_csv`가 현재 적용 경로입니다. 노드의 시작 `waypoint_csv` 파라미터만 보고
진행 중 경로를 판단하지 않습니다. `last_mission_*` 필드로 출구 표수·선택·fallback을 확인합니다.

로그는 런처가 출력한 `drive_logs/v2_.../`에 저장합니다.
`route_selected.yaml`, `transitions.csv`, `zone_observations.csv`, `mission_events.csv`,
`vehicle_vector.csv`, `mgm_snapshots.bin`을 함께 보관합니다.
기본 `record=false`는 rosbag만 끄며 CSV·스냅샷은 남습니다. bag이 필요하면 준비 명령에 `record:=true`를 추가합니다.
현재 스냅샷은 **v44**이며 동일 버전 replay를 사용합니다.

| 증상 | 확인할 항목 |
|---|---|
| Ctrl+C 뒤 GPS가 계속 연결됨 | 정상: 표시만 종료. 완전 종료는 `gps-off` |
| FIXED인데 주차 준비가 안 됨 | IMU/헤딩 유효성, 현재 주차 요청 ID, ready·참조·차속 수신 |
| GO 후 목표속도 0 | `reference_motion_blocked`, 정지 사유, 주차 준비, 신호 상태, 경로 ACK, 상위 ESTOP |
| zone 밖 신호등 메시지가 없음 | 구역 제한에 따른 정상 동작; 카메라 영상과 판단 메시지를 구분 |
| 출구에서 06만 선택됨 | `last_mission_fallback`, 좌우 표수, 모델·원본 영상 토픽 수신 |
| 변경한 CSV/코드가 안 보임 | 실행 런처 종료 → 필요한 빌드 → 새 prepare. GPS 표시만 재시작하는 것으로는 불충분 |

참조: [용인 경로 통합](docs/YONGIN_ROUTE_INTEGRATION.md),
[스테이트 v09.17](docs/STATE_V09_17.md), [CAN 계약](src/bridge_dspace/PROTOCOL.md).
