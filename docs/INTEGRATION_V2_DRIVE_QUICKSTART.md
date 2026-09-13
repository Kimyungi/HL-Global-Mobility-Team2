# Integration v2 통합 주행 + RViz 실행

> 2026-09-14 주행 준비 변경: `scripts/v2 prepare`는 전체 스택을 준비하고,
> GPS/RTCM은 별도 상주 서비스로 유지한다. `go`는 카메라 프레임 **또는**
> GPS FIXED(4)로 인가하고, `stop → go`는 GPS를 재시작하지 않는다.
> 일반 CSV 구간은 GPS 상실 시 유효한 카메라 경로로 주행한다. raw dump v26.
> 상세: [주행 준비](DRIVE_PREPARATION.md). 아래 과거 전체 센서 필수 출발 조건보다 우선한다.

2026-09-14 · `integration/v2_main` · 한라대 코스 · 현재 1/10 무인 RC 시험 구성.

## 파일 위치

워크스페이스: `/home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main`

| 역할 | 워크스페이스 기준 경로 |
|---|---|
| 한 번에 실행하는 새 런처 | [REAL_VEHICLE_integration_v2_drive.launch.py](../src/adas_mgm/launch/REAL_VEHICLE_integration_v2_drive.launch.py) |
| 깨끗한 v2 ROS 환경으로 실행 | [scripts/v2](../scripts/v2), `drive` 하위 명령 |
| 재사용하는 통합 노드 구성 | [REAL_VEHICLE_lane_gps_can.launch.py](../src/adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py) |
| 통합 RViz 런처 | [integration_v2_view.launch.py](../src/adas_mgm/launch/integration_v2_view.launch.py) |
| RViz 화면 구성 / 표시 코드 | [integration_v2.rviz](../src/adas_mgm/config/integration_v2.rviz) / [integration_view.py](../src/adas_mgm/tools/integration_view.py) |
| 7개 CSV와 대응 Zone의 경로 순서 | [halla_route_sequence.yaml](../src/stack_gps/waypoints/halla_route_sequence.yaml) |

새 런처는 기존 통합 제어 코드를 재사용한다. 실행 시 선택된 CSV/Zone 목록을 새 로그 폴더의
`route_selected.yaml`로 저장하고 그 목록을 GPS와 MGM에 전달한다.

## 1. 준비

차량 PC 터미널 **2개**: A는 통합 실행, B는 점검·출발·정지 명령이다.
베이스 PC의 기존 RTCM 송출은 별도로 실행해 둔다. 이 런처가 켜는 것은 **차량 PC의**
`/dev/ttyRadio → TCP 2101` 중계이며 베이스 설정이나 베이스 프로그램을 바꾸지 않는다.

```bash
cd "$HOME/Desktop/HL-Global-Mobility-Team2-v2_main"
./scripts/v2 check
ls -l /dev/ttyRadio /dev/ttyRover /dev/lidar_front /dev/lidar_rear /dev/lidar_left /dev/lidar_right
ip -details link show can0
df -h "$PWD" /dev/shm
```

기존 통합 주행과 차량 RTCM 중계를 종료한 상태에서 A를 실행한다. 별도 LiDAR/카메라/CAN
노드를 중복 실행하지 않는다. 이미 차량 RTCM 중계를 유지할 경우 `start_rtcm:=false`를 추가한다.
메시지·C++·패키지를 새로 받은 PC는 먼저 `./scripts/v2 build`로 `install_v2`를 갱신한다.

## 2. A — 1번 출발, 7번 복귀: 통합 노드 + RViz

```bash
cd "$HOME/Desktop/HL-Global-Mobility-Team2-v2_main"
./scripts/v2 drive \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  start_waypoint:=01 end_waypoint:=07
```

이 명령으로 차량 RTCM 중계, GPS/IMU, 차선 카메라, 신호등·정지선 카메라, 4방향 LiDAR와 융합,
주차 SLAM·벽 수집·경로 생성, 회피, E-stop, MGM, CAN bridge, 통합 RViz가 시작된다.
**CAN 송신은 시작되며, 출발 인가는 B에서 별도로 보낸다.** 기존 `wait_go` 조건을 유지한다.

경로는 **01→03→04→05→07**이다. 01→02→03→04→05→06→07 순서가 아니다.
01/02는 출발 선택, 06/07은 복귀 선택이며 운행 중 자동으로 선택지를 바꾸지 않는다.

| 원하는 운행 | A 명령의 선택 인자 | 실제 순서 |
|---|---|---|
| 1번부터 7번 복귀 | `start_waypoint:=01 end_waypoint:=07` | 01→03→04→05→07 |
| 2번부터 6번 복귀 | `start_waypoint:=02 end_waypoint:=06` | 02→03→04→05→06 |
| 3번부터 재시작 | `start_waypoint:=03 end_waypoint:=07` | 03→04→05→07 |
| 4번부터 재시작 | `start_waypoint:=04 end_waypoint:=07` | 04→05→07 |

중간 재시작은 현재 launch를 종료한 뒤 같은 A 명령에서 `start_waypoint`만 바꾼다.
**해당 CSV의 현재 위치 근처 station을 초기화**하며 CSV 첫 점까지 되돌아가게 하는 명령은 아니다.
기존 세션의 Mission/경로 진행 기억과 GPS 헤딩 융합 상태도 새 노드에서 초기화된다.

## 3. B — 점검 후 출발

```bash
cd "$HOME/Desktop/HL-Global-Mobility-Team2-v2_main"
./scripts/v2 go --require-traffic --ros-args -r /scan:=/lidar/a1/scan
```

`RTK: RTK FIXED`와 입력 점검을 통과하면 출발 인가가 발행된다. 실패하면 원인을 해결한 뒤
같은 명령을 다시 실행한다. 4-LiDAR 구성에서 go의 전방 scan 점검은 위 remap이 필요하다.
go는 입력 수신·FIXED 점검이며, 이후 실제 속도는 MGM의 정지·신호·주차 조건에 따라 달라진다.

```bash
./scripts/v2 state
```

`top=1`은 DRIVE, `active_safe_stop_reasons`는 정지 이유 mask다.
모든 상황의 실제 목표속도는 `/adas/target_ref`의 `v_ref`, 실제 차량 속도는 `/vehicle/vector`의 `v`다.

## 4. RViz 표시

차량을 중심으로 고정한 화면에 다음 항목이 함께 표시된다. 입력이나 유효 목표가 없으면 해당 표시도 없다.

| 항목 | 표시 |
|---|---|
| GPS 경로와 preview point | 주황색 `GPS` |
| 차선 카메라 목표점 | 청록색 `CAMERA` |
| MGM에서 실제 선택한 목표점 | 분홍색 `MGM` |
| 회피 노드가 생성한 목표점 | 노란색 `AVOID_TARGET` — 실제 선택 여부는 MGM 목표와 비교 |
| 주차 목표점 | 초록색 `PARKING` |
| LiDAR | 앞 a1 / 뒤 a2 / 왼쪽 b1 / 오른쪽 b2 |
| 주차 | 주변 SLAM 지도, 주차 공간·계획 경로, 좌측 수집점·벽 후보·±12cm 경계 |
| 카메라 / 신호 | 차선 영상, 신호등·정지선 영상, 신호 색, 유효 dist 기반 정지선 위치 |
| 주차 초기 수집 | 차량 좌측 방향선, 정지 대기·5프레임 수집 상태와 실제 속도 |

통합 launch에 포함된 RViz 창을 닫으면 기존 view 런처의 종료 이벤트가 전체 launch에도 전달된다.
주행 중 창을 닫지 않는다. RViz를 별도로 관리하려면 A에 `rviz:=false`를 추가하고 다른 터미널에서:

```bash
cd "$HOME/Desktop/HL-Global-Mobility-Team2-v2_main"
./scripts/v2 view
```

## 5. 현재 시험 설정과 전환 조건

- 일반 목표속도 `v_base=1.0m/s`; 정지·신호·주차·후진 제약은 기존 MGM이 적용한다.
- 일반 주행 회피와 전방 LiDAR E-stop ON. 주차 ACTIVE에서는 기존 회피/E-stop 마스킹 유지.
- E-stop 지속 1000틱(정상 100Hz에서 약 10초) 후 자동후진 설정 유지:
  `v_escape=-0.8m/s`, 최대 162틱. 명령상 약 1.3m이며 실제 이동거리 보증은 아니다.
  현재 RC 설정은 `escape_require_rear_clear=false`다.
- 회피 종료 후에도 AVOID 상태에서 GPS를 추종하며 횡오차 ≤0.1m, 방향오차 절댓값 ≤20°에서 탈출한다.
- Zone 진입·이탈 확인은 각각 새 GNSS 표본 5개다.
- 주차 Zone 진입 시 정지 → fresh CAN `abs(v_act)≤0.1m/s` → 새 LiDAR 5프레임 수집 →
  현재 CSV GPS 탐색 재개. 준비 완료 후 주차 경로로 인계한다. 자세한 기준은
  [주차 수집 명세](PARKING_LEFT_WALL_ACQUISITION.md)를 따른다.
- 주차 완료 또는 현재 CSV 종점까지 미완료 처리 후 기존 MGM 경로 전환 조건으로 다음 CSV를 요청한다.
  마지막 선택 경로에서 전체 FINISH다. 접속점에 임의 직선을 새로 넣지 않는다.
- 신호등 노출 보정 `-2`, 두 카메라 10fps, 신호등 위치 추가학습 모델은 현재 설치된 모델을 사용한다.
- 현장 실행과 동일하게 `traffic_stop_y_ratio=0`, `traffic_require_stop_gate=false`다.
  **이 값이 MGM Signal 정지를 끈다는 의미는 아니다.** `traffic_state_enabled=true`에서
  적색·정지선 인지와 기존 소실 거리 조건으로 정지한다. 정지선 검출 자체도 켜져 있다.

## 6. 정지·종료·기록

B에서 주행 정지:

```bash
./scripts/v2 stop
```

전체 종료는 **A에서 Ctrl-C 한 번** 누르고 CAN zero 및 노드 종료를 기다린다.
같은 launch에 포함한 RTCM 중계와 RViz도 종료된다. 외부에서 따로 켠 베이스 PC 송출은 별도로 종료한다.

기본 로그는 `drive_logs/v2_<시각>/`에 새로 생성된다. `route_selected.yaml`, MGM snapshot,
transitions, mission/zone 관측, GPS lateral, CAN vehicle vector 및 차선 CSV를 기록한다.
**기본 `record=false`는 rosbag만 끄며 CSV/raw snapshot은 기동 시부터 기록한다.**
원본 rosbag도 필요하면 A에 `record:=true`를 추가한다.

저장 폴더를 바꾸려면 A에 `run_log_dir:=/충분한_디스크/새_세션명`을 추가한다.
기존 로그 덮어쓰기를 막기 위해 **아직 없는 폴더**여야 한다. ROS 자체 로그 위치는
`ROS_LOG_DIR=/충분한_디스크/ros ./scripts/v2 drive ...`로 지정할 수 있다.

2026-09-14 작성 당시 차량 PC 디스크가 거의 가득 찼다. 최근 세션의 `/dev/shm/fma-v2-drive-*`는
RAM 임시 기록이며 재부팅하면 사라진다. 영구 기록 위치로 간주하지 않는다.
기록 공간 확보 없이 새 장시간 기록을 시작하지 않는다.

## 7. 경로가 뒤집혀 보일 때

차량 중심 RViz의 GPS 경로는 차량 heading을 사용해 회전한다. 최근 로그에 초기 접선 heading
약 −133°에서 융합 heading 약 +48°로 바뀐 기록이 있었다. CSV 순서가 뒤집힌 것과 구분해야 한다.
`lateral.csv`의 `heading_deg`, `heading_src`, `imu_yaw_deg`, `offset_deg`를 함께 확인한다.
재시작은 융합 상태 초기화이며 **180° 뒤집힘 원인 수정이 아니다**.

## 문서/런처 확인 범위

새 진입점은 경로 선택·로그 위치·RTCM/RViz 실행을 묶는다. 이번 추가 작업에서는 현재 실행 중인
주행 노드를 재시작하지 않았다. 경로 조합 12개, 잘못된 선택 거부 4개, CAN 확인 토큰 거부,
4→5→7의 RTCM/RViz 포함 구성까지 **오프라인 18항목을 통과**했다.
`scripts/v2 drive --show-args`, shell 문법 및 diff 검사도 통과했다. 새 통합 진입점 자체의
하드웨어 실행은 하지 않았다.
상세 현장 준비는 [한라대 런북](../src/adas_mgm/RUNBOOK_integration_v2_halla.md)을 함께 본다.
