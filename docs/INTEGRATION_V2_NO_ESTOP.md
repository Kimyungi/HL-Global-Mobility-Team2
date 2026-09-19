# Integration v2 — LiDAR E-stop 제외 시험 런처

사용자가 요청한 별도 시험 진입점은 `scripts/v2 vehicle-no-estop`이다.
`REAL_VEHICLE_integration_v2_no_estop.launch.py`가 기존 통합 런처를 재사용하며,
`stack_estop`의 정적·동적 장애물 정지와 해당 입력 미수신 보정을 제외한다.
정상 런처인 `scripts/v2 vehicle`은 기존 E-stop을 계속 사용한다.

출발 인가 대기, `/operator/stop`, CAN 고장과 재인가 래치, Reference 유효성 검사,
신호 정지, Mission/경로 순서, 회피의 TTC 정지, 종료 시 can_zero는 유지한다.
따라서 이 런처에서도 다른 정지 조건이 충족되면 목표속도가 0이 된다.
일반 후진 탈출은 0=비활성만 허용한다. 주차 Mission의 후진과는 별도 기능이다.
MGM 파라미터 `lidar_estop_enabled`는 startup-only이며 운전 중 전환하지 않는다.

2026-09-13 공유 통합 런처에서 a1 방향 불일치를 수정했다. `parking_enabled=true`이면
E-stop yaw는 v2 보정 -87도, 회피의 원본 전방 각도는 +87도를 사용한다.
일반/이 시험 런처에 공통 적용되며 자세한 재현·검증은
[전방 라이다 좌표 수정 기록](FRONT_LIDAR_ALIGNMENT.md)을 따른다.
이 문서는 실차 주행 가능 판정이나 물리 비상정지 검증을 대신하지 않는다.
실제 차량의 물리 비상정지·하위 제어기는 변경하지 않는다.

## 설치와 시작

새 MGM 바이너리와 런처가 필요하다. 기존 통합 런처를 종료하고 정지한 뒤 빌드한다.
기존 프로세스가 실행 중일 때 이 런처를 추가로 띄우지 않는다.

```bash
cd "$HOME/Desktop/HL-Global-Mobility-Team2-v2_main"
./scripts/v2 build
```

아래는 현재 사용자가 선택한 한라대 **01→03→04→05→07** 구성이다.
베이스/RTCM 중계는 기존 절차를 사용한다. 이 명령은 CAN TX를 활성화하며 go 전에는 정지 대기한다.

```bash
./scripts/v2 vehicle-no-estop \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  route_sequence_file:="$PWD/src/stack_gps/waypoints/halla_route_sequence.yaml" \
  route_start_id:=01 route_end_id:=07 \
  parking_enabled:=true \
  t_parking_zone_ranges:="[0]" parallel_parking_zone_ranges:="[0]" \
  zone_enter_confirm_samples:=5 zone_exit_confirm_samples:=5 \
  parking_search_zone_only:=true escape_after_cycles:=0 avoid_zone_only:=false \
  usb_speed:=high camera_fps:=10 \
  traffic_enabled:=true traffic_depth_enabled:=false \
  traffic_yolo_image_size:=320 traffic_yolo_inference_interval:=2 \
  traffic_red_phase_yolo_inference_interval:=3 traffic_stopline_yolo_image_size:=320 \
  traffic_require_stop_gate:=false traffic_stop_y_ratio:=0.0
```

기동 로그에서 `LiDAR E-stop input DISABLED`와 `출발 대기 모드`를 확인한다.
주행 인가는 기존 `scripts/v2 go --require-traffic --ros-args -r /scan:=/lidar/a1/scan`을 사용한다.
인가하면 바로 움직일 수 있다. 정지는 기존 `scripts/v2 stop` 또는 통합 런처의 Ctrl-C 경로다.
물리 비상정지는 차량 자체의 절차를 사용한다.

로그 폴더는 `drive_logs/v2_no_estop_...`로 구분한다.
MGM 실행 파라미터의 `lidar_estop_enabled=false`와 기동 로그로 시험 구성을 확인한다.
원복은 이 런처 종료 후 기존 `scripts/v2 vehicle` 명령으로 재기동한다.

## 검증

사용자의 "검증 없이 런처만 작성" 요청에 따라 이번 변경의 빌드·테스트·실차 실행은 생략했다.
런처와 MGM 입력 선택 코드만 작성했으며 동작 확인을 완료했다는 뜻은 아니다.
사용 전 위 빌드 절차가 필요하다. 현재 실행 중인 차량 프로세스에는 적용하지 않았다.
