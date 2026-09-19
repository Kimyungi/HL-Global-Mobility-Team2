# T 주차 실주행 코드 연결 — 2026-09-16

이 변경은 MATLAB 재생이 아니라 `stack_parking_node`와 MGM/기존 drive 런처의
실행 경로를 연결한다. 이 PC의 v2 통합 런처에 반영하며 재빌드된 설치본을 사용한다.
실차/CAN/dSPACE MPC 주행 검증을 완료했다는 뜻도 아니다.

## 순서와 제어권

1. 기존 GPS로 03 추종. 기존 T Mission Zone/state=1 진입 확인 시 MGM이 정차한다.
2. 실제 속도 정차 확인 후 보정된 좌측 b1 스캔 3개로 ref 1/2를 판별한다.
   적어도 하나가 비어 있다는 사용자 가정을 사용한다. 다른 경로의 반복 관측된
   점유 또는 해당 경로의 충분한 관측 증거가 필요하며, 무반사/NaN은 빈 공간이 아니다.
3. 선택을 고정하고 PREPARE/ACTIVATE 승인 후 **03 CSV의 남은 점을 전진**한다.
   Zone 폭 때문에 state 점 이전에 멈추는 경우도 기존 CSV의 앞선 8점을 사용한다.
4. GPS 종점과 실제 위치가 공통 접점에 도달하면 실제 정차 확인 후 후진한다.
5. 선택 CSV를 후진 추종한다. 후방 a2의 횡방향 벽 지지가 약 0.50 m 이내이면
   정차하며 실제 정차와 새 벽 스캔을 확인한다. 거리는 **후방 LiDAR 기준**이다.
   고립점, 도킹 전 장애물, 벽 없이 CSV가 끝나는 경우에는 성공이 아닌 FAULT 정차다.
6. 실제 정차 상태로 **3초** 기다린다. 대기 중 밀리면 정차 확인 후 3초를 다시 센다.
7. 용인 T주차는 선택 후보에 대응하는 별도 탈출 CSV를 전진 추종한다.
   평행주차는 실제 진입한 선택 경로 구간을 역순으로 전진한다.
8. 탈출 끝점에 도달하면 추가 정차 없이 `ParkingStatus.done=true`와 전진 명령을
   함께 발행한다. MGM 완료 확인까지 전진 reference를 유지한다.
   중간 경로 전환은 실측 정차 없이 다음 CSV 요청 → 새 GPS generation 확인 → 추종으로 진행한다.

`/adas/target_ref`와 CAN 발행자는 추가하지 않았다. 기존 MGM만 최종 기준점을 발행한다.
ParkingStatus는 기존 노드에서 하나만 발행하며, 용인 평행주차도 동일한 CSV 시퀀스를 사용한다.
1 m/s 전진·후진·도킹을 사용하고 MGM은 주차 모듈의 속도를
`v_base` 이하로 보존한다.
속도 부호가 하위 단의 전·후진 요청이다. 현재 VehicleVector에는 기어 ACK가 없으므로
**기어 ACK를 확인했다고 가장하지 않는다**. 부호 전환 전에 실제 속도 0을 확인한다.

## 런처 및 좌표

- 최신 맵 PR #105에 의존한다. 일반 `REAL_VEHICLE_integration_v2_drive.launch.py`는
  `t_reference_enabled=true`로 어댑터를 사용한다. 일반 parking 단독/다른 런처 기본은 false다.
- `t_reference_origin_csv`는 이번 실행에서 선택한 **첫 CSV**로 자동 설정한다.
  start=01/02/03에 따라 달라지는 `RoutePlan`의 원점을 그대로 사용한다.
- `t_reference_route_csv`는 `waypoints_halla_20260916_path_03.csv`이다.
  입력 GPS의 route ID, CSV 경로, sequence/instance/index가 다르면 진행하지 않는다.
- 주차 ref CSV의 lat/lon을 `stack_gps`와 동일한 111320 및 원점 위도 cos로 변환한다.
  원점이 다른 `east_m/north_m`를 GPS pose에 직접 더하지 않는다.
- GPS RTK fixed(4), 유효한 IMU 융합 차체 헤딩이 필요하다. 후진 COG를 차체 헤딩으로
  쓰거나 경로 접선을 실제 헤딩으로 대체하지 않는다.
- GPS HeadingFusion은 fresh TargetRef의 PARKING 동안 COG 보정/재시드를 보류한다.
  기존 IMU 정렬과 자이로 갱신은 유지하고, IMU 유실/미정렬을 숨기지 않는다.
  fresh non-Parking TargetRef에서만 COG 보정을 재개한다.
- raw a1/a2/b1은 `lidar_fusion_v2/config/fixed_geometry.yaml`의 장착·FOV·거리 보정을
  한 번 적용한다. 융합 스캔에 다시 보정하지 않는다.
- `/parking/t_reference_phase`에 순서/선택/정차 원인을 발행한다.

## 정지 및 호환성

- T Mission은 03 종점에서 자동 취소하지 않는다. 피드백 유실도 04로 건너뛰는 근거가
  아니며 요청을 유지한 정지가 된다. 미판별 탐색도 종점에서는 정지한다.
  평행 주차의 기존 종점 취소 정책은 그대로다.
- 이동 중 pose/scan/CAN 속도/제어권 유실, 역행 시계, 반대방향 속도, 추종 오차는
  FAULT 정차로 고정한다. 단순 센서 복구로 자동 재출발하지 않는다.
- 현재 v2 상위 ESTOP은 주차 중에도 적용된다. 두 전진 구간에는 주차 모듈의
  raw 전방 LiDAR 통로 검사도 적용한다. 관측 없는 통로는 unknown이다.
- 기존 MGM 외부 정지/CAN/운전자 최종 정지 게이트는 유지한다. 명시적인 MGM 취소는
  기존 계약대로 요청을 해제한다. `/parking/manual_command` stop은 요청을 해제하지 않고
  FAULT로 멈춘다. 요청 해제는 `/operator/cancel_mission`을 사용한다.
- 스캔·pose·속도는 취득 stamp로 검사한다. 반복 발행으로 reference_stamp를 새로 만들지 않는다.
- raw dump v33: bus 배치는 같지만 T 종점/속도 의미가 바뀌므로 과거 dump는 과거 빌드로 재생한다.
- PR #104의 독립 reverse core를 포함·확장한다. 이 통합 PR을 사용하면 #104를 별도 병합할 필요가 없다.

## 검증 범위와 현장 확인

오프라인에서 두 슬롯 전체 순서, 3초 대기, 실제 CSV 원점 01/02/03, 실제 b1 FOV의
합성 장애물 판별, ROS 어댑터 callback/메시지 계약, MGM 종점 제어권·04 ACK와
정차/속도 회귀를 검사했다. 어댑터 테스트의 ROS 전송/메시지 객체는 stub이다.
**실제 ROS graph/colcon 빌드, 센서·CAN·차량 및 MPC 폐루프는 실행하지 않았다.**

Python 코어/어댑터 테스트:

```sh
PYTHONPATH=src/stack_parking:src/stack_parking/test python3 -m unittest discover \
  -s src/stack_parking/test -p 'test_t*parking*.py' -v
```

MGM C++ 검사는 ROS 없는 코어 소스 6개를 각 test에 링크하여 실행 가능하다.
`parking_entry_test`, `fixed_speed_test`, `route_sequence_test`가 변경 계약을 검사한다.
`parking_search_route_ros_smoke.py`는 unresolved T 종점 유지 계약으로 갱신했으나
이번 Windows 환경에서는 ROS smoke를 실행하지 않았다.

현장에서는 먼저 CAN 송신 없이 ROS 연결/좌표/헤딩/전후진 부호를 확인하고,
통제된 저속 시험에서 GPS 노이즈에 따른 0.30 m 추종 오차, 실제 b1 시야,
후면 벽 기울기/폭/0.50 m 거리, 정차 지연 및 전진 복귀를 검증해야 한다.
CSV의 급곡률/접점 yaw는 참조 그대로 유지한다. 곡률 경고는 실차 추종 가능성의 보증이 아니다.

2026-09-19: 용인 T/평행 공통 대기는 3초. 탈출 끝점에서 EXIT_STOP을 거치지 않고
DONE과 전진 reference/속도를 함께 발행하며, MGM 완료 확인까지 유지한다.
중간 경로 전환에는 실측 정차를 요구하지 않는 기존 revised-v2 경로 전환을 사용한다.
