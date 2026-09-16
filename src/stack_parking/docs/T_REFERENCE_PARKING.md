# T parking: 두 CSV 중 선택하여 후진, 후방 벽 0.5m에서 정차

## 구현 범위

`stack_parking/t_reference_parking.py`는 ROS와 독립된 알고리즘 코어다.
이번 변경은 코어, 테스트, 문서만 포함한다. 실차 노드, MGM, CAN, 기존 CSV는 변경하지 않았다.
기존 `ParkingMission`과 병행 발행하도록 연결하지 않는다.

사용자 조건:

- 두 후보 중 적어도 하나는 비어 있다고 가정한다. 주차칸 네 모서리는 불필요하다.
- CSV 행 순서대로 **후진**한다. CSV의 접선 yaw에 pi를 더한 값이 차량 yaw다.
- 좌측 LiDAR로 두 경로의 차량 통과 영역을 검사한다.
- **후방 LiDAR 원점에서** 차량 뒤쪽 벽까지 약 0.50m가 되면 정차한다.
- 실제 정지를 확인해야 성공이며 자동 출차/일반 주행 복귀는 하지 않는다.

## 상태와 선택

`IDLE -> STOPPING -> SCANNING -> REVERSE -> WALL_STOP -> SUCCESS`

1. T 주차 요청을 latch한다. MGM에 정차/주차 제어권 요청을 먼저 보낸다.
   실제 PARKING 제어권 인계 확인, 최신 속도 및 pose/센서가 없으면 출발하지 않는다.
2. |실속도| <= 0.035m/s가 0.5초 유지되면 측정한다.
3. CSV를 0.05m 이하 간격으로 선형 보간하고 차량의 회전된 직사각형 footprint에
   0.08m 여유를 더한다. 직선·곡선과 차량의 앞뒤 돌출부 모두 포함한다.
4. 한 후보의 footprint에 실제 장애물 점이 있고 다른 후보에는 없으면
   '적어도 한 후보는 비어 있다'는 명시적 가정으로 다른 후보를 선택한다.
   둘 다 막힌 관측이면 가정보다 측정을 우선하여 정차한다.
5. 둘 다 장애물이 없으면 실제 측정 광선이 후보 footprint 표본보다 멀리
   도달한 비율을 비교한다. 70% 이상 관측된 후보 중 높은 쪽, 동률은 1번이다.
   이 표본 비율은 경험적 판단 지표이지 무충돌의 수학적 보장이 아니다.
   미수신/NaN/Inf/없는 각도 구간은 free ray로 만들지 않는다.
6. 같은 후보가 **서로 다른 3개 스캔**에서 선택될 때 고정한다.
   최초 위치 오차 <= 0.30m, 후진 차체 방향 오차 <= 20도도 확인한다.
   CSV 곡률반경은 기본적으로 경고만 출력하며 선택을 차단하지 않는다.
7. 주행 중에는 후보를 바꾸지 않는다. 경로 침입, 위치 이탈, estop, 제어권 상실,
   센서/속도/pose stale이면 FAULT로 정차 유지한다. 자동 재출발하지 않는다.

## 후방 완료 판정

- `/lidar/a2/scan`의 장착각과 거리 offset을 적용한 점군을 사용한다.
- 후방 차폭 범위의 근접 물체가 0.5m 이하이면 먼저 정차한다.
- 벽 판정은 최소 6점, 횡방향 폭 0.25m 이상, 직선 RMS <= 0.025m,
  차량 가로방향에서 기울기 <= 20도인 `x = a*y+b` 지지점으로 한다.
  고립 점/콘은 정차 근거일 수 있지만 성공 근거가 아니다.
- 경로 마지막 직선 2m 안에서 후방 벽 <= 0.5m를 감지해야 WALL_STOP으로 간다.
  더 이른 감지는 예상치 못한 장애물로 정차한다.
- 서로 다른 3회 벽 관측 + 실제 정지 0.5초 후 `parking_success=True`.
- 성공 후 속도 0을 유지한다. CSV 끝에 도착했지만 벽이 없으면 실패 정차한다.
- 0.5m는 **정차 명령을 내리는 관측 문턱**이다. 실제 최종 거리는 제동 지연에
  영향을 받는다. 최종 거리를 0.5m로 맞추려면 실차 제동거리/지연 보정이 필요하다.

수치들은 저장소 제원과 초기 설정을 이용한 시험값이다. 실제 왼쪽 FOV, 벽 재질,
노이즈, 정지 거리로 검증해야 한다. 합성 전체 원형 스캔으로 한 단위 테스트는
실차 좌측 가시성 검증을 대체하지 않는다.

## 연결 계약 (아직 미구현)

- `/lidar/b1/scan`: 좌측 선택. `/lidar/a2/scan`: 후방 벽/정차.
  `lidar_fusion_v2/config/fixed_geometry.yaml`에서 장착값과 offset을 읽어
  `scan_from_ranges()`에 전달한다. 보정된 cloud에 offset을 다시 적용하지 않는다.
  센서별 실제 acquisition timestamp와 실제 sensor origin을 유지한다.
- pose와 CSV east/north는 **동일한 ENU 원점**이어야 한다. `parking_map` 좌표를
  ENU로 착각하여 직접 넣지 않는다. 유효한 map-ENU 등록과 시간 동기화가 필요하다.
  `VehicleVector.x/y/yaw`를 GNSS ENU 값이라고 가정하지 않는다.
- 실제 속도는 `/vehicle/vector.v`. pose 유효성 판단은 ROS adapter의 책임이다.
- 현재 MGM 진입 조건은 `gps_parking_zone && parking_space_found`다.
  STOPPING을 가능하게 하려면 별도 주차 요청/phase 계약과 MGM 분기를 추가해야 한다.
  빈 공간 판별 전 `space_found=True`로 거짓 보고하여 우회하지 않는다.
- `Output.request_stop`은 속도 0 요청이다. `phase`는 주차 제어권 유지에 사용한다.
  `parking_owned`는 실제 MGM 응답을 확인해서 입력해야 한다.
- 선택 후 `reference`는 현재 차량 기준 목표 1점이고 `v_suggest`는 음수다.
  기존 parking localization의 dx/dy/dyaw/update 계약도 그대로 보존해야 한다.
- **parking_success를 ParkingStatus.done에 바로 연결하지 않는다.** 현재 MGM의
  done은 주차를 끝내고 LANE/WAYPOINT로 복귀하는 의미다. 성공 후 HOLD를 위한
  명시적인 상위 상태/출차 허가가 필요하다. 새 성공 알림만 별도로 전달한다.
- 생성형 decision backend/Simulink를 쓰는 경우 동일 상태 전이 수정과 parity 테스트가
  필요하다. Python 코어만 실행해 실제 T parking 진입/정차가 된다고 간주하면 안 된다.
- 좌측 선택용 데이터만으로 주행 중 전 방향 장애물을 보호할 수 없다.
  기존 전·후 인지, estop 및 MGM 신선도 watchdog은 계속 유지해야 한다.

## 현재 CSV와 하위 제어기

제공된 parking_ref_01/02.csv의 곡률반경은 이산 접선 계산으로 각각 약 0.951m/0.924m다.
저장소 차량 설정의 반경 기준 1.15m보다 작지만, 사용자는 하위 dSPACE의 5차 궤적 생성과
MPC가 이 참조를 추종하도록 요청했다. 이에 **기본 설정에서는 출발을 차단하지 않고**
`Output.warnings`에 후보별 수치를 계속 제공한다. CSV 자체와 차량 제원은 바꾸지 않는다.
엄격한 기하 검증이 필요하면 `Config(enforce_min_radius=True)`로 명시적으로 켠다.

참조 경로와 실제 실행 궤적은 다를 수 있다. 하위 MPC가 있다고 해서 기계적 조향 한계나
충돌 여유가 없어지는 것은 아니다. 실제 추종 가능성, GPS/pose 오차, 제동거리는 실차 검증
대상이다. 이 PR은 GPS 필터나 MPC를 변경하지 않으며 센서 freshness, 장애물, 시작점/방향,
추종 오차 정차 조건도 해제하지 않는다. 0.30m 위치 오차와 45도 방향 오차의 추종 가드는
초기 시험 설정이며 노이즈/실차 로그를 바탕으로 별도 조정해야 한다.

## 오프라인 테스트

저장소 루트에서 Python + NumPy 사용:

```powershell
$env:PYTHONPATH = (Join-Path (Get-Location) 'src/stack_parking')
python -m unittest discover -s src/stack_parking/test -p test_t_reference_parking.py -v
```

```python
from stack_parking.t_reference_parking import load_reverse_csv, TwoReferenceParking

paths = (load_reverse_csv('parking_ref_01.csv'), load_reverse_csv('parking_ref_02.csv'))
core = TwoReferenceParking(paths)
core.trigger()  # T 주차 요청. 이것만으로 차량을 제어하지 않는다.
# core.tick(now, pose_enu, pose_stamp, speed, speed_stamp,
#           left_scan, rear_scan, parking_owned=actual_mgm_parking_state)
```
