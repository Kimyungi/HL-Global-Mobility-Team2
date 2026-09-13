# 주차 좌측 벽 기준선 — 2026-09-13

통합 `parking.launch.py`는 `parking_lateral_wall` 인식 노드를 함께 실행한다.
현재 단계의 출력은 좌측 벽 기준선과 경계/선택점이다. 기존 `space_detector`의
주차 공간 선택이나 `path_planner`의 충돌 판정을 이 선으로 대체하지 않는다.
이 노드는 차량 목표값·Mission 명령을 발행하지 않는다.

입력은 `/unified_lidar/cloud`의 모든 센서 점이다. 특정 센서 ID로 제한하지 않는다.
기준 위치는 `lidar_fusion_v2/config/fixed_geometry.yaml`의 b1 장착 위치
`x=0.215329, y=0.211549m`다. 센서 yaw를 쓰지 않고 차량 +y(왼쪽 90°)를 사용한다.

- 융합 설정의 기존 1° 각도 간격으로 b1 위치에서 바라본 점을 재분류한다.
- 중심 bin과 양옆 5개 bin씩, 최대 11개 점을 사용한다. 같은 bin은 가장 가까운 점 하나만 사용한다.
- 관측이 없는 bin을 더 멀리 떨어진 각도의 점으로 채우지 않는다.
- 기존 벽 초기 탐색 거리 0.3~3m, 최소 6점, 진행 방향 대비 최대 45°를 사용한다.
- 중심 bin 관측을 반드시 요구하고, 센서 generation이 다른 5개 연속 유효 프레임을 받는다.
- 프레임마다 SLAM pose로 선택점을 지도 좌표로 옮겨 이동을 보상한 후 직선을 적합한다.
- 입력 공백·역행·무효 SLAM·중심점 부재는 미확정 연속 카운트를 초기화한다.
- 직선의 수직 잔차 한계와 경계선은 기존 `wall_line_offset_m=0.12`, 즉 ±12cm다.
- 11개 국소 각도 bin 방식에서는 기존 전체 벽 탐색의 최소 길이 50cm를 요구하지 않는다.
  짧은 선택 창을 모은 선이므로 표시 길이가 실제 관측 벽 길이라는 뜻은 아니다.
- 확정선은 map에 고정된다. 새로운 MGM PREPARE(request ID 증가) 또는
  `/parking/left_wall/reset` Bool true에서 다시 획득한다.

출력:

- `/parking/left_wall/markers`: 선택점, 5프레임 지지점, 수직 탐색 방향선,
  녹색 기준선, 주황색 ±12cm 경계, 카운트/상태 문자.
- `/parking/left_wall/diagnostics`: frames, locked, fresh, selected_points 및 원점.
- 입력 소실 시 확정선은 회색 이력으로만 남고 fresh=false다.
- 통합 RViz bridge가 위 마커를 차량 기준으로 변환한다.

## 주차 진입 정지 → 수집 → GPS 탐색 재개

`parking_zone_entry_active=true` 통합 경로에서는 PARKING 진입 틱에 목표속도와
내부 ramp를 즉시 0으로 한다. GPS 목표점은 보존한다. fresh CAN `/vehicle/vector.v`의
절댓값이 0.1m/s 이하가 된 뒤에만 새 LiDAR 프레임을 센다. 후진 -0.1m/s도 경계에 포함한다.
속도 미수신·0.2초 초과·비유한 값 또는 수집 중 속도 초과는 미완료 카운트를 초기화한다.

수집은 정지 확인 이후 timestamp의 서로 다른 유효·비어 있지 않은 cloud 5개다.
새 요청 이후의 유효 SLAM pose도 요구한다. 재발행·과거·정지 전 queued scan은 제외한다.
5프레임 **수집 수**와 5프레임 **벽 적합 성공 수**는 별도다. 벽이 없거나 적합 실패해도
수집을 마치면 GPS로 출발하여 탐색을 이어간다. 없는 벽을 만들어 표시하지 않는다.
확정에 실패한 벽 적합은 후속 관측으로 계속 시도한다. 기준선 확정 시 ±12cm 경계를 표시한다.

`/parking/left_wall/status`는 request ID, 수집 수, 완료 여부, 실제 속도/유효성/정지 여부,
단계를 발행한다. ParkingStatus가 이를 전달하며 MGM은 현재 request/mode의 fresh 완료
응답으로만 정지를 해제한다. 재출발 뒤 속도가 올라가도 같은 요청의 완료는 유지한다.
응답 소실 시 GPS 탐색은 다시 정지한다. 새 요청은 정지와 수집을 처음부터 수행한다.

5프레임 완료만으로 주차 maneuver를 시작하지 않는다. 기존 공간/경로/localization 준비와
ACTIVATE 응답을 받아야 Parking reference로 전환한다. 완료 이전 ready 응답도 인계하지 않는다.
주차 완료 또는 현재 CSV 종점에 의한 종료, 외부/CAN 정지와 기존 Signal 제약은 유지한다.

통합 RViz는 선택점·벽 후보/확정선·±12cm 경계와 함께 대시보드에 `LEFT SCANS n/5`,
`WAITING_FOR_STOP`/`WAITING_FOR_CAN_SPEED`/`COMPLETE_GPS_RESUME` 및 실제 속도를 표시한다.
기존 GPS·카메라·4방향 LiDAR·주차 경로/공간 마커도 유지한다.

메시지·MGM·Parking을 함께 빌드한 다음 통합 노드를 새로 실행해야 적용된다.
센서가 연결되지 않은 현재 세션에 출발 인가를 새로 발행하지 않았다.
CoreSnapshot 변경으로 raw dump는 v24다. v23 dump는 당시 빌드로 재생한다.

검증: 새 core 시험 8개와 기존 wall-gap 시험 4개 통과. 5개 generation 경계,
재발행 제외, 중심점 소실, 이동 보상, 가까운 가림점 우선, 불일치 프레임,
시간 역행/공백, 확정선 보존과 reset을 검사한다.


추가 검증: CTest 21/21, 관련 Python 27/27 통과. 정지 전/중 속도, 후진 속도 경계,
중복/정지 전 scan, 새 요청, 4→5 수집, 벽 미검출 상태의 수집 완료, 상태 freshness를 검사했다.
격리 ROS mock으로 진입 정지→완료 후 GPS 재출발→ready/ACTIVATE 인계 및
03 CSV 주차 미발견→종점→04 자동 전환을 확인했다. 실차 센서로 수행한 검증은 아니다.
