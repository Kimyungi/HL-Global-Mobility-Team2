# stack_avoid_v2 — 벽 중앙 회피와 MGM 연결

2026-09-15. [설계안](../../docs/NEW_AVOID_V2_DESIGN.md)의 첫 구현이다.
기존 `stack_avoid`의 gap/station/cubic 코드를 호출하지 않는 C++ 코어와 ROS 입력부를 추가했다.
GPS 노드에는 실제 사용 중인 다점 경로 발행을 추가했다. 기존 LINE/GPS 추종 계산과 zone 상태 전이는 유지한다.
통합 launch는 이 provider를 기본 실행하고 MGM이 유효한 계획을 소비한다.
[운영 연결 계약과 검증 범위](../../docs/AVOID_V2_MGM_INTEGRATION.md)를 참조한다.

## 현재 경로 생성 — 벽 사이 중간점 연결

현재 코어는 **연석과 고정 차량을 같은 점유 경계로 취급하는 벽 중앙 경로 생성기**다.
이전 GPS 차단 검출·추월 탐색·복귀 분기를 교체했다.
[벽 중앙 설계](../../docs/AVOID_V2_WALL_MIDPOINT.md)와 [현재 실패 수정 결과](../../docs/AVOID_V2_FIXES.md)를 참조한다.

1. zone 밖에서는 기존 GPS가 주행한다. zone 진입 후 LiDAR ray로 free/occupied/unknown을 갱신한다.
2. GPS 진행 방향에 수직인 단면을 0.2m 간격으로 두고, 단면의 관측 자유 공간을 0.05m 간격으로 읽는다.
3. 차폭과 양쪽 0.15m 여유가 들어가는 자유 구간을 찾고, 이웃 단면과 이어지는 통로를 동적 계획법으로 선택한다.
4. 선택한 양쪽 경계의 중간점을 연결한다. 차량 앞부분의 선행 진입을 고려한 제한과 평활화 강도를 유한 개 비교한다.
5. 연결 안내선을 조향 지연이 있는 bicycle 모델로 따라가며 차체·조향·횡가속과 표본 사이 제동 공간까지 검사한다. 한쪽 통로가 실패하면 반대쪽 후보와 저속 후보도 비교한다.
6. 통로 폭·곡률·조향 변화량에 따라 미리 감속하며, 새 관측에서도 안전한 기존 경로의 속도 계획을 유지한다.
7. 검증된 다점 경로·점별 속도·100ms 뒤 요청 속도와 경로 길이 1m 앞의 목표점을 출력한다. 실패하면 HOLD다.

GPS는 다점 경로의 진행 station·갈림길 선호·출구 판정에 사용한다. 장애물 ID·차량 개수·
추월 방향 상태는 경로 생성에 쓰지 않는다. `WALL_FOLLOW=6`이 새 주행 상태이며,
`maneuver_active`는 false다. `obstacle_detected`는 GPS 부근 점유 여부의 과거 진단 필드로만 남는다.
zone 안에서는 `gps_follow=false`, zone 밖 GPS 소유권에서만 true다.

점유 지도와 입력 계약은 유지한다. 자유 셀은 관측 시각부터 만료하고 점유 셀은 두 번의
독립된 새 clear 관측 전까지 남는다. 미관측은 통과 가능한 공간으로 채우지 않는다.
지도 경계는 추가 통과 금지 영역이다. 실제 연석은 LiDAR 점유로도 반영되며,
LiDAR 벽을 지도보다 안쪽에 둔 시험으로 지도 경계만 따라가는 구현이 아님을 확인했다.

3원·거리장은 빠른 통과 확인용이다. 이 근사로 통과를 확인하지 못하면 회전된 차량 사각형과
점유/미관측/도로 밖 격자 셀을 직접 대조한다. 차체 여유와 표본 사이 이동 여유는 유지한다.
제동 검사는 기존처럼 측정 조향을 유지한 정지 궤적 모델이다.

GPS `/perception/gps_route`, 도로 경계 `/avoid_v2/course`, zone `/perception/gps_path`와
원본 LiDAR/VehicleVector 입력 및 출구 후 3개 독립 관측 확인 조건은 유지한다.
20ms 코어 예산, 35ms ROS callback 무효화, 별도 watchdog도 유지한다.

## 현재 실행 경계

통합 v2 launch는 `control_enabled=true`로 실행한다. MGM의 신선한 AVOID_ACTIVE 피드백에 따라
계산하며 MGM은 `/avoid_v2/plan`의 유효기간과 입력 계약을 검사해 기존 TargetRef/CAN 경로에 전달한다.
기존 zone 진입·GPS_RETURN·표식 소모 상태 전환은 유지한다. 계획 만료/HOLD 시 출력 속도는 0이다.
운영 모드의 완료는 장애물 관측 이후 GPS 정렬과 관측된 안전한 연결 경로를 3회 확인한 결과다.
코스 파일이 없으면 GPS 주변의 계산용 지도를 만들며, 지도 안을 자유 공간으로 가정하지 않는다.

`shadow.launch.py`는 계속 그림자 전용이다. `control_enabled=false` 계획은 MGM이 소비하지 않는다.
별도 watchdog은 진단용이며 운영 정지는 MGM 자체의 매 틱 계획 만료 검사로 처리한다.
이 패키지는 `/operator/go`나 CAN에 직접 발행하지 않는다.

아직 구현/검증이 남은 항목:

1. 실차에서 계획 만료/HOLD에 따른 MGM 속도 0 명령과 물리적 정지 응답의 검증.
2. dSPACE의 실제 1점 추종 모델 식별, 명령 적용 시점 보정, 실제 제동 조향 동작의 검증.
   현재 제동 검사는 측정 조향을 유지하는 모델이며 액추에이터가 이 정책을 실행한다는 보장은 없다.
3. 실제 코스 경계 입력과 GPS/차량 odometry 정합. 현재는 최초 GPS 정합을 고정하고
   후속 GPS와 위치 0.15m 또는 yaw 0.08rad 초과 불일치가 생기면 새 세션 전까지 HOLD한다.
   장거리 odometry 보정·지도 재정합은 추가 구현 대상이다.
4. 실제 제어기의 제동 실행 확인. 새 계획 실패 시 빈 경로/HOLD를 발행하고 MGM은 속도 0을 출력한다.
5. 벽 단면 연결과 평활화 후에도 HOLD하는 좁은 배치의 실행 가능성 개선. 현재 단면 연결은 전진 station 방향의 통로만 다룬다.
6. 실차의 센서 사각지대·자기 차체 반사·스캔 왜곡·측정 오차 한계 검증.
   광선이 지나간 격자 셀의 free 표시는 유한 빔 해상도의 근사다. 작은 장애물 검출 보장은 별도다.
7. 호스트 ingress→최종 CAN TX까지 50ms 계측과 실행 기한 보장.
   현재 `processing_ms`는 callback 시작부터 planner 반환까지이며 ROS 대기·출력 직렬화·CAN은 포함하지 않는다.

실시간 OS의 실행 보장이 없는 상태에서 20ms의 벽시계 확인은 작업을 정확히 그 시각에
중단한다는 뜻이 아니다. 시간이 초과된 출력을 무효화하는 동작이며, 엔드투엔드 지연 검증을 대체하지 않는다.

## 파일 구성

| 파일 | 역할 |
| --- | --- |
| [core.hpp](include/stack_avoid_v2/core.hpp), [core.cpp](src/core.cpp) | ROS 무의존 지도·차량 운동 탐색·완료 검사 |
| [node.cpp](src/node.cpp) | GPS/VehicleVector/원본 LiDAR 연결과 그림자 출력 |
| [shadow.launch.py](launch/shadow.launch.py) | 기존 센서 구독과 별도 watchdog 시작 |
| [shadow.yaml](config/shadow.yaml) | 초기 시뮬레이션용 모델·시간 예산 |
| [publish_course.py](tools/publish_course.py) | 명시적 코스 JSON 발행 |
| [watchdog.py](tools/watchdog.py) | 독립 프로세스의 계획 만료 진단 |
| [core_test.cpp](test/core_test.cpp), [scenarios.cpp](test/scenarios.cpp) | 결손 입력·기하·반복 계획·무작위 배치 검사 |
| [ros_smoke.py](test/ros_smoke.py) | 격리 ROS 입력·중단·복구 검사 |

추가 메시지는 [GpsRoute.msg](../fma_interfaces/msg/GpsRoute.msg),
[AvoidCourse.msg](../fma_interfaces/msg/AvoidCourse.msg), [AvoidPlan.msg](../fma_interfaces/msg/AvoidPlan.msg)다. 기존 메시지 필드 배치는 변경하지 않았다.

## 빌드와 시험

### 레퍼런스 패스 HTML 시각화

[시각화 열기](../../docs/avoid_v2_path_lab.html). 외부 라이브러리나 서버 없이
브라우저에서 열 수 있는 단일 HTML이다. 차량 2대·3대의 26개 시나리오를 최대 1m/s·10Hz 기록으로 재생하며
도로·점유 격자, 벽 단면·중간점·연결 안내선, 차량 운동 검사 구간 일부와 최종 경로·목표점을 표시한다.
목표점은 누적 경로 길이가 1m 이상이 되는 첫 표본이다(약 5cm 표본 간격).

브라우저의 별도 회피 알고리즘이 아니라 `tools/export_lab.cpp`가 실제 코어를 실행한
출력이다. 진단용 `PlanTrace`는 선택한 벽 쌍·중간점·안내선과 제한된 운동 검사 구간을 저장하고, `Grid::debug_snapshot`은
자유·점유·미관측·도로 밖 셀을 내보낸다. ROS 노드는 이 진단 출력을 요청하지 않는다.

현재 HTML은 **3.0m 1차로**, 자차·고정 차량 모두 **0.85m × 0.62m**, **0.2~1.0m/s·10Hz**다.
노란 경계 연결선·보라색 원래 중간점과 안내선·청록색 검증 경로를 구분한다.
26개 배치를 각 3회 반복한 일반 주행은 **54/60회 완주**, **6회 HOLD**다.
미관측·관측 만료·근접 차량 진단 18회는 예상 HOLD다.
[실패 수정과 현재 결과](../../docs/AVOID_V2_FIXES.md)를 참조한다.
HTML과 `avoid_v2_same_vehicle_scenarios`는 `test/same_vehicle_cases.hpp`의 같은 조건을 사용한다.
미해결 주행을 포함한 시험은 종료 코드 1을 반환하며 실패를 제외하지 않는다.

자유 공간을 명시적으로 제공하고 선택 경로의 앞부분을 이상적으로 실행한 기하 재생이다.
실제 센서 가림·1점 추종·HOLD 이후 제동 거동은 검증하지 않는다. zone 밖 재생은 기존 GPS
주행을 가정한다. 코어 시간은 전체 50ms 지연의 검증값이 아니다.

현재 코어로 HTML을 다시 생성하고 UI를 검사하려면:

```bash
python3 src/stack_avoid_v2/tools/build_lab.py
python3 src/stack_avoid_v2/tools/verify_lab.py
```

생성에는 `g++`, 브라우저 검사에는 Chrome 또는 Chromium이 필요하다. 검사기는
기록 일관성과 목표점 위치를 확인하고 시나리오 전환·재생·단계·레이어·확대·툴팁을
조작한다. 화면은 `/tmp/avoid_v2_browser_check`에 저장한다. 시간 예산이 있는 실제
탐색이므로 재생 기록은 생성 시 CPU 부하에 따라 달라질 수 있다.

### 코어와 ROS 시험

ROS 없는 코어 시험:

```bash
cmake -S src/stack_avoid_v2 -B /tmp/fma_avoid_v2_build \
  -DAVOID_V2_CORE_ONLY=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/fma_avoid_v2_build -j2
ctest --test-dir /tmp/fma_avoid_v2_build --output-on-failure
```

기존 실행 설치본과 분리한 ROS 빌드:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/fma_avoid_v2_logs build \
  --base-paths src/fma_interfaces src/stack_avoid_v2 src/stack_gps \
  --build-base /tmp/fma_avoid_v2_ros_build \
  --install-base /tmp/fma_avoid_v2_ros_install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source /tmp/fma_avoid_v2_ros_install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=186 ROS_LOG_DIR=/tmp/fma_avoid_v2_ros_logs \
  python3 src/stack_avoid_v2/test/ros_smoke.py \
  /tmp/fma_avoid_v2_ros_install/stack_avoid_v2/lib/stack_avoid_v2/avoid_v2_node
```

실제 센서 토픽의 그림자 관측은 `lidar_fusion_v2`가 설치된 환경에서 실행한다.
launch는 해당 패키지의 `fixed_geometry.yaml`을 직접 읽어 보정값의 복사본을 만들지 않는다.
회피 입력은 `config/shadow.yaml`의 `sensor_ids: [a1, b1, b2]`로 제한한다.
전방 a1·좌측 b1·우측 b2의 3개만 구독하며, 후방 a2는 입력이나 신선도 검사에 사용하지 않는다.
공유 fusion 설정과 다른 기능의 센서 드라이버 구성은 별도로 유지된다.
HTML은 합성 관측을 재생하므로 이 설정 변경으로 실제 3개 센서의 시야·가림 검증 결과가 되지는 않는다.

```bash
ros2 launch stack_avoid_v2 shadow.launch.py course_file:=/absolute/path/course.json
```

이 launch는 센서 드라이버나 차량 출발을 실행하지 않는다. 경계 입력이 없으면 HOLD한다.
`course_frame`은 기본 `map`이며 좌표는 GPS의 ENU 위치와 동일해야 한다.
JSON route_id는 `GpsRoute.route_id`와 같아야 한다. GPS route sequence에서는 설정 route ID,
단일 CSV 모드에서는 확장자를 포함한 CSV 파일명이다. 경계는 제공된 GPS 경로를 포함해야 한다.
새 메시지를 포함한 `stack_gps` 설치본을 실행해야 하며, route geometry가 없으면 HOLD한다.
RTK FIXED·실제 헤딩·신선한 VehicleVector·유효한 GPS zone 정보를 요구한다.
zone 안에서는 각 빔의 pose 이력과 설정된 모든 LiDAR를 추가로 요구한다.
스캔 `time_increment=0`, 프레임 불일치 또는 pose 보간 공백은 거부한다.

코스 JSON 형식 예시(직선 합성 fixture이며 실차용 코스가 아님):

```json
{
  "frame_id": "map",
  "route_id": "synthetic",
  "entry_station_m": 1.0,
  "exit_station_m": 6.0,
  "boundary": [[-2,-3], [12,-3], [12,3], [-2,3]]
}
```

GPS 경로에는 출구 이후 차체와 정지 경로를 검사할 길이가 필요하다.
zone 진입 위치는 인식 시작 후 제동·조향할 거리를 확보하도록 장애물보다 앞에 설정한다. 좌우 경계를 모르는 상태에서
위 예시 폭을 실차 경로에 적용하지 않는다. 모델/보정 파라미터 변경은 노드를 재시작해 적용한다.

## 기존 검증 기록 — 2026-09-15 zone 로직 변경 당시

Release 코어 시험 3개 실행 파일 통과. 단위 조건과 경로 표본 검사를 수행했다.
표본마다 늘어나는 assertion 개수를 독립된 주행 시나리오 수로 해석하지 않는다.

- 무작위 seed 20260915의 2대/3대 배치 8개 모두 연속 추월 후 GPS 복귀·출구 통과.
- 기존 S자 2대 반복 주행과 12개 무작위 초기 계획 시험도 통과.
- 장애물 없는 GPS 복귀에서 횡오차 <0.05m, 방향오차 <0.05rad, 조향 <0.03rad로 수렴 확인.
- zone 밖/zone 정보 결손/미관측/진행 공간 밖 차량/관측된 GPS 경로 차단을 구분하는 시험 통과.
- 격자/3원 검사와 별도의 사각형 SAT 검사기로 다중 차량 충돌 여부 확인.
- HTML은 12개 시나리오, 2,065 프레임을 포함하고 Chrome 1440px/390px 조작 검사 통과.
  이 기록의 코어 최대 약 18.2ms 이내; ROS·CAN 포함 50ms 전체 지연 보장은 아님.
- GPS 다점 발행의 단일 CSV/route sequence ENU·식별자 검사 통과. 기존 GPS 시험 169개 통과.
- 격리 ROS에서 GPS 경로 결손, zone 밖 LiDAR 비활성, zone 정보 결손, 진입 후 새 관측 대기,
  정상 GPS 경로 출력, 센서 오류·관측 중단/복구와 프로세스 정지 시 watchdog 시험 통과.

이 절은 과거 그림자 실행 검증 기록이다. 현재 MGM 연결은 위 운영 계약을 따르며, 실측 제어 응답과
실차 센서 시야 검증은 남아 있다.

## 과거 검증 — S자 좌우 배치 (0.6m/s)

[상세 보고서](../../docs/AVOID_V2_S_LATERAL_TEST.md). GPS 진행 방향의 법선 기준으로
좌/우를 정의한 5개 배치를 3회씩 시험했다. 좌→우·우→좌·좌→우→좌·우→좌→우는
12/12 완주, 차량 충돌과 도로 이탈 없음. 좌우 동시/중앙 통로는 0/3 완주로 HOLD했다.
회피 코어 변경 없이 실패를 포착하는 회귀 시험을 추가했으며,
당시 `avoid_v2_s_lateral_scenarios`는 이 미해결 조건 때문에 실패했다.
현재 수정 후에는 이 시험이 통과하며, 남은 실패는 위 동일 차량 시나리오 시험에 기록한다.
당시 HTML은 17개 시나리오였고 1440px/390px 브라우저 조작 검사를 통과했다.
현재 HTML은 위 1m/s·동일 차량·좌우 연석 조건의 26개 시나리오로 대체했다.
