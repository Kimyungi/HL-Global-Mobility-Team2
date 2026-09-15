# 벽 중앙 회피를 v2 기본 provider로 연결

2026-09-15. `integration/v2_main` 통합용 변경.

## 실행 경로

통합 vehicle/drive/no-estop launch는 기본 `avoid_v2_enabled=true`로 `stack_avoid_v2/avoid_v2_node`를 실행한다. 기존 `stack_avoid_node`는 이 모드에서 실행하지 않는다. 이전 provider의 비교 실행은 `avoid_v2_enabled=false`를 명시한다. 새 provider는 기존 base MGM과 `avoid_zone_only=true`, 보정된 multi-LiDAR bringup을 요구한다.

전방 a1·좌측 b1·우측 b2만 회피 입력으로 사용한다. MGM의 회피 입력 신선도 역시 이 3개 raw scan을 요구한다. a2의 공유 드라이버와 주차 사용은 유지한다. 독립 LiDAR E-stop, 주차·신호·운전자·CAN 정지와 Safety 후진의 우선권은 바꾸지 않는다. 후진 중 새 전진 경로는 발행하지 않으며 pose 이력은 유지한다.

`control_enabled=true`인 플래너가 신선한 MGM `AUTONOMOUS_DRIVE/AVOID_ACTIVE` 피드백을 받을 때만 새 회피 에피소드를 계산한다. 미션 활성/비활성 전환 시 기존 경로·완료 증거를 초기화한다. `shadow.launch.py`는 계속 `control_enabled=false`이며 MGM은 그림자 계획을 명령으로 소비하지 않는다.

## 유지한 zone 상태 전환

`core/manager_step.cpp`의 상태 전환 코드와 GPS 표식/membership 계산은 변경하지 않는다.

- CSV state=4 또는 기존 회피 zone 신호의 독립 GPS generation 5회 확인 → `AVOID_ACTIVE`.
- 장애물 검출·경로 유효 여부는 진입의 선행 조건이 아니다.
- 빈 경로·유효기간 만료·센서 상실은 회피 소유권을 유지한 속도 0/HOLD다.
- zone 이탈·무검출·타이머·신호 해제만으로 회피를 종료하지 않는다.
- 장애물 관측 이력이 있고 플래너가 복귀 완료를 발행하면 기존 `GPS_RETURN`으로 진입한다. 현재 GPS 횡오차 0.1m 이하·헤딩 20° 이하 및 실제 heading 유효성을 확인한 뒤 비활성으로 복귀한다.
- 완료한 표식의 재진입 방지, 새 경로/세션 reset, 주차/FINISH/disable 우선권을 유지한다.

새 플래너의 `maneuver_done` 증거는 유효한 벽 경로, 전방 GPS 통로의 점유 해소, 현재 GPS 횡오차 0.1m·헤딩 20° 이내, 최소 1m 또는 제동거리만큼의 GPS 연결 차체 영역이 관측상 비어 있음을 3개 새 계획에서 확인하는 것이다. 고정 차량의 개수나 CSV 종점을 완료 조건으로 쓰지 않는다. 코어 HTML용 지리적 출구 완료와 운영 MGM의 복귀 완료를 구분한다.

## 지도와 출력 계약

`/perception/gps_route`의 실제 ENU 경로를 사용한다. 명시적인 `/avoid_v2/course`가 있으면 그 경계를 추가 통과 금지 영역으로 적용한다. 운영 모드에서 코스 파일이 없으면 현재 GPS station 기준 뒤 5m·앞 30m의 경로를 선택하고, 그 경로의 좌표 범위에 12m 여유를 둔 **계산용 직사각형**을 만든다. 지도 끝에 가까워지면 새 관측을 요구하며 창을 갱신한다. 이 영역은 도로/연석 위치나 자유 공간을 뜻하지 않는다. 셀은 unknown에서 시작하며 raw LiDAR 광선의 실제 관측으로만 자유/점유를 갱신한다. 용량·GPS·스캔·정합 검증 실패는 HOLD다.

MGM은 `/perception/avoid` 대신 `/avoid_v2/plan`을 직접 구독한다. `AvoidPlan`의 control 활성, route 식별, 입력 시각, 단일 vehicle-frame 목표점, 유효기간, 처리 기한과 속도의 유한성을 매 10ms 제어 틱에서 확인한다. ROS 시각과 단조 증가 수신 시각을 함께 확인하므로 플래너가 멈추거나 ROS 시각이 멈춰도 마지막 유효 목표가 무기한 남지 않는다. 기존 `/perception/avoid` 발행은 새 모드의 권한을 덮어쓰지 못한다.

유효 계획은 계산한 100ms 뒤 요청 속도를 MGM에 전달한다. 기존 신호·안전 정지와 회피 상한은 우선 적용된다. 회피 진입/복귀 시 블렌딩은 생략하여 검사한 목표점의 xy/yaw/곡률을 유지한다. 다른 LINE/GPS 전환과 legacy provider의 블렌딩은 유지한다. CoreParams 변경으로 raw dump 버전은 31이다.

## 검증과 남은 한계

- MGM의 기존 zone 전환 시험을 유지하고 새 입력 계약·무블렌딩·속도 전달 시험을 추가한다.
- 복귀 증거 시험은 장애물 미관측, 정렬 오차, 점유/미관측 연결, 실패 계획과 새 세션이 잘못된 완료를 만들지 않는지 검사한다.
- `test/mgm_integration.py`는 실제 C++ 플래너/MGM과 합성 3개 scan을 사용한다. zone 진입, 목표점·속도 전달, 프로세스 SIGSTOP 만료, zone 이탈 소유권 유지와 센서 복구를 검사한다. 센서 드라이버/CAN/go는 실행하지 않는다. 이 시험의 센서 FOV는 합성이며 실차의 3개 FOV를 재현한 시험은 아니다.
- HTML은 기존 합성 관측 코어 기록이다. 실제 장착 시야, 차체 주변/후방 미관측 공간과 연석 미검출은 검증되지 않았으며 이 때문에 시작/주행 중 HOLD할 수 있다. 미관측을 자유 공간으로 채워 이 한계를 숨기지 않는다.
- 기존 직선 중앙 2대/3대 배치의 HOLD는 남아 있다. MGM 연결이 모든 배치의 완주를 의미하지 않는다.
- 코어 20ms 및 ROS callback 35ms 초과 출력 무효화는 유지하지만, 센서 ingress→CAN→액추에이터 전체 50ms 지연과 실제 제동/조향 응답은 실차 계측이 남아 있다.

빌드: `scripts/v2 build`. 통합 시험: `ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=192 python3 src/stack_avoid_v2/test/mgm_integration.py install_v2` (먼저 v2 환경 source). 코어 전체 시험: `ctest --test-dir build_v2/stack_avoid_v2 --output-on-failure`. 남은 동일 차량 실패를 제외하지 않는다.

## 이번 검증 결과

운영 `install_v2`에 fma_interfaces·stack_gps·stack_avoid_v2·adas_mgm 빌드를 완료했다.
MGM CTest 27/27, GPS 180개, 런처/준비/전방 보정 38개, 시각화 6개 검사가 통과했다.
새 플래너/MGM의 격리 ROS 연결은 11개 단계, 기존 그림자 ROS smoke도 통과했다.
회피 코어는 9개 중 8개 통과했다. 동일 차량 일반 주행 54/60 완주, 기존 직선 중앙
2대/3대 배치의 6회 HOLD가 남아 해당 CTest는 실패 상태다. 진단 18회는 예상 HOLD다.
원시 기록은 [통합 검사 로그](avoid_v2_mgm_checks.txt)를 참조한다.
