# 회피 시작 표식과 웨이포인트 복귀 — 2026-09-14

한라 최신 PR #98(`8796cdd`)의 `state=4`는 Path 4 idx 55에 있다.
YAML `avoid_zones`는 비어 있고, Path 3의 Zone 4는 주차 접근 구간이다.
사용자 지정에 따라 CSV의 `state=4`를 실제 회피 시작 신호로 해석한다.

## 시작과 종료

`avoidance_enabled=true`, `avoid_zone_only=true`인 base manager 동작이다.

1. 표식 이전: 일반 LINE/GPS 주행. 장애물 검출·Nav 상실만으로 일반 회피에 진입하지 않는다.
2. 표식 통과: GPS의 `avoid_zone=true`가 독립 GPS generation 5회 확인되면
   `AVOID_ACTIVE`. 장애물이나 유효 회피 목표점이 먼저 있어야 하는 조건은 없다.
3. 장애물 관측 전: 회피 상태에서 기존 회피 노드의 GPS station+1m 참조를 사용한다.
4. 장애물 관측 후: 기존 `A → G → B` 3차 Hermite 회피 경로를 사용한다.
5. 플래너가 B의 station 0.25m 이내, GPS 경로 0.25m 이내, 헤딩 20° 이내,
   다음 GPS 연결 검사까지 통과하여 `maneuver_done`을 발행하면 `GPS_RETURN`.
6. 현재 GPS 횡오차 0.1m 이하·헤딩 오차 20° 이하와 유효한 실제 heading을
   확인하면 `INACTIVE`로 전환하고 일반 주행을 다시 선택한다.

장애물을 아직 관측하지 않은 상태의 오래된 done은 완료로 인정하지 않는다.
무검출·타이머 만료·지리적 구간 이탈·신호 해제는 회피 종료 조건이 아니다.
주차 미션·FINISH·명시적 disable/session reset의 기존 우선권은 유지한다.
기존 경로 완료 정책으로 새 CSV를 인계받으면 이전 경로의 회피 기억도 초기화한다.

## GPS 시작 신호

CSV의 quality 필터·연속 중복 좌표 제거를 거친 실제 경로 인덱스에 표식을 매핑한다.
state 1/2/3은 회피 시작 신호로 해석하지 않는다. 여러 CSV를 순서대로 주행할 때
각 CSV 자체의 표식을 읽으므로 Path 4 표식이 다른 경로에 적용되지 않는다.

한 점을 정확히 샘플링해야 하는 조건으로 만들면 주행 중 표식을 놓칠 수 있다.
따라서 Path 4의 `avoid_zone`은 idx 55부터 CSV 끝(idx 191)까지 true로 유지한다.
이 구간은 시작 지점 통과 표시이며, **회피 종료 위치를 CSV 끝으로 정한 것이 아니다.**
표식에는 기존 YAML 회피 구간의 5m 선행 확장을 적용하지 않는다.

MGM은 같은 GPS generation을 여러 번 수신해도 진입 표본을 한 번만 센다.
GPS 상실/오래된 관측은 진입을 만들지 않고 기존 상태를 보존한다.
복귀가 완료된 표식은 소모한 것으로 기억해, 신호가 계속 true여도 다시 진입하지 않는다.
확인된 구간 이탈 후 새 진입 또는 경로 전환/세션 reset으로 다음 시작을 허용한다.
현재 계약은 CSV당 state=4 시작점 하나다. 여러 개가 있으면 두 번째를 조용히
무시하지 않고 경로 로딩 오류로 알린다. 필요하면 경로 CSV를 분리한다.

기존 명시적 YAML/위경도 회피 구간도 시작 신호로 사용할 수 있다.
`avoid_zone_only=false`를 명시하면 기존 장애물 검출 기반 정책을 선택한다.
기존 generated/legacy backend의 게이트 의미는 그대로이며 운영 v2는 base manager를 사용한다.

## 출력과 정지

Zone 정책 런처는 회피 노드에 `avoid.require_mgm_active=true`도 전달한다.
회피 노드는 장애물 인지를 계속 발행하되 MGM의 신선한 `AVOID_ACTIVE` 상태가
있을 때만 회피 참조를 만든다. 진입할 때 이전 목표·완료 상태를 초기화하여
구간 밖에서 본 장애물의 과거 경로를 이어받지 않는다. 상태 피드백이 오래되면
참조를 비우고, GPS_RETURN/비활성 전환을 받으면 회피 경로를 초기화한다.

진입 후 빈/오래된 회피 참조를 받으면 회피 상태를 유지하고
`reference_motion_blocked=true`, 속도 0으로 새 참조를 기다린다.
기존 독립 LiDAR E-stop과 후진의 Safety 권한은 유지한다. 회피 구간 밖에서
독립 E-stop 후진이 시작됐다는 이유로 일반 AVOID를 활성화하지 않는다.
주차 상태에서는 기존 미션 우선권이 적용된다.

외부 참조는 기존 한 점이며 회피 곡선 생성식·검출 거리·차체 충돌 검사·속도 설정은 같다.
두 장애물 연속 연결은 별도 설계안이며 이번 변경에 구현하지 않았다.

## 확인

- `test_avoidance_marker.py`: 최신 Path 4 표식, 필터/중복 처리, 구간 시작과 유지.
- `avoid_zone_entry_test.cpp`: 무장애물 진입, GPS generation 확인, 조기 종료 차단,
  실제 기동 후 복귀, 완료 표식 재진입 차단, 주차/disable/독립 후진 우선권.
- `test_v2_launch.py`: 기본 Zone 정책이 MGM 파라미터까지 전달되는지 확인.

실행 순서는 저장소 루트의 `RUN_BOOK_FINAL.md`를 따른다.

검증 결과: MGM 빌드와 CTest 26/26 통과, GPS·런처·주행 준비 Python 검사 통과,
회피 노드·활성화 피드백·경로별 표식 전달 Python 검사 통과.
`gps_cubic_avoid_ros_smoke.py --zone-entry`는 격리 ROS_DOMAIN_ID=193에서 실제
회피/MGM 노드의 무장애물 진입, 목표점 전달, GPS 창 소실/복구, 실제 곡선 복귀,
동일 표식 재진입 차단까지 8단계를 통과했다. 하드웨어·CAN·실차 주행은 실행하지 않았다.
