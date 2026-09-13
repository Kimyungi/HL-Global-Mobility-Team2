# Integration v2 — Zone 안에서만 주차 탐색

> 역사적 정책: 현재 v2 기본은 [즉시 주차 진입](MGM_PARKING_ENTRY.md)이다.
> 아래는 `parking_zone_entry_active=false && parking_search_zone_only=true`에서만 적용한다.

2026-09-12 사용자 변경 기준. 현재 `parking_search_zone_only=true`이며 진입/이탈 확인은 5/5다.
이 문서가 이전 시간·거리 제한과 Zone 밖 준비 완료에 관한 5·6차 설명보다 우선한다.

## 시작·성공·실패

- 미완료·미실패 Mission의 stable Zone entry에서 PREPARE를 시작한다. 실제 GNSS generation 5회로 진입을 확인한다.
- 요청의 `source_zone_id`로 탐색 범위를 고정한다. 겹친 다른 Zone이나 대표 표시 Zone의 이탈은 취소 조건이 아니다.
- source Zone의 유효한 stable 내부 문맥과 기존 request ID/mode·fresh 준비 응답·localization readiness를
  모두 만족하면 ACTIVE다. 주차 실행은 ACTIVATE 이후에만 진행한다.
- PREPARE에서 source Zone의 이탈이 5회로 확정되면 `ZONE_EXIT=9`로 CANCEL하고 실패를 기록한다.
  같은 틱의 준비 완료보다 이탈 실패가 우선한다. 늦은 ready/ref/done은 종료된 요청을 재활성화하지 않는다.
- 실패 뒤 공통 nav_reselect로 **현재 CSV의 다음 점부터 일반 주행을 계속한다**. 실패 순간 다음 CSV로 건너뛰지 않는다.
- ACTIVE로 인계한 뒤에는 기존 Parking 제어권·done 조건을 유지한다. 이 탐색 이탈 규칙은 ACTIVE를 중단하지 않는다.

시간/이동거리 제한은 없다. `parking_search_timeout`·`max_parking_search_distance`의 -1은 이 모드에서
사용하지 않는 과거 설정이다. `parking_calibration_state=NOT_REQUIRED(3)`으로 표시하며 양수 설정을 요구하지 않는다.
elapsed_s와 실제 속도 기반 travel_distance는 관측 기록만 유지한다. 외부 stop 중에도 경과 시간은 기록된다.

GPS loss/invalid snapshot/과거 generation은 확정 이탈이 아니다. PREPARE 요청을 유지하고
`SAFE_STOP_MISSION_ZONE_UNKNOWN=1024`로 정지하며 준비 완료를 승인하지 않는다. 유효 문맥 회복 뒤
현재 source Zone 내부면 탐색을 계속하고, 밖이면 정상 5회 이탈 확인 후 실패 처리한다.
기존 실제 속도 freshness/finite 상실·monotonic 역행은 MOTION_UNAVAILABLE 취소다.
명시 취소·모듈 중단·FINISH·새 session·외부/CAN 정지도 기존 정책을 유지한다.

stable Zone을 쓰므로 최초 raw 외부 관측에서 바로 실패하는 것은 아니다. 현재 5회 확인 지연을 포함한다.
Zone 크기/주차점 위치는 원본을 유지하며 새로운 거리 폭이나 임의 확장을 넣지 않았다.

## 실패 기억과 CSV 전환

`mission_failed[256]`은 Mission ID별 실패 기억이며 `mission_completed[256]` 성공 기억과 별개다.
이번 실패 원인은 ZONE_EXIT뿐이다. operator cancel 등 다른 취소를 성공·실패 완료로 바꾸지 않는다.
같은 session에서 실패 ID는 재진입/다른 공유 Zone으로도 재시도하지 않는다. 명시적 start_session에서만 초기화한다.

현재 한라대 `endpoint_and_missions`의 Mission 조건은 **성공 완료 또는 Zone 이탈 실패**로 종료된 모든
필수 Mission을 요구한다. 현재 CSV의 종점까지 계속 주행하고 실제 정지·기존 안전 조건·새 GPS 응답을
확인한 뒤 다음 CSV로 넘어간다. 따라서 실패한 주차의 done을 종점에서 무한히 기다리지 않는다.
`missions_complete`는 종료된 Mission 자체가 경계인 별도 manifest 정책이며 한라대에는 사용하지 않는다.

선택한 순서는 01→03→04→05→07이다. 04 탐색 실패 뒤에는 실제 04 끝점 위치에서 05 reference를
새로 생성한다. 05의 시작점은 업로드된 주차 후 복귀점으로 04 끝점과 약 2.8m 다르다.
자동 좌표 이동이나 새로운 직선 segment는 만들지 않는다. 그 위치에서 기존 PathEngine으로 복귀하는
기하와 차량 추종은 실차 확인 대상이다.

## 메시지·분석·재현

MgmState에 `mission_failed`, `parking_search_zone_only`, `CANCEL_ZONE_EXIT=9`,
`NOT_REQUIRED=3`, `SAFE_STOP_MISSION_ZONE_UNKNOWN=1024`를 추가했다.
현재 요청의 실패와 reason/ID는 다음 요청 전까지 보존한다. 성공 완료 bool은 false다.
mission_events.csv는 기존 cancel 이벤트 + reason 9를 사용한다. 분석 도구는 `zone_exit_failure`로 집계한다.
core_replay CSV에도 실패·reason·정책을 기록한다. CoreParams/State/Output bus 변경에 따라 raw dump는 **v15**다.
구 dump는 해당 버전 빌드로 재생한다.

`parking_search_zone_only`는 startup-only다. false는 과거 시간·거리 제한 시험/재현용이며
v2 기본과 이번 런북은 true다. true에서는 사용하지 않는 두 시간·거리 parameter의 런타임 변경을 거부한다.
기존 detector/planner/SLAM/localization은 바꾸지 않고 기존 typed CANCEL을 재사용한다.

검증은 `./scripts/v2 build`, `./scripts/v2 test`로 수행한다. 새 core 검사는 5회 경계, 동시 ready,
긴 시간·거리 무제한, source/overlap, GPS 불명, ACTIVE 유지, 실패 재시도 억제/session reset,
실패 후 현재 CSV 유지와 종점 인계를 검사한다. ROS 모의 시험은 01→03→04→05→07에서 두 주차를
실패시켜 CANCEL 발행·늦은 응답 거부·현재 CSV 진행·종점 인계·v15 replay를 확인한다.
2026-09-12 실행 결과: 13개 패키지 빌드 성공, CTest **17/17**, 새 Zone 탐색 core **31 checks**,
Python **382 passed / 3 skipped**다. 기존 MGM ROS 32 PASS와 시작/종료 네 조합의 성공 흐름,
01→03→04→05→07의 두 주차 실패 흐름 및 v15 replay를 통과했다.
기존 SciPy/NumPy 경고 1건이 있다. 실차/센서/CAN bridge는 이 검증에서 실행하지 않았다.
