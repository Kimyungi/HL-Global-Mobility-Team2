# Integration v2 — 경로 선택·순서·전환 계약

> 2026-09-13 주차 종료 기준은 [즉시 주차 진입](MGM_PARKING_ENTRY.md)이 우선한다.
> Zone 진입 즉시 PARKING, 정상 종료는 done 또는 현재 CSV 종점이다. 미완료 종점은
> ROUTE_END=10 실패로 기록하며 기존 실제 정지/새 CSV 응답을 확인한 뒤 인계한다. 현재 dump는 v19다.

2026-09-12. 사용자 정정 순서는 **(01 또는 02)→03→04→05→(06 또는 07)**이다.
이번 지정 run은 **01→03→04→05→07**이다. 01→02 및 06→07은 주행하지 않는다.
현재 raw dump는 v15이며 단일 `waypoint_csv` 운용은 기존 동작이다.

## 실행 전 경로 선택

정본은 `src/stack_gps/waypoints/halla_route_sequence.yaml`이다. `routes`는 사용 가능한 CSV/Zone
목록이고 `sequence.start/via/end`가 분기와 공통 순서를 정의한다.
`route_start_id`(01/02), `route_end_id`(06/07)를 실행 전에 모두 명시해야 한다.
기본값은 빈 문자열이며 누락·허용되지 않은 ID·다른 CSV/Zone override는 launch 전에 거부한다.
한 번 선택하면 5개 CSV와 대응 Zone을 선로딩하고 run 중 변경하지 않는다.
선택한 순서도 manifest/CSV/Zone 내용 hash에 포함하므로 다른 선택은 다른 sequence_id다.

```bash
# 하드웨어 없는 파일 검사. 해당 터미널에 v2 Python 패키지가 있어야 한다.
python3 -m stack_gps.route_plan src/stack_gps/waypoints/halla_route_sequence.yaml --start 01 --end 07
```

차량 launch에는 `route_sequence_file:=<manifest 경로> route_start_id:=01 route_end_id:=07`을 전달한다.
같은 파일을 쓰더라도 출발점이 02이면 start만 02로, 복귀를 06으로 정하면 end만 06으로 바꾼다.
자동 판별 결과는 이번 run의 선택을 바꾸지 않는다. 선택을 바꾸려면 재기동한다.

확인한 `src/stack_exit_decision/models/README.md`와 원격 모델 브랜치 `12e7ba2`에는
`red_blue_red`/`blue_red_red` 모델이 있으며 runtime integration은 제외돼 있다.
현재 v2/main에서 이 결과를 06/07 CSV 선택으로 연결한 실행 코드는 확인되지 않았다.
임의 class→경로 매핑을 넣지 않고 명시적 선택만 사용한다. 자동 분기 연결은 후속 작업이다.

## 업로드 경로와 Mission

손상민 PR #86의 최신 `28ba409e4db29d0f212449b80d2e9cf935276ac5`까지 반영했다.
CSV/Zone 원본 기하를 사용하며 한라대 manifest에는 `entry_connection: straight`가 없다.

| 전환 | 기준 |
|---|---|
| 선택한 01/02→03 | 두 출발 경로의 끝점이 03 시작점과 일치 |
| 03→04 | 끝점 일치, 03 T자 Mission 성공 또는 Zone 이탈 실패 필요 |
| 04→05 | 04 평행 주차 성공 또는 Zone 이탈 실패 필요. CSV 끝점은 의도적으로 불일치 |
| 05→선택한 06/07 | 05 끝점이 두 종료 경로의 시작점과 일치 |
| 선택한 06/07 종료 | Top FINISH |

최신 04는 `(-62.335,-72.648)`에서 끝나고 05는 주차 후 복귀점 `(-64.335,-70.648)`에서
시작한다. 해당 불연속은 업로드 문서가 명시한 Parking 인계다. 04 종점을 ACTIVE 중 관측하면
기억하고, Parking이 복귀 위치로 이동한 뒤 done을 보낼 때 옛 종점을 다시 찾지 않는다.
실제 Parking이 이 조건을 만족하는지와 복귀점에서의 GPS 인계는 현장 검증 대상이다.
새 직선 연결로 Parking 이동을 대신하지 않는다.

03 GPS-only Zone 3은 idx 28~116, T자 주차는 03/145, 평행 주차는 04/163이다.
05/300의 state=3은 신호 예상 위치이며 카메라/MGM Signal 판정을 강제하지 않는다.
업로드에는 Zone 경계/주차점이 있고 `parking_zone_span_m=1.0`은 점을 구간으로 만드는 폭이다.
Zone 확인 표본 수와 Parking 탐색 시간/거리 제한값은 포함돼 있지 않다.

## MGM 전환과 제어권

1. DRIVE에서 fresh 유효 GPS 비종점 표본을 본 뒤 기존 PathEngine.at_end를 만나면 종점을 기억한다.
   ACTIVE Parking에서도 종점은 기억하되 Parking을 중단하지 않는다.
2. `endpoint_and_missions`는 종점 + 해당 경로 모든 Mission 성공 또는 Zone 이탈 실패 + 현재 요청 해제를 요구한다.
   `missions_complete`는 모든 Mission 완료가 종점 조건을 대체하며 Mission이 하나 이상이어야 한다.
   Zone 이탈 실패는 성공과 별도 종료로 인정한다. 그 외 취소·미진입은 완료가 아니다. 한라대는 endpoint_and_missions를 사용한다.
3. 실제 차속이 fresh finite이고 기존 정지 허용치 `abs(v)<=1e-3`이며 외부/신호/안전 정지 등이
   해제되면 다음 경로를 요청한다. 중간 종점은 전체 FINISH가 아니다.
4. GPS 적용 응답과 요청 전보다 새로운 유효 GNSS reference를 확인한 뒤 기존 go 인가로 자동 재개한다.
   인계 틱/응답 대기는 목표속도와 ramp 0이다. 이전 경로 geometry와 새 경로를 혼합하지 않는다.
5. 마지막으로 선택한 06 또는 07 본경로의 종료에만 FINISH를 latch한다.

경로별 Zone/Mission/지정 정지 ID를 sequence 전체에서 유일하게 매핑한다. CSV 변경은 새 Mission
session이 아니며 완료 기억은 유지한다. 일반 Navigation/Avoidance/Signal/Safety와 Reference gate도 유지한다.
미보정 Zone/Parking 입력을 우회하지 않는다. 자동 전환은 무정차 통과를 보장하지 않는다.

## 명령·진단·session

manifest/선택/파일 내용 hash, GPS instance, 요청 번호, index 및 단계가 모두 일치해야 인계를 승인한다.
같은 요청 재전송은 멱등이며 GPS 노드는 완료 판단 없이 MGM 명령을 적용한다.
설정·producer 재시작·예상 밖 index/단계·시계 역행은 FAULT다. 자동 skip이나 응답 대기 timeout은 없다.
명시적 새 session은 완료 기억을 지우고 **선택한 시작 경로**를 다시 요청한다. 당시 raw 내부 Mission Zone은
확인된 exit/reentry 전까지 억제한다. go/센서 복구는 sequence를 초기화하지 않는다.

RoutePhase: DISABLED=0/RUNNING=1/WAIT_MISSION=2/WAIT_STOP=3/WAIT_ACK=4/FINISHED=5/FAULT=6.
SAFE_STOP_ROUTE_SEQUENCE bit 512는 매 틱 계산한다. FAULT 자체는 명시적 새 session이 필요하다.
한라대 `index`는 선택된 5개 경로의 **0~4 순번**이다. 실제 CSV 번호는 GpsPath.route.route_id를 본다.
01→03→04→05→07이면 index 0/1/2/3/4에 route_id 01/03/04/05/07이 대응한다.

위치/TF/pose delta는 선택한 첫 경로의 ENU 원점을 run 내내 유지한다. track_index는 현재 CSV 내 순번이다.
일반 고정 manifest의 선택적 직선 연결 기능과 connecting/requested_connecting bus는 유지하지만 한라대에서는
항상 false다. Zone 탐색 실패 기억/정책 bus 추가로 v15다. v14 이하 dump는 해당 버전 빌드로 재생한다.
MBD에는 선택 후의 순서·동일 hash와 RouteControl 입력을 공급한다. generated v1.88은 지원하지 않는다.

현재 파일은 `/perception/gps_path.route`로 확인한다. waypoint_csv 파라미터는 시작 설정이므로
mark_zone에는 실제 본경로 CSV/Zone을 --track/--out으로 명시한다.

## 검증과 실행

[한라대 런북](../src/adas_mgm/RUNBOOK_integration_v2_halla.md),
[용인 단일 Course A 런북](../src/adas_mgm/RUNBOOK_integration_v2_yongin.md)을 사용한다.
`./scripts/v2 build`와 `./scripts/v2 test`는 격리 build_v2/install_v2를 사용한다.
ROS 시험은 실제 GPS wrapper에 합성 fix와 Parking 응답을 공급하고 MGM만 실행한다.
4개 시작/종료 조합, Mission 완료, 같은 위치에서 CSV 인계, 04→05 ACTIVE Parking 중 이동과
종점 기억, 마지막 FINISH, raw replay를 검사한다. Parking 이동은 모의 입력이며 실제 알고리즘 수행 증거가 아니다.
2026-09-12 Zone 탐색 정책까지 반영한 결과: 13개 패키지 빌드, CTest 17/17, Python 382 passed / 3 skipped,
기존 MGM ROS 32 PASS. 시작/종료 4개 조합 각각에서 5개 CSV·Mission 2개·인계 4회·최종 FINISH와
v15 재생을 통과했다. 04→05는 ACTIVE 동안 종점 관측→복귀점 이동→done 순서의 모의 입력을 검증했다.
추가로 01→03→04→05→07의 두 주차 탐색을 Zone 이탈로 실패시켰다. 실패 뒤 현재 CSV를 계속 주행하고
종점에서 다음 CSV를 요청하며 마지막 FINISH까지 도달하는 모의 흐름과 v15 replay를 통과했다.
두 런북의 Bash 블록 37개 문법을 확인했다. 기존 SciPy/NumPy 경고 1건이 있다.
기존 차선 모델을 v2 모델 폴더에 준비했으나 센서·CAN bridge·차량은 실행하지 않았다.
운영 Zone 확인 수는 **5/5**다. 주차는 이후 사용자 지정 [Zone 탐색 정책](MGM_ZONE_SEARCH.md)을 적용하며 시간·거리 제한을 쓰지 않는다. 아직 commit/push하지 않았다.
