# 차선 판단 zone 게이트 및 zone 판정 점검 — 2026-09-18

차선 카메라는 상시 유지하고 `/perception/lane_camera`, RViz 영상은 계속 갱신한다.
통합 프로파일은 `zone_gated=true`로 GPS 전용·신호등·주차·회피 구간에서 차선 추론과
`LanePath` 발행을 중단한다. 주차·회피가 진입 zone 밖까지 이어져도 활성 상태 동안 중단한다.
일반 구간 복귀 시 보관 경로·계수·추적 이력을 초기화하고 새 프레임으로 판단한다.

## 현재 zone 판정 (2026-09-18 후속 요청 반영)

- station은 실제 차량 좌표를 CSV에 투영한 진행 위치다. zone은 투영 위치의 최근접 CSV 인덱스로 판단한다.
- GPS 전용/신호등 zone: station 또는 GPS preview 인덱스가 범위에 있으면 진입 관측.
- 주차/마지막 미션: station만 사용. preview만 들어간 것으로는 미션 진입하지 않는다.
- station이 특별 zone에 속하면 다른 preview-only GPS zone 진입을 억제한다.
  stop/accel 및 좌표로 지정한 avoid의 station 범위도 이 우선권에 포함된다.
  CSV state=4는 시작 인덱스만 우선권을 갖고 이후의 유지 표시는 preview를 차단하지 않는다.
- station 자체가 둘 이상의 zone 범위에 속하면 `zone_id`가 작은 zone 하나만 활성화한다.
  station zone이 없고 preview GPS zone이 여러 개 겹쳐도 작은 ID 하나를 선택한다.
  MGM은 진입·이탈 확인 중 잠시 공존하는 확정 zone에서도 작은 ID를 표시한다.
  이미 실행 중인 주차·회피 미션의 제어권은 기존 완료 조건까지 유지한다.
- MGM 진입·이탈은 각각 새 GPS 관측 5회 확인한다. 원시 membership 변경과 확정 표시가 같은 틱은 아니다.
- `RoutePlan.bind()`는 명시한 zone ID를 유지하고 자동 생성한 미션 ID만 빈 번호로 배정한다.

근거: `stack_gps/zones.py:ZoneMap.snapshot`, `stack_gps/path_engine.py:_station_snapshot`,
`stack_gps/node.py:_fill_zone_context`, `stack_gps/route_plan.py:RoutePlan.bind`,
`adas_mgm/core/zone_step.cpp`.

## 최근 로그 확인

`drive_logs/v2_20260918_181319_639424` 기록에서 YAML의 신호등 zone [3]은 실행 ID [2]다.
zone [2]의 원시 진입 연속 구간은 827개/170개 관측이며 각각 5번째 관측부터 확정됐다.
주차 zone [1]의 원시 진입 구간도 5개/10개/26개 관측에서 각각 확정됐다.
검사한 구간에서는 원시 진입이 계속되는데 확정이 전혀 안 되는 현상은 확인되지 않았다.
station/preview와 GPS 투영 위치가 맞지 않는 경우는 이 확인만으로 배제할 수 없다.

## 검증

모의 카메라에서 추론 OFF 중 최신 영상·heartbeat 유지, 특별 zone/주차/회피 차단,
일반 구간 복귀 및 추적 초기화를 검증한다. 기존 GPS station/preview 우선순위 시험과
상위 zone manager 시험도 수행한다. 실차 주행을 재시작하거나 실제 카메라를 새로 열지 않는다.

## CSV zone [4] 주차 접근 감속

원본 `halla_0919.csv` 수정 내용을 분할 CSV에 반영했다. 경로 03의 zone [4]는
idx 117~145이며, GPS 로더가 실제 CSV의 `zone_id`를 읽어 속도 제한 범위를 구성한다.
통신용으로 다시 매겨진 ZoneContext ID를 물리 zone [4]와 혼동하지 않는다.
기존 `GpsPath.accel_zone` 속도 범위 필드와 `v_accel_zone=0.5`를 사용한다.
station이 다른 특별 zone에 없으면 preview 진입부터 제한하며, station이 범위에
남아 있으면 preview가 나가도 최대 0.5m/s를 유지한다. 이탈하면 세션 속도로 복귀한다.
주차·회피 실행기가 제어하는 속도와 정지 명령에는 이 항법 속도 제한을 덮어쓰지 않는다.

## 모든 활성 zone의 웨이포인트 강제

CSV의 모든 양수 `zone_id` 범위를 항법에 연결한다. station 또는 허용된 preview가
범위에 들어가면 웨이포인트 전용이며, 상위에서 확정된 특별 zone도 종류·ID와 무관하게
카메라 항법을 금지한다. GPS 무효/FLOAT에서는 카메라로 대체하지 않고 정지한다.
차선 카메라의 영상·heartbeat는 계속 유지한다. 신호등 추론 활성화는 기존 신호등
zone 문맥만 사용하므로 다른 CSV zone 때문에 신호등 판단이 켜지지 않는다.
zone [4]는 이 웨이포인트 강제에 0.5m/s 제한을 함께 적용한다.

회피 중 차선 추론 억제는 MGM의 AVOID_ACTIVE/GPS_RETURN을 따른다. CSV state=4의 경로 끝까지 유지되는 표시만으로 차선 추론을 계속 억제하지 않는다.
