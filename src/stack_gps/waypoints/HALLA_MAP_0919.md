# 0919 실행 지도

`halla_0919.csv`와 경로별 CSV의 zone_id/inside_zone 및 state를 기준으로 한다.
`halla_route_sequence.yaml`은 `zones_halla_0919_path_01.yaml`~`07.yaml`을 사용한다.
기존 `zones_halla_20260916_path_*.yaml`은 보관용이다.

| 경로 | CSV 범위/표시 | 실행 동작 |
|---|---|---|
| 03 | zone [3], idx 57~116 | 일반 신호등·정지선 감지, 최대 1m/s |
| 03 | zone [4], idx 117~145 | 주차 접근 0.5m/s |
| 03 | state=1, idx 145 | T 주차 |
| 04 | state=4, idx 77 | 회피 진입 |
| 04 | idx 185, state=0 | 옛 평행주차 제거 |
| 05 | state=5, idx 70 | 기존 정지점 정책 |
| 05 | zone [2], idx 300~316, state=3 진입 | 별도 출구 신호 판별 |

zone [1]을 포함한 물리 구역의 경로주행 범위는 CSV에서 읽는다.
신호등 zone [3]을 물리 zone [1]·[2]로 확장하지 않는다.
진입은 기존 station/2.5m preview 및 MGM 확정 샘플 정책을 따른다.
출구는 06·07 사전 로드 후 Left→06, Right→07, 무검출·동률→06이다.
런처를 재실행해야 새 경로 설정이 로드된다.
