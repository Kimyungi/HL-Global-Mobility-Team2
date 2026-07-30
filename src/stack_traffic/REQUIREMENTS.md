# stack_traffic — 요구사항

**담당: 김재민** · 산출물: 신호등 정지 (8/2) — 정지선 검출은 이현준 담당

## 역할

신호등 인식 + 이현준의 정지선 거리 입력 → 정지 요구

## 계약 (이것만 지키면 나머지는 자유)

- 입력:
  - camera — YOLOv8n으로 신호등 위치 검출, HSV로 적색 여부 판정.
  - `/perception/stopline_distance` (`std_msgs/Float32`) — 이현준의 정지선까지 거리[m].
    0 이상은 유효 거리, `-1.0`은 미검출.
- 출력: `/perception/traffic_stop` (`fma_interfaces/TrafficStop`).
  - `stop_required`는 **요구이지 명령이 아니다**: 적용 여부·우선권은 MGM 스테이트 머신이 결정
    (lane·waypoint에서는 긴급 정지 다음 순위, parking에서는 비활성 — CLAUDE.md §4).
- 적색 투표가 해제되면 stop_required=False — 출발 판단도 MGM
  (요구 소멸 = 기본 속도 복귀).
- 금지: v_ref 직접 산출 금지. 정지 거리 프로파일링이 필요하면 stop_distance로 전달만.

## 기본 결합 조건

- `/home/jaemin/traffic_red_binary_test.py`의 적색 이진 판정 로직을 사용.
- 최근 5프레임 중 적색 3프레임 이상이면 `red_active=True`.
- `red_active` AND 정지선 거리 0.5m 이내이면 `stop_required=True`.
- 적색이 아니거나 신호등 미검출이면 투표에 0으로 반영.
- 정지선 입력은 0.5초가 지나면 stale로 판단하며 `stop_distance=-1.0`을 출력.

## 실행

```bash
ros2 run stack_traffic stack_traffic_node --ros-args \
  -p model_path:=/home/jaemin/yolov8n.pt \
  -p camera_source:="2" \
  -p stop_trigger_distance_m:=0.5
```

## 공통 규칙 (CLAUDE.md)

- 출력은 `fma_interfaces` 메시지로만. MGM은 이 토픽만 구독한다.
- 경로를 내는 스택은 전부 동일 ref points 포맷 — {x, y, yaw, curvature}, vehicle frame (§5.4).
- 판단 로직(모드 전환·정지 결정·우선권)은 MGM 스테이트 머신에만 존재한다 (§4, §5.1).
- 실행: `ros2 run stack_traffic stack_traffic_node`
