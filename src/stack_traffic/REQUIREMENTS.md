# stack_traffic — 요구사항

**담당: 김재민** · 산출물: 신호등 정지 (8/2) — 정지선 검출은 이현준 담당

## 역할

신호등 인식 + 이현준의 정지선 거리 입력 → 정지 요구

## 계약 (이것만 지키면 나머지는 자유)

- 입력:
  - camera — YOLOv8n으로 신호등 위치 검출, HSV로 적색 여부 판정.
  - `/perception/stopline` (`fma_interfaces/StopLine`) — 이현준의 정지선 결과.
    `detected=true`일 때 `distance`[m]가 유효하며, `header.stamp`를 신선도 판단에 사용.
- 출력: `/perception/traffic_stop` (`fma_interfaces/TrafficStop`).
  - `stop_required`는 **요구이지 명령이 아니다**: 적용 여부·우선권은 MGM 스테이트 머신이 결정
    (lane·waypoint에서는 긴급 정지 다음 순위, parking에서는 비활성 — CLAUDE.md §4).
- 적색 정지 래치는 적색 투표가 해제될 때만 `stop_required=False`로 해제.
  정지선 미검출·stale 또는 카메라 프레임 수신 실패는 해제 조건이 아니다.
- 금지: v_ref 직접 산출 금지. 정지 거리 프로파일링이 필요하면 stop_distance로 전달만.

## 기본 결합 조건

- `/home/jaemin/traffic_red_binary_test.py`의 적색 이진 판정 로직을 사용.
- 최근 5프레임 중 적색 3프레임 이상이면 `red_active=True`.
- `red_active` AND 정지선 거리 0.5m 이내이면 정지 래치를 활성화하여
  `stop_required=True`.
- 래치 활성화 후 정지선이 시야에서 사라지거나 입력이 stale이어도 정지 요구를 유지.
- 정상 카메라 프레임 처리에서 `red_active=False`가 되면 래치를 해제.
- 카메라 프레임 수신 실패 시에는 적색 투표와 래치를 변경하지 않고 최신 정지 요구를 재발행.
- 적색이 아니거나 신호등 미검출이면 투표에 0으로 반영.
- 정지선 입력은 `header.stamp`로부터 0.5초가 지나면 stale로 판단하며
  `stop_distance=-1.0`을 출력.

## Python 의존성

```bash
python3 -m pip install -r src/stack_traffic/requirements.txt
```

OpenCV와 NumPy는 ROS PC의 Ubuntu 패키지를 사용한다.

```bash
sudo apt install python3-opencv python3-numpy
```

## YOLO 모델 배포

- 기본 검색 위치: 설치된 패키지의 `share/stack_traffic/models/yolov8n.pt`.
- 저장소의 `models/`에 모델을 배치하고 다시 빌드하면 기본 위치로 설치된다.
- 모델 파일을 저장소에 포함하기 전에는 용량과 라이선스를 확인해야 한다.
- `model_path` 파라미터를 지정하면 기본 검색 위치보다 우선한다.

## 실행

```bash
ros2 run stack_traffic stack_traffic_node --ros-args \
  -p camera_source:="2" \
  -p stop_trigger_distance_m:=0.5
```

위 명령은 패키지의 `models/yolov8n.pt`를 자동으로 사용한다.
패키지 밖의 모델을 사용할 때만 `model_path`를 지정한다.

## 공통 규칙 (CLAUDE.md)

- 출력은 `fma_interfaces` 메시지로만. MGM은 이 토픽만 구독한다.
- 경로를 내는 스택은 전부 동일 ref points 포맷 — {x, y, yaw, curvature}, vehicle frame (§5.4).
- 판단 로직(모드 전환·정지 결정·우선권)은 MGM 스테이트 머신에만 존재한다 (§4, §5.1).
- 실행: `ros2 run stack_traffic stack_traffic_node`
