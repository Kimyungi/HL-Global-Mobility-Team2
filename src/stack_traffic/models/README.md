# 신호등 모델

## 객체 검출

`yolov8n.pt`가 신호등 bbox를 검출한다. 다른 YOLO 모델을 쓰려면 실행 시
다음 파라미터로 지정한다.

```bash
-p model_path:=/absolute/path/to/model.pt
```

신호등 색상은 검출된 bbox 안에서 HSV 비율로만 판정한다. 별도 ONNX 색상
분류기는 실험 결과 대비 연산·설정 복잡도가 커서 실행 경로와 설치 대상에서
제외했다.

## 정지선 세그멘테이션 모델

`stopline_yolov8s_seg.pt`는 USB의 다음 학습 결과에서 가져온 YOLOv8s-seg
모델이다.

```text
YOLOv8s-seg 2/YOLOv8s-seg/runs/segment/
stopline_20260830_additional/weights/best.pt
```

- SHA-256: `e9bd58a64bbe078e879bc3f8de3342ba31b8aa4f175da6f65f94156c0a52eb26`
- 입력 크기: 640
- 클래스: `stop_line`, `crosswalk`, `other_road_marking`

정지선 운영 검출 경로는 이 세그멘테이션 모델을 사용한다. 모델의 `stop_line`
mask에서 bbox와 차량 쪽 최하단 y를 구한 뒤 기존의 3/5 프레임 안정화와 OAK-D
depth 거리 검사를 적용한다. 실차 실행과 확인 명령은
[`RUNBOOK_full_operation_20260830.md`](../../adas_mgm/RUNBOOK_full_operation_20260830.md)를 따른다.
