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

## 정지선 segmentation

`stopline_yolov8s_seg.pt`는 정지선의 픽셀 마스크를 예측한다. USB의
`runs/segment/stopline_20260830_additional/weights/best.pt`를 2026-09-03에
가져왔으며 SHA-256은
`e9bd58a64bbe078e879bc3f8de3342ba31b8aa4f175da6f65f94156c0a52eb26`이다.

`stopline_distance_test.launch.py`는 기본적으로 이 모델을 쓴다. 기존
색상·윤곽선 방식으로 비교하려면 다음처럼 실행한다.

```bash
ros2 launch stack_traffic stopline_distance_test.launch.py \
  stopline_detector_type:=color
```

다른 segmentation 가중치는 `stopline_model_path`로 지정한다.

```bash
ros2 launch stack_traffic stopline_distance_test.launch.py \
  stopline_detector_type:=yolo_seg \
  stopline_model_path:=/absolute/path/to/best.pt
```

현재 모델의 라벨은 `stop_line`, `crosswalk`, `other_road_marking` 세 이름을
가지지만 실제 제공 학습 라벨에는 `stop_line`만 존재한다. 검증 영상도 두 개의
연속 촬영에서 나뉜 것이므로, 다양한 날씨·시간·도로에서 별도 test 세트를 만든
뒤 실차 정지 제어를 승인해야 한다. 실제 실행과 안전 확인은
[`RUNBOOK_full_operation_20260904.md`](../../adas_mgm/RUNBOOK_full_operation_20260904.md)를 따른다.
