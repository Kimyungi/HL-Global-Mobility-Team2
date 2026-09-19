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

### v2 실차 적용 — 2026-09-13 저조도 500장 추가학습

현재 `yolov8n.pt`는 `traffic_incremental_500_20260913`의 검증 최상위
11회차 가중치다. 기존 사진 1,941장과 승인된 저조도 사진 500장으로 17회
학습했고, 기존 검증 785장에서 mAP50 94.46%, 재현율 91.85%를 기록했다.
새 500장에서는 모두 검출되고 추가 오검출 상자가 0개였으며, 이는 학습 사진
재평가 결과다. 기존 80클래스 형식과 신호등 ID 9를 유지한다.

- SHA256: `e6a6b87485f7e5ff86d58cb9d8c6d9cac8382f93c65c9402fec1bece41c46f38`
- 실차 입력: `traffic_yolo_image_size:=640`. 320 입력에서는 대표 6장의
  작은 신호등이 모두 누락되어, 통합 런처의 기본값도 640으로 변경했다.
- 신호등 HSV 색상 판정 및 정지선 segmentation 가중치는 그대로 사용한다.
- `install_v2`의 모델 경로는 이 파일에 연결되어 있으므로 재빌드 없이 적용된다.
- 교체 전 가중치와 설정은 이 워크스페이스의 `.cache/model-backups/traffic_*`에
  보관한다. 적용 내역은 `.cache/traffic_model_deployment_latest.json`에 기록한다.

## 정지선 segmentation

`stopline_yolov8s_seg.pt`는 정지선의 픽셀 마스크를 예측한다. USB의
`runs/segment/stopline_20260830_additional/weights/best.pt`를 2026-09-03에
가져왔으며 SHA-256은
`e9bd58a64bbe078e879bc3f8de3342ba31b8aa4f175da6f65f94156c0a52eb26`이다.

`stopline_distance_test.launch.py`와 정지선 검출이 활성화된 노드는 항상 이
segmentation 모델을 쓴다. 기존 색상·윤곽선 검출 경로는 제거되었다.

다른 segmentation 가중치는 `stopline_model_path`로 지정한다.

```bash
ros2 launch stack_traffic stopline_distance_test.launch.py \
  stopline_model_path:=/absolute/path/to/best.pt
```

현재 모델의 라벨은 `stop_line`, `crosswalk`, `other_road_marking` 세 이름을
가지지만 실제 제공 학습 라벨에는 `stop_line`만 존재한다. 검증 영상도 두 개의
연속 촬영에서 나뉜 것이므로, 다양한 날씨·시간·도로에서 별도 test 세트를 만든
뒤 실차 정지 제어를 승인해야 한다. 실제 실행과 안전 확인은
[`RUNBOOK_full_operation_20260904.md`](../../adas_mgm/RUNBOOK_full_operation_20260904.md)를 따른다.
