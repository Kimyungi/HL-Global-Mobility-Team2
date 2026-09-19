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

### v2 적용 — 2026-09-19 누적 라벨 396장 추가 학습

현재 `yolov8n.pt`는 첫 127장 시험 모델 `best.pt`에서 누적 저장 라벨
396장(학습 307 / 검증 89)으로 추가 학습한 최적 5회차 모델이다.
총 15회 학습했으며 검증 정밀도 94.98%, 재현율 88.46%, mAP50 97.70%,
mAP50–95 63.92%다. 같은 촬영 환경의 검증이며 독립 도로 성능은 아니다.

- SHA256: `0e60f7f6996132fcced5f40d54e2311bf4cc63c6e54341d8921ecb0bb930a2e5`
- **단일 클래스 `0: traffic light`**. 이전 모델의 COCO 신호등 ID 9와 다르다.
  검출 노드는 클래스 이름으로 ID를 구하므로 새 모델에서는 자동으로 `[0]`을 사용한다.
- v2 prepare/drive → traffic_zone_supervisor → stack_traffic_node의 기본 모델
  경로에 적용한다. `model_path`를 별도로 지정한 실행은 해당 경로가 우선한다.
- `install_v2` 모델은 이 소스 파일에 연결되어 있다. 실행 중인 검출기는
  가중치를 메모리에 유지하므로 교체 후 검출기 재시작이 필요하다.
- 입력 640, 신규 검출 0.65 / 추적 0.59, HSV 적색 판정,
  정지선 segmentation 및 상태 전이 조건은 유지한다.
- 2200~2299번 비교에서 conf 0.25의 박스 발생은 6→93/100장이었다.
  정답 라벨이 없는 사진의 검출 수이며 정확도가 아니다.
- 교체 전 모델과 실행 설정은 `.cache/model-backups/traffic_before_396_*`에 보관한다.
  배포 메타데이터는 `traffic_deployment_20260919.json`을 따른다.

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

### 신호등 신뢰도 기반 적색 보조 조건 (2026-09-19)

현재 검출 시작 0.65 / 추적 유지 0.59와 별개로, 선택된 YOLO 신호등
박스의 신뢰도가 0.70 이상이면 HSV 색상과 무관하게 적색 관측으로 사용한다.
이는 단일 클래스 모델에 적용한 운영 규칙이며 적색 클래스 확률을 뜻하지 않는다.
최근 5프레임 중 3회 투표 및 기존 정지선 gate는 유지한다.
YOLO 생략 프레임에서는 기존 최대 수명 이내의 정상 템플릿 추적만 이전
적색 근거를 이어간다. 템플릿 점수 자체는 적색 판정에 사용하지 않는다.
다음 YOLO의 낮은 신뢰도/미검출, 추적 소실, target/zone 초기화 시 보조 근거를
해제하며, 생략 프레임에서 HSV 녹색이 보이면 이전 근거를 이어가지 않는다.
미검출이나 0.70 미만이라는 사실만으로 녹색을 판정하지 않는다.
