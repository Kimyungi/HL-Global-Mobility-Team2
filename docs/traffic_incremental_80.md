# 기존 80개 클래스를 유지하는 신호등 추가 학습

현재 작업은 이전 4,000장 단일 클래스 미세조정을 대체합니다.
기존 `yolov8n.pt`와 80개 COCO 클래스 ID를 그대로 사용합니다.
신호등은 기존 ID 9이며, 새 사진 2,726장을 촬영 세션 단위로 학습 1,941장 / 검증 785장으로 나눴습니다.
새 사진에는 승인 수정 라벨 320장과 미검토 자동 라벨 2,406장이 포함됩니다.
COCO 1,274장과 다른 객체의 의사 라벨은 이번 학습에 사용하지 않습니다.

## 고정 범위

- backbone, neck, 공유 박스 회귀, 분류 head의 공통 특징 추출층 고정.
- 세 스케일의 마지막 분류 convolution에서 `traffic light` 행(9)만 업데이트.
- 나머지 79개 분류 행과 모든 BatchNorm 통계는 고정.
- gradient mask 및 매 optimizer step 후 모델·EMA 복원으로 학습 대상 외 가중치 변화를 방지.
- checkpoint 저장 전에 모델과 EMA의 고정 상태를 검사하고, 다르면 저장을 중단.

모델 구조와 다른 79개 클래스의 NMS 이전 점수·박스 좌표를 유지하는 방식입니다.
검출 후 전체 후보 수 제한(top-k/max_det)은 신호등 점수가 달라지면 최종 목록에 영향을 줄 수 있습니다.
신호등 점수 보정에 학습 범위가 제한되므로 박스 회귀까지 조정하는 전체 미세조정보다 적응 범위가 작습니다.
단순히 80개 클래스 이름만 유지하거나 backbone 일부만 고정한 것과는 다릅니다.

## 검증 및 실행

`test_preserve_traffic_head.py`는 실제 모델에서 여러 optimizer step을 수행해:
신호등 행의 gradient만 남는지, BatchNorm이 변하지 않는지,
다른 79개 클래스와 박스 좌표의 원시 출력이 정확히 같은지, EMA 복원이 되는지 확인합니다.

학습 스크립트 `scripts/training/train_incremental_80.py`:

1. 기존 모델의 785장 검증 성능 기록.
2. CPU에서 최대 30 epochs, patience 8, batch 16, imgsz 512로 위 제한 학습.
3. 학습 완료 후 best.pt의 고정 가중치와 원시 출력을 기존 모델과 비교.
4. 신호등 검증 mAP50-95와 recall이 모두 기존 이상인지 기록.
5. 결과 보고서만 생성하고 ROS 가중치를 자동 교체하지 않음.

현재 기준선은 precision 0.4826, recall 0.3414, mAP50 0.3884, mAP50-95 0.1779입니다.
이는 이 785장 데이터의 지표이며 다른 도로의 성능 보증이 아닙니다.

산출물은 `datasets/traffic_incremental_80/`에 있습니다.

- `training_process.json`: 실행 PID 및 설정
- `training.log`: 진행 로그
- `baseline_metrics.json`: 기존 모델 기준선
- `runs/yolov8n_80class_incremental/results.csv`: epoch별 진행 결과
- `runs/yolov8n_80class_incremental/weights/best.pt`: 후보 가중치
- `training_result.json`: 완료 후 보존 검증·성능 비교 결과

기존 ROS의 단일 신호등 선택·HSV 색상 판정 코드는 이번 학습 변경에서 수정하지 않았습니다.
이전 학습은 첫 epoch checkpoint 저장 시 polars 미설치로 종료됐고 후보 파일이 없었습니다.
polars를 설치하고 해당 직렬화 경로를 확인한 후 이번 학습을 시작했습니다.

## 원본 연산 구조 보존

설치된 Ultralytics가 기존 YAML로 모델을 재구성하면 SPPF 첫 convolution의
활성화가 SiLU에서 Identity로 달라지는 것을 체크포인트 출력 대조에서 발견했습니다.
가중치 모양·값 비교만으로는 이 차이를 잡을 수 없습니다.
`PreserveTrafficTrainer.get_model()`은 원본 모델 모듈을 deepcopy하여 기존 활성화와
연산 속성까지 보존합니다. 원본과 복사 모델의 원시 출력 일치 테스트도 추가했습니다.
문제가 있던 후보는 `runs/incompatible_sppf_attempt/`에 격리했으며 적용 대상이 아닙니다.
최종 정상 후보 경로는 위의 `runs/yolov8n_80class_incremental/weights/best.pt`입니다.
