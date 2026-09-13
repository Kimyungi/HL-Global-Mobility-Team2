# 신호등 4,000장 학습 실행

현재 `src/stack_traffic/models/yolov8n.pt` 체크포인트의 `train_args.data`는
`coco.yaml`이며 80개 클래스 중 9번이 `traffic light`입니다.
사진은 가중치에 포함되어 있지 않아 COCO 원본과 정답 라벨을 별도로 확보했습니다.
출처: https://docs.ultralytics.com/datasets/detect/coco/

- 용인 새 사진 2,726장: 승인 수정 320장 + 자동 라벨 2,406장. 제외 274장은 포함하지 않음.
- COCO train2017 신호등 사진 4,139장 중 중앙 crop에 신호등이 남는 2,323장에서
  시드 `20260908`로 1,274장을 무작위 추출. 클래스 9를 단일 클래스 0으로 변환.
- 양쪽 모두 가로 30~70%, 세로 0~50% 중앙 crop. 총 4,000장 고유 이미지.
- 데이터 경로: `datasets/traffic_training_4000/dataset/data.yaml`.
- 학습 2,961장 / 검증 912장 / 시험 127장. 용인 15:02:56 촬영 전체는 검증 전용.
  COCO는 서로 다른 이미지 단위로 1,020/127/127 분리.
- 시험 127장은 COCO만 포함하므로 용인 실차에서의 독립 시험을 대신하지 않습니다.

원본 COCO 파일 ID와 URL, 라벨 출처, crop 위치, SHA-256은 데이터셋 manifest에 기록합니다.
COCO 선택 목록은 `datasets/traffic_training_4000/coco_selection.json`에 있습니다.
이미지 라이선스는 COCO 원본 이미지별 조건을 따릅니다.

기존 YOLOv8n 사전학습 가중치에서 단일 클래스 검출기로 미세조정합니다.
CPU, 30 epochs(조기 종료 patience 8), imgsz 512, batch 8,
backbone freeze 10, mosaic/mixup/좌우반전 비활성으로 실행합니다.
현재 GPU는 사용할 수 없어 시스템 드라이버를 변경하지 않고 CPU로 실행했습니다.

학습 프로세스 정보: `datasets/traffic_training_4000/training_process.json`.
로그: `datasets/traffic_training_4000/training.log`.
진행 지표: `datasets/traffic_training_4000/runs/yolov8n_center_4000/results.csv`.
후보 가중치: 같은 run의 `weights/best.pt`.
완료 후 시험 결과: `datasets/traffic_training_4000/runs/training_result.json`.

기존 ROS 가중치는 덮어쓰지 않았습니다. 학습 완료·시험 성능과 실제 검출 결과를
확인한 다음 `central_traffic_test.launch.py model_path:=...`로 적용할 후보입니다.
학습 스크립트 자체는 가중치를 자동으로 활성화하지 않습니다.

재현 스크립트는 `scripts/training/`의 `prepare_reviewed_traffic.py`,
`add_coco_traffic.py`, `build_mixed_traffic.py`, `train_traffic_4000.py`입니다.
실행 중인 학습이 있는 상태에서 학습 스크립트를 중복 실행하지 마세요.
