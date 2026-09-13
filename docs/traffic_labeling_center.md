# 용인 중앙 신호등 재라벨링

프로젝트: <https://app.roboflow.com/xanadu-ef7kh/yongin-oak-d-center-traffic-light-3000>

원본 3,000장을 동일한 고정 ROI로 자른 후 SAM 3로 다시 라벨링합니다.
기존 전체 화면 프로젝트와 라벨은 보존합니다. 공개 라이선스는 BY-NC-SA 4.0입니다.

- 가로 30~70%, 세로 0~50%: 1280×720 입력에서 `(384, 0)–(896, 360)`.
- 출력 512×360. 왜곡이나 확대 없이 자릅니다.
- 클래스 `0: traffic light`. 점등 램프만이 아니라 신호등 등기구 전체 외곽입니다.
- ROI 안에 여러 신호등이 보이면 모두 라벨링합니다. 주변 신호등을 배경으로 오학습시키지 않으며, 대상 하나 선택은 ROS가 합니다.
- ROI 밖의 신호등은 학습 영상에도 ROS 검색에도 포함되지 않습니다.
- SAM 3 설명: `complete traffic light housing`, confidence 0.4.
- 미검출 사진은 자동 라벨이 비어 있는 초안입니다. 실제 배경인지 가림/누락인지 검토가 필요합니다.

`datasets/yongin_traffic_center/manifest.json`에 원본 경로, crop 좌표, 세션을 기록합니다.
이미지는 `images/`, YOLO 초안은 `roboflow_labels_draft/`이며 같은 파일명으로 대응합니다.
라벨 좌표는 **잘라낸 512×360 이미지** 기준이며 원본 1280×720에 그대로 적용하면 안 됩니다.
원본 좌표로 되돌릴 때 x에 384를 더하고 y는 그대로 둡니다.

## ROS 동작

`central_traffic_only:=true`는 아래 설정을 적용합니다.

- 학습과 같은 `CENTRAL_TRAFFIC_ROI`를 고정 검색 영역으로 사용합니다.
- 좌우 타일 순회 없이 중앙 crop 전체를 추론합니다.
- 신규 대상은 최소 크기·신뢰도 조건을 통과한 후보 중 **가로 중앙선에 가장 가까운 하나**입니다.
- 동률에서는 신뢰도, 면적 순입니다. 선택 후에는 기존 위치 연결 규칙으로 같은 대상을 유지합니다.
- 전체 화면으로 벗어날 수 있는 template tracking은 비활성화합니다.
- 색상은 기존 HSV 빨강/초록 판정, 정지·재출발 판단은 기존 로직을 사용합니다.

이 규칙은 화면상의 중앙을 기준으로 합니다. 차로·경로 의미를 해석해 해당 차로 신호임을 보장하는 규칙은 아닙니다.
카메라 위치가 바뀌면 crop과 실제 대상 위치를 다시 확인해야 합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select stack_traffic --symlink-install
source install/setup.bash
# 실제 학습 완료한 가중치의 절대경로로 교체
ros2 launch stack_traffic central_traffic_test.launch.py model_path:=/absolute/path/to/best.pt
```

이 launch는 기존 교통 카메라 진단 launch를 중앙 모드로 포함합니다.
새 모델 학습·가중치 교체·차량 제어 launch 실행은 이 재라벨링 작업에 포함하지 않습니다.
`model_path`를 비우면 기존 모델을 사용하므로 새 모델 학습 완료로 해석하면 안 됩니다.
기존 launch와 노드의 기본값은 이전 동작(`central_traffic_only=false`)입니다.

세 세션을 우선 전부 train으로 업로드하지만 최종 학습 분할은 아닙니다.
학습 전에 촬영 구간별로 train/val/test를 분리하고 라벨을 검토해야 합니다.

## 이번 실행 결과

3,000장 전부 업로드·SAM 3 재라벨링·로컬 다운로드 완료.
박스 3,064개, 미검출 302장, 복수 후보 292장입니다.
미검출에는 중앙 ROI에 신호등이 없는 장면과 가림/검출 누락이 섞일 수 있으므로 정답 배경으로 자동 승인하지 않았습니다.
복수 후보는 검출용 학습 라벨로 보존하며 ROS의 중앙 선택 로직이 한 개를 고릅니다.
서버 작업 세 건 모두 Review 상태이며, 로컬 모든 라벨의 클래스와 좌표 범위를 검증했습니다.

검토 화면을 새 데이터셋으로 여는 명령:

```bash
ros2 run stack_traffic traffic_label review --dataset datasets/yongin_traffic_center
```

중앙 선택·ROI 일치·기존 색상/정지 판단 등 관련 테스트 66개, ROS 패키지 빌드 및 launch 인자 로딩을 확인했습니다.
실제 카메라/차량에서 새 중앙 모드를 구동한 검증은 아직 수행하지 않았습니다.
