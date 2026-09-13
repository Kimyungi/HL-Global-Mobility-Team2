# 용인 신호등 3,000장 수동 라벨링

ROS 2 Humble의 `stack_traffic` 패키지에서 실행하는 오프라인 사진 라벨러입니다.
카메라 또는 차량 제어 노드를 실행하지 않고 촬영한 RGB 원본을 사용합니다.
연결 이력(`src/adas_mgm/RUNBOOK_full_operation_20260904.md`)의 교통용 OAK-D는
MxID `14442C10B167CFD200`입니다. 차선용 `14442C105157D3D200`와 구분하세요.

## 실행

프로젝트 루트에서 실행합니다. 현재 PC에는 GUI 의존성이 설치되어 있습니다.
다른 PC에는 `python3-tk`, `python3-pil`, `python3-pil.imagetk`가 필요합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select stack_traffic --symlink-install
source install/setup.bash

# 실제 용인 사진 폴더를 지정하세요. JPG/PNG/BMP/TIFF, 하위 폴더 포함.
ros2 run stack_traffic traffic_label prepare --source /실제/용인/사진폴더
ros2 run stack_traffic traffic_label label
```

기본 저장 위치는 `datasets/yongin_traffic`이며 Git에서 제외됩니다.
다른 위치는 모든 명령에 `--dataset /저장/경로`를 지정하세요.
원본을 보존하고 EXIF 회전을 반영한 PNG 사본을 만듭니다. 충분한 디스크 공간이 필요합니다.
동일 파일 내용은 중복 제거합니다. 3,000장보다 많으면 정렬된 목록 전체에서 균등 간격으로
3,000장을 선택하며, 적으면 실제 장수를 표시합니다. 신호등 포함 여부를 자동 선별하지는 않습니다.
`manifest.json`에 원본 경로, 해시, 크기를 기록합니다. 이미 준비된 폴더는 덮어쓰지 않습니다.
rosbag/동영상은 이 도구에 직접 입력할 수 없으며 먼저 RGB 사진으로 추출해야 합니다.

## 작업 방법

- 왼쪽 마우스 드래그: 신호등 박스 추가. 여러 신호등은 각각 박스를 그립니다.
- `+` / `-`: 확대/축소. 확대 후 오른쪽 드래그로 화면 이동.
- `Z`: 마지막 박스 삭제. 기존 박스도 삭제 후 다시 그릴 수 있습니다.
- `S`: 저장 후 다음 사진. `A` / `D`: 이전/다음 사진.
- `N`: 신호등 없는 배경 사진으로 확인 후 저장. 빈 라벨 파일을 만듭니다.
- 다시 실행하면 첫 번째 미완료 사진부터 이어집니다. 종료/이동 시 미저장 변경을 확인합니다.

클래스는 **0: traffic light** 하나입니다. 현재 검출 코드가 이 이름을 인식하고,
빨강/노랑/초록은 박스 안의 HSV로 별도 판정합니다.
불이 켜진 작은 원만이 아니라 **하나의 신호등 등기구 전체 외곽**을 타이트하게 감싸세요.
기둥이나 배경은 제외하고, 적색/녹색/꺼진 신호등에도 동일한 기준을 적용하세요.
작거나 가려진 신호등도 식별 가능하면 보이는 외곽을 일관되게 표시하세요.
판단하기 어려운 사진은 `D`로 미완료 상태로 남겨 나중에 재검토하세요.
검출 결과나 글자가 그려진 디버그 화면 대신 원본 RGB를 사용하세요.

## 저장 형식과 확인

`images/yongin_000001.png` ↔ `labels/yongin_000001.txt` 형태입니다.
YOLO 라벨 한 줄은 `class cx cy width height`이며 좌표는 원본 크기 기준 0~1입니다.
라벨 파일 없음은 미완료, 빈 파일은 확인한 배경입니다.

```bash
ros2 run stack_traffic traffic_label check
```

검증 명령은 이미지 읽기, 클래스와 좌표 범위, 완료 장수를 확인합니다.
학습 전 모든 사진의 라벨을 검토하고 별도 백업하세요.
학습/검증/테스트 분할과 학습 실행은 아직 수행하지 않습니다.
연속 촬영의 비슷한 프레임이 학습과 검증에 섞이지 않도록 촬영 구간 단위로 분리하세요.

## 현재 PC의 용인 원본

`/home/sangmin/FMA_ws/training_data/` 안의 다음 세 폴더를 사용합니다.
각각 1,000장, 1280×720 RGB 사진입니다.

- `신호등_학습용_사진_20260905_145914`
- `신호등_학습용_사진_20260905_150256`
- `신호등_학습용_사진_20260905_151147`

다른 목적인 `완주_전_출구_결정용_사진_*`와 ZIP 사본은 포함하지 않습니다.
새 데이터셋을 준비할 때 선택 명령은 다음과 같습니다. 기존 데이터셋에서 작업을
계속할 때는 `prepare`를 반복하지 말고 `label`만 실행하세요.

```bash
ros2 run stack_traffic traffic_label prepare \
  --source /home/sangmin/FMA_ws/training_data \
  --include '신호등_학습용_사진_*/*.jpg'
```

## Roboflow 자동 라벨링

사용자 승인으로 생성한 공개 프로젝트:
<https://app.roboflow.com/xanadu-ef7kh/yongin-oak-d-traffic-light-3000>

라이선스: `BY-NC-SA 4.0`. 세 촬영 세션은 각각 `yongin_20260905_145914`,
`yongin_20260905_150256`, `yongin_20260905_151147` 업로드 배치로 구분합니다.
클래스는 `traffic light`, 자동 라벨링 모델은 SAM 3입니다.
전체 작업 프롬프트는 `complete traffic light housing`, confidence는 0.4입니다.
첫 4장 테스트는 `traffic light`, confidence 0.35를 사용했습니다.
자동 결과는 Roboflow Annotate에서 검토 후 확정해야 합니다.

API 키는 저장소 밖 `/home/sangmin/.config/roboflow/api_key`에 권한 600으로 저장됩니다.
키 내용을 소스 코드, 로그, Git에 넣지 마세요.
로컬 `datasets/yongin_traffic/roboflow_uploads.jsonl`은 원본 사진과 서버 이미지 ID를
연결하고, `roboflow_jobs.json`은 자동 라벨링 작업 ID와 서버 상태를 기록합니다.
업로드의 임시 split 값은 전부 train이며 학습용 최종 분할이 아닙니다.
학습 버전 생성 전에 촬영 구간 단위의 검증/테스트 분할을 별도로 정해야 합니다.

자동 라벨 초안의 로컬 YOLO 사본은 `datasets/yongin_traffic/roboflow_labels_draft/`입니다.
같은 번호의 `images/yongin_XXXXXX.png`와 대응합니다. 기존 수동 라벨 폴더 `labels/`는
자동 결과로 덮어쓰지 않습니다. 자동 라벨이 완료되어도 정확도가 검증된 정답은 아니므로,
Roboflow Annotate의 Review에서 누락·중복 박스를 확인하고 확정하세요.

## 자동 라벨 검토 화면

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run stack_traffic traffic_label review
```

`review_queue.json` 순서대로 미검출 24장 → 중복 의심 199장 → 일반 사진을 보여줍니다.
중복 의심은 작은 박스 면적의 80% 이상이 다른 박스와 겹치는 기준이며 실제 오류 확정은 아닙니다.
자동 라벨을 미리 표시하고 `S`로 수정 결과를 저장·승인합니다.
`Shift+클릭`으로 해당 박스를 삭제할 수 있습니다. 겹치면 작은 박스부터 삭제합니다.
왼쪽 드래그로 새 박스를 그리고 `N`은 실제 신호등 없는 배경, `X`는 학습 제외입니다.
`D`는 승인 없이 건너뛰며, 다음 실행 시 미검토 사진부터 이어집니다.

승인한 파일은 `reviewed_labels/`, 승인·제외 기록은 `review_decisions.json`에 저장합니다.
학습 대상으로 고를 때는 파일 존재만 보지 말고 decisions의 최신 상태가 `approved` 또는
`approved_background`인지 확인해야 합니다. 이전 승인 후 제외로 바꾼 파일도 남아 있습니다.
이 화면의 결정은 로컬에 저장되며 Roboflow 서버의 Review 상태에는 아직 동기화되지 않습니다.
