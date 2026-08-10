# stack_traffic — OAK-D 신호등·정지선 정지 요구

담당: 김재민

## 역할과 계약

`stack_traffic`이 OAK-D Pro 한 대에서 다음 작업을 모두 수행한다.

1. RGB 상단 ROI에서 YOLOv8n으로 주행 대상 신호등 bbox 검출
2. bbox 내부 HSV 비율로 적색·초록색 판정
3. RGB 하단 ROI에서 긴 흰색 횡방향 정지선 검출
4. 정렬 depth에서 정지선 인접 노면의 optical-Z 측정
5. 적색과 정지선 근접 조건을 결합해 `/perception/traffic_stop` 발행

외부 `/perception/stopline`은 구독하지 않는다. CAN과 `v_ref`도 직접 만들지
않는다. MGM이 `TrafficStop.stop_required`를 `v_ref=0`으로 병합하고
`bridge_dspace`가 CAN으로 전송한다.

## 신호등 판정

- 설치 모델은 `models/yolov8n.pt` 하나다. 별도 ONNX 색상 분류기는 사용하지
  않는다.
- YOLO는 기본 2프레임마다 실행한다. 중간 프레임은 짧은 template 추적으로
  화면 표시와 적색 유지에만 사용한다.
- 신규 bbox는 `confidence_threshold`, 기존 bbox와 이어지는 후보는 더 낮은
  `tracking_confidence_threshold`를 사용한다.
- template 추적은 fresh YOLO 검출 뒤 최대 age와 연속 실패 제한 안에서만
  허용한다. 유효한 template이 이어지는 동안 단순 YOLO miss만으로 bbox를
  버리지 않고, 만료 후 오래된 template을 다시 살리지 않는다.
- 적색은 최근 유효 관측 5개 중 3개 이상이면 활성화한다.
- unknown·미검출은 색상 투표창을 진행시키지 않는다. template 적색 투표는
  동일 target에서 fresh YOLO 적색을 먼저 확인한 뒤에만 허용한다. 초록
  해제는 fresh YOLO에서만 허용한다.
- 패키지와 실험 launch 모두 fresh YOLO bbox의 초록 3/5로만 해제하는
  `resume_on_green=true`가 기본이다. `resume_on_red_clear`는 끈다.

## 정지선 판정

- 하단 ROI에서 저채도·고명도 마스크를 만든다.
- 폭, 가로세로비, 중심 통과, 각도, 두께, 채움률 조건으로 분리된 횡단보도
  무늬와 차선 오검출을 거른다.
- 최근 5프레임 중 최소 3프레임에서 차량 쪽 경계 y가 안정적이어야 한다.
- 흰 도색 자체의 stereo 무효값을 피하기 위해 정지선 주변 노면 depth를 쓰고,
  행별 중앙값에 `1/Z` 모델을 맞춘다. 기울기·잔차·일관성 검사를 통과한 값만
  최근 depth 중앙값에 넣는다.

정지선 근접 gate는 두 가지다.

- `stopline_stop_y_ratio`: 전체 영상 높이 대비 정지선 회전 사각형의
  최하단 끝점 y를 시간축으로 안정화한 위치.
  고정 장착 카메라에서 위치 구분의 기본 gate로 사용한다. 회전 사각형으로
  추정한 차량 쪽 경계가 화면 아래로 외삽되면 1을 조금 넘을 수 있으며,
  유효한 임계값 범위는 0~1.10이다.
- `stopline_stop_distance_m`: OAK optical-Z 중앙값. 현장 로그에서 두 위치가
  확실히 구분될 때만 보조 gate로 사용한다.

값이 `0`인 gate는 비활성이다. 하나만 켜면 그 gate만 사용하고, 둘 다 켜면
조기 정지를 막기 위해 두 조건을 모두 만족해야 한다. 둘 다 `0`이면 측정
전용 모드라서 정지 요구를 만들지 않는다.

최종 진입 조건은 다음과 같다.

```text
red_active
AND stable_stopline
AND enabled_stopline_gates
```

`TrafficStop.stop_distance`에는 유효한 optical-Z 미터 값만 넣고 frame id를
`oak_rgb_optical_frame`으로 표시한다. depth가 무효하면 `-1.0`이다. y ratio를
거리 필드에 넣지 않는다.

## 카메라와 성능

- OAK-D가 두 대 이상 연결되는 통합 환경에서는 `oak_mxid`에 교통용 카메라의
  MxID를 반드시 지정한다. 빈 값은 OAK-D가 한 대뿐인 단독 시험에서만 자동
  선택하며, 여러 대가 보이면 노드는 잘못된 장치 사용을 막기 위해 종료한다.
  0대 열거 또는 단일 장치 open 실패는 2초 간격으로 최대 3회 재시도하고,
  끝까지 실패하면 무핀 자동 선택 없이 종료한다. 현재 차량의 교통용 MxID는
  현장 launch 기본값을 단일 기준으로 사용하며, 다른 장치에서 시험할 때만
  launch 인자로 재정의한다.
- Python 의존성은 `python3 -m pip install -r
  src/stack_traffic/requirements.txt`로 설치한다. DepthAI 3.x는 현재 파이프라인과
  호환하지 않으므로 `depthai>=2.30,<3.0` 범위를 유지한다.
- 기본 실험 입력: 1280x720, 30 FPS, RGB 정렬 depth.
- y-only 실차 모드에서는 stereo depth를 장치 단계에서 꺼 RGB 처리량을
  우선한다. depth gate를 사용할 때만 다시 켠다.
- YOLO의 논리 검색 범위는 넓은 상단 ROI다. 추적 전에는 이 범위를
  중앙·좌·우의 좁은 가로 타일로 한 장씩 훑고, 검출 뒤에는 bbox 중심을
  따라가는 타일만 사용해 먼 신호의 입력 픽셀 크기를 보존한다.
- `imgsz=576`, 직사각형 입력, 격프레임 추론을 사용하며 한 프레임에 타일
  하나만 처리한다.
- 보조 crop/mask/depth 창은 기본적으로 끈다. 전체 depth 색상화는 보조 창을
  켰을 때만 수행한다.
- depth decimation 결과가 RGB보다 작으면 종횡비를 확인한 뒤 nearest-neighbor로
  RGB 크기에 맞춘다. 종횡비가 다르면 프레임을 거절한다.
- 50 Hz polling에서 OAK queue가 비어 있는 것은 정상이다. 마지막 정상 프레임
  뒤 0.5초가 지나야 카메라 fault로 판단한다.
- 카메라 fault는 안전 정지로 래치되며 노드를 재시작해야 해제된다.
- 실차 정지 gate가 활성인 기동 직후에는 5회 YOLO 판단창과 정지선
  필터창이 준비될 때까지 `stop_required=true`를 발행한다.
- 모델·OpenCV·정지선 처리 예외도 카메라 fault와 같이 정지로 래치한다.

## 현장 보정

카메라 위치·각도를 완전히 고정한 뒤 측정 모드에서 로그의 다음 값을 기록한다.

```text
y_raw, y_ratio, y_med, line_z, z_med, stable, accepted
```

`stopline_stop_y_ratio`는 흔들리는 raw 값이 아니라 `y_med` 기준으로 정한다.
최종 정차 위치의 값을 그대로 쓰면 그 위치에서야 감속을 시작하므로 지나칠 수
있다. 첫 시험은 0.28m/s 이하에서 보수적으로 조금 이른 요청 위치를 사용하고,
실제 MGM·차량 응답을 측정해 임계값을 조정한다.
현장값 `0.98`은 현재 정지선 ROI, 고정된 카메라 장착 자세, 0.28m/s 이하에서만
검증된 값이며 범용 기본값이 아니다. 장착 자세·ROI·속도를 바꾸면 다시 측정한다.

## 안전 조건

- MGM은 `/perception/traffic_stop` 미수신/시간 초과 0.5초 후
  `traffic_stop_required=true`로 보정해야 한다(LANE/WAYPOINT).
- 카메라·모델·프로세스 장애와 무관하게 물리 E-stop 담당자가 즉시
  제어할 수 있어야 한다.
- dSPACE의 CAN counter watchdog이 실제로 동작하는지 먼저 확인한다.
- 실제 주행은 유효한 E-stop heartbeat와 경로 입력이 있어야 한다.
- `dummy_ref_publisher`와 MGM을 동시에 실행하지 않는다.
- 신호등 정지는 MGM의 LANE/WAYPOINT 상태에서만 적용된다.

실행 명령과 터미널 배치는 저장소 루트의
`TRAFFIC_STOP_TEST_COMMANDS.md`를 따른다.
