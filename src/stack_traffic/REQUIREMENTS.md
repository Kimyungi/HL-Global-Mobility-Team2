# stack_traffic — OAK-D 신호등·정지선 정지 요구

담당: 김재민

## 역할과 계약

`stack_traffic`이 OAK-D Pro 한 대에서 다음 작업을 모두 수행한다.

1. RGB 상단 ROI에서 YOLOv8n으로 주행 대상 신호등 bbox 검출
2. bbox 내부 HSV 비율로 적색·초록색 판정
3. 적색 확정 뒤 RGB 하단 ROI에서 주간 흰색·야간 국소 대비·평행 에지 쌍으로 횡방향 정지선 검출
4. 선택적 저해상도 depth 진단에서 정지선 인접 노면의 optical-Z 측정
5. 적색/초록 상태와 최초 래치용 metric 정지선 거리를 `/perception/traffic_stop`으로 발행

외부 `/perception/stopline`은 구독하지 않는다. CAN과 `v_ref`도 직접 만들지
않는다. MGM은 적색에서 TRAFFIC 상태에 진입하고, 최초 유효 정지선 거리를 래치한
뒤 `/vehicle/vector.v`를 적분해 정지선 소실 이후의 남은 거리를 계산한다.
`bridge_dspace`가 목표속도를 CAN으로 전송하고 dSPACE 실차속도를 다시 돌려준다.

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
- 적색 3/5가 한 번 확정되면 `red_phase_latched=true`로 기억하고 당시 신호등
  bbox를 anchor로 저장한다. YOLO가 초록 화살표·비원형 초록 점등을 놓치더라도
  저장된 동일 bbox 안의 초록색 3/5는 fresh 해제 관측으로 인정한다. anchor 밖의
  초록색, anchor가 없는 상태의 초록색, unknown·황색은 해제 근거가 아니다.
  적색과 초록이 동시에 활성화된 비정상 투표창에서는 적색이 우선한다.
- unknown·미검출은 색상 투표창을 진행시키지 않는다. template 적색 투표는
  동일 target에서 fresh YOLO 적색을 먼저 확인한 뒤에만 허용한다. 초록 해제는
  fresh YOLO bbox 또는 확정 적색 anchor 안의 최신 프레임에서만 허용한다.
- 패키지와 실차·실험 launch 모두 위 조건의 초록 3/5에서 즉시 해제하는
  `resume_on_green=true`를 기본으로 둔다. `resume_on_red_clear`는 끈다.

## 정지선 판정

- 하단 ROI에서 기존 저채도·고명도 마스크와 CLAHE 기반 국소 대비 마스크를 OR 결합한다.
  야간 기본값은 LAB 명도 60 이상, 주변 노면 대비 25 이상이다.
- 색상 마스크와 별도로 Canny/Hough로 위·아래 수평 에지 쌍을 찾는다. 각 에지는 ROI
  폭의 35% 이상, 절대 각도 ±12도, 상호 각도 차 4도 이내여야 하며 두 에지 사이가
  위·아래 노면보다 밝기 8 이상이어야 한다. 단일 에지·그림자는 후보가 아니다.
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

TRAFFIC 상태 진입은 정지선과 무관하게 다음 조건 하나다.

```text
red_phase_latched
```

진입 뒤에만 정지선 검출을 수행한다. 최초 `stable_stopline`과 유효 depth 거리가
확정되면 MGM이 그 거리를 한 번 래치한다. 약 1m 안쪽에서 정지선이 검출되지 않아도
정상이며, 그 뒤에는 영상 거리 대신 실차속도 적분값을 사용한다. 확정 초록은
TRAFFIC을 즉시 해제한다.

`TrafficStop.stop_distance`에는 유효한 optical-Z 미터 값만 넣고 frame id를
`oak_rgb_optical_frame`으로 표시한다. depth가 무효하면 `-1.0`이다. y ratio를
거리 필드에 넣지 않는다.

## 카메라와 성능

- OAK-D가 두 대 이상 연결되는 통합 환경에서는 `oak_mxid`에 교통용 카메라의
  MxID를 반드시 지정한다. 빈 값은 OAK-D가 한 대뿐인 단독 시험에서만 자동
  선택하며, 여러 대가 보이면 노드는 잘못된 장치 사용을 막기 위해 종료한다.
  0대 열거 또는 단일 장치 open 실패는 5초 간격으로 최대 4회(약 15초 창)
  재시도하고,
  끝까지 실패하면 무핀 자동 선택 없이 종료한다. 현재 차량의 교통용 MxID는
  현장 launch 기본값을 단일 기준으로 사용하며, 다른 장치에서 시험할 때만
  launch 인자로 재정의한다.
- 신규 산업용 PC 배포 의존성은 검증한 `ultralytics==8.4.61`,
  `depthai==3.6.1`로 고정한다. 카메라 코드는 기존 개발 장비의 DepthAI 2.30+
  호환도 유지하며, API를 장치를 열지 않는 feature check로 구분한다. 3.6보다
  이른 3.x runtime은 fail-closed한다. v2는
  `Device(pipeline, DeviceInfo, UsbSpeed)`, v3는
  `Device(DeviceInfo, UsbSpeed) -> Pipeline(device)` 순서로 열며, 두 경로 모두
  동일한 MxID·bounded retry·실제 USB 속도 검증 계약을 지킨다.
- 노드 기본값도 안전한 쪽(640x360, 10 FPS, `oak_usb_speed=high`, 교통용 MxID
  핀닝)이다 — 인자를 손으로 붙이는 걸 한 번 잊으면 위성 수·HDOP·RTCM이 전부
  정상으로 보이는 채 RTK FIXED만 안 잡힌다 (CLAUDE.md §6, 2026-08-24).
  차량 launch의 표준 프로필은 1280x720, 10 FPS, `oak_usb_speed=high`,
  RGB-only다. HIGH 요청 후 실제 `getUsbSpeed()`가 HIGH가 아니면 노드는
  fail-closed한다.
- USB2 안전 payload 상한은 36 MB/s로 둔다. 비압축 BGR은 3 B/px, depth는
  2 B/px로 계산하며 1280x720@10 RGB-only는 27.65 MB/s라 허용한다.
  같은 해상도의 RGBD는 46.08 MB/s라 거부하고, depth 진단은
  640x360@10 RGBD(11.52 MB/s)를 사용한다.
- y-only 실차 모드에서는 stereo depth를 장치 단계에서 꺼 RGB 처리량을
  우선한다. depth gate를 사용할 때만 저해상도 프로필로 다시 켜고 y/z 임계값을
  그 프로필에서 다시 보정한다.
- 산업용 PC 기동 전
  `ros2 run stack_traffic stack_traffic_ml_preflight`로 torch/torchvision native
  op와 실제 NMS, Ultralytics, DepthAI API 세대를 확인한다. 같은 호스트에서
  `stack_lane`까지 기동할 때만 `--require-xpu`를 추가해 lane용 Intel XPU를
  검사한다. traffic YOLO가 XPU를 쓴다는 의미는 아니다. 사전점검은 패키지를
  설치·삭제·업데이트하지 않는다.
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
- 통합 실차 launch는 두 OAK-D 동시 초기화 경쟁으로 프로세스가 시작 직후 종료되는
  경우를 위해 `stack_traffic_node`를 2초 간격으로 자동 respawn한다. 출발 전 노드 목록
  확인은 별도로 수행한다.
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
현재 한라대 운영 파일은 `0.60`을 사용하며, 최근 정지 시험에서 에지 쌍 후보가
`y_ratio=0.885~0.978`, `width=45~80%`, `stable=1`, `FINAL_STOP=1`을 만들었다.
이 값도 고정된 카메라 장착 자세와 현장에만 유효하다. 장착 자세·ROI·속도·정지선이
바뀌면 측정 런북으로 다시 측정한다. 과거 `0.98`은 현재 운영값이 아니다.

## 안전 조건

- MGM은 `/perception/traffic_stop` 미수신/시간 초과 0.5초 후 fail-safe 정지한다.
- TRAFFIC 중 `/vehicle/vector`가 0.2초 이상 stale이면 거리 적분을 신뢰하지 않고
  fail-safe 정지한다.
- 카메라·모델·프로세스 장애와 무관하게 물리 E-stop 담당자가 즉시
  제어할 수 있어야 한다.
- dSPACE의 CAN counter watchdog이 실제로 동작하는지 먼저 확인한다.
- 실제 주행은 유효한 E-stop heartbeat와 경로 입력이 있어야 한다.
- `dummy_ref_publisher`와 MGM을 동시에 실행하지 않는다.
- 적색은 LANE/WAYPOINT에서 TRAFFIC으로 전이하며, 초록은 LANE으로 복귀한다.

실행 명령과 터미널 배치는
`src/adas_mgm/RUNBOOK_full_operation_20260904.md`를 따른다.
