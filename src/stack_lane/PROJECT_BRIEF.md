# stack_lane 작업 브리핑

> 담당: 이현준 · 산출물: 차선 단독 주행 (마일스톤 8/2)
> 이 문서는 전체 프로젝트 구조를 파악하고 stack_lane이 구현해야 할 범위를 정의하기 위한 작업 노트.
> 프로젝트 전체 아키텍처의 기준 문서는 [`CLAUDE.md`](../../CLAUDE.md), stack_lane 공식 계약은 [`REQUIREMENTS.md`](REQUIREMENTS.md).

---

## 1. 프로젝트 개요

WHEELTEC 플랫폼 기반 자율주행 시스템. 6개 시나리오를 6명이 스택 단위로 분담:
차선 주행 / GPS(waypoint) 주행 / 장애물 회피 / 신호등·정지선 정지 / 돌발 장애물 긴급 정지 / 라이다 주차.

**계층 구조 (Signal processing → Decision → Control → Actuation):**

| 계층 | 이름 | 위치 | 주기 | 담당 |
|---|---|---|---|---|
| Signal processing | ADAS application (각 stack) | PC (Ubuntu 22.04 + ROS 2 Humble) | 비동기 (센서별) | 각자 |
| Decision | ADAS MGM | PC | 10ms 고정 | 김윤기(ROS2 레퍼런스) / 김재민(Simulink MBD) |
| Control | Vehicle MGM | dSPACE | 10ms | 손상민 |
| Actuation | 하위 제어 | dSPACE | 5ms 독립 태스크 | — |

- 통신: PC ↔ dSPACE는 **CAN (classic 2.0A, 1Mbps, PCAN 어댑터)**, 10ms 주기.
- **stack_lane은 Signal processing 계층 하나만 담당.** Decision(모드 전환·정지 판단)은 절대 여기서 하지 않음 — MGM 스테이트 머신의 고유 영역.

## 2. 팀 구성 및 각 스택 역할

| 이름 | 스택 | 역할 |
|---|---|---|
| 김윤기 (팀장) | stack_gps | GPS·IMU 융합, RTK, waypoint ref |
| **이현준 (나)** | **stack_lane** | **카메라 차선 검출(YOLO) → 차선 ref + 정지선 검출** |
| 손상민 | stack_parking + MPC/Vehicle MGM | 라이다 주차, dSPACE 제어 |
| 이기돈 | stack_avoid | 장애물 인지, 회피 가능 판정(TTC), 회피 경로 |
| 김재민 | stack_traffic + 하위제어 | 신호등·정지선 결합 정지 요구 |
| 박찬미 | stack_estop | 돌발 장애물 긴급 정지 |

모든 스택은 `fma_interfaces`에 정의된 메시지로만 MGM과 통신 (직접 결합 금지). 스켈레톤 노드는 이미 중립값을 퍼블리시하므로 전체 파이프라인(스택 → MGM → 브리지 → dSPACE)이 지금도 관통 실행 가능.

## 3. stack_lane이 구현해야 하는 것 (핵심)

### 출력 ① `/perception/lane_path` (`fma_interfaces/LanePath`)

```
std_msgs/Header header
RefPoint[] points      # {x, y, yaw, curvature} — vehicle frame, 생성 시점 차량 = (0,0,0)
float32 confidence     # 0.0~1.0
```

- **카메라 주기(100ms, OAK-D Pro)마다 발행.**
- **points 개수는 정확히 1개** (2026-07-29 팀 합의). dSPACE 측 quintic 궤적 생성이 이 점 하나로 MPC 지평(200ms/N=20)을 채우는 구조라, 여러 점을 낼 필요도, 내서도 안 됨.
- 목표점은 **차량 전방 ~1m 부근** 권장.
- `confidence`는 MGM의 lane↔waypoint 히스테리시스 전이 판정의 **재료**일 뿐 — 전이 여부 자체는 절대 여기서 판단하지 않음. 차선을 놓쳤을 때(가림, 급커브, 저조도 등) confidence가 실제로 떨어지는 것이 검증 포인트.

### 출력 ② `/perception/stopline` (`fma_interfaces/StopLine`)

```
std_msgs/Header header
bool detected
float32 distance   # [m] vehicle frame 전방 x
```

- 수신자는 **MGM이 아니라 stack_traffic(김재민)**. OAK-D 장착 각도상 신호등이 화각에 안 들어와서, stack_lane이 정지선 거리만 검출해 넘기고 신호등 적색 판정은 stack_traffic이 별도 웹캠으로 해서 결합.
- **미검출 프레임에도 매 주기 `detected=false`로 발행** — stack_traffic의 stale 판정이 이 발행 주기에 의존하므로 발행을 멈추면 안 됨.
- `header.stamp` 필수.
- **QoS: 기본 Reliable로 발행** (SensorDataQoS/Best Effort 금지 — 수신 측이 기본 QoS라 Best Effort로 내면 연결 자체가 안 됨).
- 정지 트리거 임계는 stack_traffic 쪽 0.5m — 따라서 **0.5m 이내 근거리에서 시야 이탈 직전까지 distance가 끊기지 않고 유지**되는지가 검증 핵심.

### 명시적 금지 사항

- v_ref 산출 금지
- 정지 판단 금지 (정지선 거리 "보고"만, 정지 여부는 stack_traffic → MGM)
- 모드 판단 금지 (lane↔waypoint 전이는 MGM만)

### 입력

- 카메라: **OAK-D Pro**, 100ms 주기.
- 차선 검출 방식: **YOLO 기반** (팀 계약상 지정, 세부 모델·후처리는 자유).

## 4. 개발 시 지켜야 할 공통 규칙 (CLAUDE.md 발췌)

1. 판단 로직(모드 전환/정지 결정/우선권)은 MGM 스테이트 머신에만 존재. stack_lane은 순수 인지(perception) 노드.
2. MGM 10ms 루프는 인지 노드와 **별도 프로세스**. stack_lane 안에서 YOLO 추론이 얼마나 걸리든 MGM 루프를 블로킹하지 않음 (MGM은 pull 방식으로 최신 스냅샷만 읽음). → 노드 설계 시 이 비동기성을 깨지 않게 주의.
3. 모든 경로 소스(차선/GPS/회피/주차)는 동일 `{x, y, yaw, curvature}` vehicle frame 포맷 — MGM/dSPACE 무수정 원칙 유지.
4. stack_lane → stack_traffic 전달은 MGM을 경유하지 않는 유일한 스택 간 직접 연결 (2026-07-30 결정). 그 외 스택 간 직접 통신 없음.

## 5. 현재 상태 (스켈레톤)

[`stack_lane/node.py`](stack_lane/node.py)는 100ms 타이머로 `LanePath`를 발행하되 `confidence=0.0`, `points`는 비워둔 placeholder 상태. `/perception/stopline` 발행 로직은 **아직 스켈레톤에도 없음** — REQUIREMENTS.md엔 명시돼 있지만 코드로는 구현 전. 실제로 채워야 할 것:

- [ ] OAK-D Pro 카메라 입력 파이프라인 연결
- [ ] YOLO 기반 차선 검출 → 단일 목표점(x, y, yaw, curvature) 산출 로직
- [ ] confidence 산출 로직 (검출 품질과 실제 연동되게)
- [ ] `/perception/lane_path` 100ms 발행 (현재 skeleton 확장)
- [ ] 정지선 검출 로직 + `/perception/stopline` 발행 노드/로직 신규 구현 (Reliable QoS)
- [ ] 검증: 차선 신뢰도 저하 시나리오, 정지선 0.5m 이내 근거리 시야 이탈 시나리오

## 6. 확정된 정보 (2026-08-07)

### 하드웨어 / 환경
- 카메라: **OAK-D Pro**, 지면 기준 **높이 140cm**, **pitch를 줘서 장착** (근거리 사각 줄이는 목적) → **최소 가시거리 ≈ 2.5m**.
- 테스트 장소: **실제 대형 자동차 운전면허 연습장** — 실도로 환경에서 열리는 대회 대비. 시뮬레이션이 아니라 실차 실주행이 최종 목표.

### 모델
- **YOLOPv2** 사용 확정 (drivable area + lane segmentation + object detection 동시 추론하는 panoptic 모델).
- **모델이 무거워 REQUIREMENTS.md의 100ms(카메라 주기) 스펙을 못 지킬 가능성이 큼** — 실측 후 실제 발행 주기를 확정하고 팀에 공유 필요.

### 경로 생성 전략 (계획)
- 차선을 **20~30개 점**으로 우선 검출 → 점들을 이어 경로(곡선) 생성 → **lookahead 거리**를 정하고 그 거리에 해당하는 점 하나를 최종 목표점으로 산출.
- 계약상 출력은 여전히 **점 1개** — 내부적으로만 다점 검출 후 1점으로 축약하는 구조이므로 `LanePath.points` 계약과 충돌 없음.
- **lookahead 거리: 3m로 확정.** 카메라 최소 가시거리(2.5m) 이상이라 검출 가능 범위 안에 들어오고, dSPACE quintic 궤적 생성 쪽도 문제없을 것으로 판단 (2026-08-07, 사용자 확인 — 필요 시 손상민/김윤기와 실주행 데이터로 재검증).

### 목표
- **confidence를 0.7 이상으로 안정적으로 유지**하는 것이 1차 목표.
- MGM 히스테리시스 임계값 확인 결과 `lane_conf_exit=0.4`, `lane_conf_return=0.6` (`src/adas_mgm/config/params.yaml`) — 0.7이면 두 임계 모두 여유 있게 상회. 다만 실제 필요한 건 "평균 0.7"이 아니라 "주행 중 순간적으로도 0.4 밑으로 안 떨어지는" 안정성 쪽에 더 가까움 (히스테리시스가 순간값 기준으로 카운트되므로).

### 정지선 (`/perception/stopline`)
- 사용자는 이 부분을 **작업하지 않음 — 팀 내 다른 사람이 담당 예정**.
- **주의:** 현재 `REQUIREMENTS.md`는 이 출력을 stack_lane(이현준) 책임으로 명시하고 있어 실제 배정과 문서가 어긋난 상태. 누가 담당하는지, 어떤 노드/토픽 이름으로 낼지 팀과 확정한 뒤 `REQUIREMENTS.md`를 갱신해두는 게 안전 (안 그러면 나중에 MGM/stack_traffic 쪽에서 "이현준이 낸다"고 알고 기다리는 상황 발생 가능).

## 7. 결정된 사항 (2026-08-07)

- **lookahead 거리 = 3m로 확정.** (§6 참조)
- **lane_path staleness watchdog은 지금 안 만듦.** 실주행 테스트하면서 YOLOPv2 연산 주기가 실제로 너무 길어진다고 판단되면 그때 추가하기로 보류. 단, 이 결정은 "테스트 전까지는 MGM이 오래된 목표점을 감지 못한 채로 계속 쓴다"는 리스크를 안고 가는 것이므로, 실차 테스트 시 **연산 주기(프레임 간격)를 실측 로깅**해두면 나중에 이 판단(추가 여부)의 근거가 됨.

### 추론 위치 / 차선 조건 (2026-08-07)
- **YOLOPv2 추론은 호스트 PC(GPU/CPU)에서 진행.** 온보드 VPU(OAK-D blob 변환) 경로는 사용 안 함 — 100ms 초과 시 최적화는 입력 해상도 축소, TensorRT/OpenVINO 변환, 프레임 스킵 등 PC 추론 기준으로 접근.
- **차선: 흰색 + 노란색 병행 사용.** 차선 폭 규격 **3.7m** — 단 실제 코스를 BEV로
  재면 **4.07 ± 0.11m**(2026-08-15, run_0815_181526·182032 both 검출 1900프레임,
  범위 3.65~4.30)이며 `lane_fit.LANE_WIDTH_M`은 **실측값 4.06**을 쓴다. 그 상수는
  BEV 좌표에서 잰 폭과 맞춰야 편측/양측 경로가 서로 일치하기 때문 —
  자세한 근거는 `lane_fit.py`의 LANE_WIDTH_M 주석 참조. 노면을 줄자로 재서
  3.7m가 맞다면 호모그래피 배율이 ~10% 부풀려진 것이므로 재캘리브레이션 대상(미확인).
  - ⚠️ **주의**: YOLOPv2의 lane segmentation head는 기본적으로 **이진(차선/비차선) 마스크**만 출력 — 색상(흰/노랑) 구분 기능이 내장돼 있지 않음. 흰색·노란색을 구분해야 하는 용도(예: 중앙선 vs 가장자리선 구별)가 있다면, 세그멘테이션된 차선 픽셀 위치의 OAK-D RGB 원본에서 **HSV 색상 필터링 등 별도 후처리**가 필요. 단순히 "차선이 어디 있는지"만 필요하면(현재 목표점 1개 산출 목적) 색상 구분 없이도 동작은 가능 — 색상 구분이 실제로 왜 필요한지(차로 유지 로직에 색상이 들어가는지) 명확히 하면 이 부분 설계가 갈림.

### 캘리브레이션 방법 (계획)
1. **Intrinsic**: OAK-D Pro 공장 캘리브레이션(`device.readCalibration()`)을 우선 사용, 필요시 체커보드로 OpenCV 재보정.
2. **Extrinsic/지면 매핑**: 해석적(pitch각 삼각법) 대신 **호모그래피 실측 캘리브레이션** 채택 — 실제 노면에 실좌표를 아는 기준점 격자(전방 1~4m × 좌우 ±2m 권장, lookahead 3m·차선폭 3.7m 커버)를 놓고 촬영 → `cv2.findHomography()`로 H 산출 → 런타임엔 차선 픽셀을 H로 vehicle frame (x,y) 변환.
3. IPM 적용 전 렌즈 왜곡 보정(undistort) 필수.
4. 캘리브레이션 기준점의 실측 원점 = vehicle frame 원점과 반드시 일치시켜야 함 (아래 open item과 연결).

### vehicle frame 원점 (2026-08-07)
- **카메라 위치를 그대로 원점으로 사용.** `stack_gps`도 GPS 안테나(fix) 위치를 별도 lever-arm 보정 없이 base_link로 쓰고 있어 (`src/stack_gps/stack_gps/node.py:284` map→base_link TF) — 기존 팀 관행과 일치. 차량 중심 대비 카메라의 대략적인 실제 장착 오프셋은 디버깅용으로 기록해두면 좋음.

### 차선 색상 구분 (2026-08-07)
- **구분 안 함.** YOLOPv2 이진(차선/비차선) 마스크 그대로 사용 — 설계 단순화.

### 곡선 피팅 / BEV vs 원본 (2026-08-07 — 방향 결정)
- **BEV 기반 다항식 피팅 + 슬라이딩 윈도우 채택.** 원근(원본) 이미지에서는 곡선 구간에서 차선이 소실점으로 수렴해 다항식 피팅이 왜곡되지만, BEV(탑다운)에서는 차선이 대략 평행/완만한 곡선으로 펴져서 슬라이딩 윈도우+다항식이 안정적으로 동작함 (슬라이딩 윈도우 기법 자체가 BEV 전제로 설계된 방법).
- §7 캘리브레이션에서 만든 **호모그래피 H를 그대로 재사용** — 새 변환을 만들 필요 없음. 두 방식 중 택1:
  - (a) 세그멘테이션 마스크 전체를 `cv2.warpPerspective(H)`로 BEV 이미지 변환 후 슬라이딩 윈도우 → 결과 픽셀을 H로 다시 미터 단위 변환
  - (b) 이미지 워핑 생략, 세그멘테이션된 차선 픽셀 (u,v)를 바로 H로 실좌표(x,y[m])로 변환 후 x구간별 비닝+다항식 피팅 — 워핑 연산이 없어 더 가벼움 (YOLOPv2가 이미 무거운 상황엔 유리)
- lookahead(3m) 지점에서 `yaw = atan(dy/dx)`, `curvature = y'' / (1+y'^2)^1.5` (2차 다항식이면 y''는 상수)로 RefPoint 4개 필드 모두 산출 가능.

### 실제 트랙 조건 (2026-08-07)
- **한국 운전면허 시험 코스 표준 요소(S자, ㄱ자/굴절, 방향전환 등) 전부 포함** — 곡률 범위는 국내 면허시험장 표준으로 커버됨, 별도 조사 불필요.
- ⚠️ **신규 리스크**: **ㄱ자(직각 코너) 구간에서 카메라가 차선을 화각(FOV) 밖으로 놓칠 가능성.** 140cm 높이 + pitch로 근거리(2.5m 이내)를 죽인 상태라, 코너 진입 시 안쪽 차선이 프레임 밖으로 빠르게 벗어날 수 있음. OAK-D Pro가 표준 FOV/와이드(W) 버전인지에 따라 영향도가 다름 — **실제 코너 구간에서 confidence 급락 여부를 현장 테스트로 반드시 확인**.

## 8. 남은 확인 항목

1. **OAK-D Pro 모델(표준 vs Wide-FOV)** — 위 ㄱ자 코너 리스크 판단에 필요.
2. **confidence 산출 방식 구체화** — BEV 슬라이딩 윈도우의 인라이어 픽셀 수/피팅 잔차 등을 기반으로 할 예정이나, 0.7 목표에 맞춘 구체적 수식은 실측 데이터 확보 후 튜닝 필요.

## 9. 진행 상황 — YOLOPv2 시각화 검증 (2026-08-07)

**참고**: `FMA_ws_ROS_TRANSFER_READY`에 이미 훨씬 정교한 stack_lane 구현체(motion compensation, drift debt, hard/soft reset 등)가 로컬 전용 브랜치(`codex/stack-lane-ros-vehicle`, origin에 미push)로 존재하는 걸 발견했으나, 이번 세션은 여기서 새로 설계한 방향(3m lookahead, 호모그래피+BEV)으로 처음부터 코드를 작성하기로 결정 (모델 가중치 `yolopv2.pt`만 재사용).

- 도구 스크립트: [`tools/visualize_yolopv2.py`](tools/visualize_yolopv2.py) — ROS/MGM과 무관한 독립 개발 도구. `--source`로 `oak`(OAK-D Pro 라이브)/웹캠 인덱스/이미지·동영상 경로를 모두 받음. 전처리·후처리(letterbox, seg/ll 크롭 `[12:372]`, bilinear 2x)는 YOLOPv2 원저자 공식 `demo.py`/`utils.py` 로직 그대로 이식.
- 가중치: `models/yolopv2.pt` (156MB, TorchScript) — `.gitignore`에 `*.pt` 추가해 실수로 커밋되지 않게 함.
- **정적 샘플 이미지로 1차 검증 완료**: 주행가능영역(초록)·차선(빨강) 세그멘테이션이 실제 이미지에서 정확하게 겹쳐짐 (`tools/visualize_yolopv2.py --source <이미지> --headless --save <출력경로>`).
- **추론 속도 벤치마크 (RTX 4060 Laptop, FP16, 640×384 net 입력)**: 워밍업 후 정상 상태 **median 11.3ms/frame (~89 FPS)**. 최초 1회 콜드스타트만 683ms — 이는 CUDA/cuDNN 초기화 오버헤드이며 이후 프레임엔 해당 없음.
  - ⚠️ **이전 가정 정정**: "YOLOPv2가 무거워서 100ms 스펙을 못 지킬 가능성이 크다"고 예상했었는데, 최소한 이 개발 PC 기준으로는 **모델 추론 자체는 병목이 아님** (100ms 예산 중 순수 추론은 11ms만 사용, ~89ms 여유). 카메라 캡처·전처리·경로 피팅·ROS 오버헤드를 다 더해도 100ms 안에 들어올 가능성이 높아짐.
  - **단, 실차 탑재 PC 사양이 이 개발 PC(RTX 4060)와 다르면 그 PC에서 반드시 재측정 필요** — 이 벤치마크는 개발 노트북 기준.
- **OAK-D Pro 라이브 연결 확인 완료** (2026-08-07): USB에서 Movidius MyriadX 정상 인식, `--source oak`로 실시간 추론 성공 (9.9~11ms/frame, 벤치마크와 일치). 실내 벽을 비춘 첫 테스트라 세그멘테이션은 대부분 비어 있는 게 정상 — 카메라 기동 직후 노출 적응 전 프레임은 오검출 가능성 있음(배포 시 워밍업 프레임 스킵 필요).

## 10. 캘리브레이션 도구 (2026-08-07)

차량을 당장 쓰기 어려운 상황이라, 실측 캘리브레이션은 나중에 하기로 하고 **"파일만 교체하면 되는 파라미터"**로 설계를 분리했다 (아래 §11과 맞물림).

- [`tools/capture_calibration_frame.py`](tools/capture_calibration_frame.py) — OAK-D에서 캘리브레이션용 정지 프레임 저장 (노출 적응 위해 초기 프레임 자동 스킵).
- [`tools/calibrate_homography.py`](tools/calibrate_homography.py) — 사진 위에서 기준점 클릭 → 터미널에 실측 (x,y)[m] 입력 → `cv2.findHomography`로 H 계산 → FIT/VALIDATION 재투영 오차 리포트 → `config/homography.json` 저장 + BEV 미리보기 이미지 생성.
  - 좌표계: 원점(0,0) = 카메라 광학중심의 지면 투영점, x=전방(+), y=좌측(+) (§6 원점 결정과 일치).
  - **합성 데이터로 계산 로직 검증 완료** — 알려진 H로 생성한 가상 점들을 넣었을 때 원래 H가 오차 0으로 복원됨.
  - 아직 실측 미실행 (차량 사용 어려운 상황) — §7에서 정한 격자 배치(전방 2.5/3.5/5.0m × 좌우 ±1.85/±0.9/0m)로 나중에 진행.

## 11. 차선 인식/경로 추정 파이프라인 구현 (2026-08-07)

차량 없이도 진행 가능하도록 **캘리브레이션(호모그래피)을 스왑 가능한 파라미터로 분리**하고, 차선 피팅 알고리즘을 먼저 구현했다. `config/homography.json`이 없으면 카메라 스펙 기반 analytic placeholder(높이 1.40m, pitch 12° 가정 — 실측 전 추정치)를 자동 사용하고, 실측 파일이 생기면 코드 변경 없이 자동 전환된다.

### 모듈 구성 (`stack_lane/` 패키지, ROS 무의존 순수 함수)
- `yolopv2_infer.py` — YOLOPv2 추론 (기존 `tools/visualize_yolopv2.py`에서 분리해 재사용 가능하게 리팩터링).
- `bev.py` — `load_homography()`(실측 있으면 로드, 없으면 placeholder), `BevGrid`(world↔BEV픽셀 변환, px/m 스케일은 우리가 정하는 상수라 H 교체와 무관하게 고정), `warp_to_bev()`.
- `lane_fit.py` — 지난 대화에서 정한 설계 그대로 구현:
  - 근거리 히스토그램에서 임계값 넘는 **모든** 봉우리를 후보로 수집 (top-2만 보지 않음)
  - 중심(y=0) 기준 좌/우로 나눠 **가장 안쪽 후보부터** 차선폭(3.7m ± 허용오차) 검증하며 조합 탐색
  - 검증 통과 조합 없으면 편측(중심에 가장 가까운 후보) 폴백, 차선폭 절반(1.85m) 오프셋으로 중심선 추정
- `lane_path.py` — lookahead(기본 3m) 지점에서 `yaw = atan(dy/dx)`, `curvature = y''/(1+y'^2)^1.5` 산출 + confidence(윈도우 검출률 × 피팅 잔차 × 폭 일치도, 편측이면 페널티 0.7배).

### 검증
- **합성 데이터 3시나리오 전부 통과** (알려진 곡선을 BEV에 직접 그려서 검증, `python3 -c`로 실행):
  1. 양쪽 검출 — 추정 y/yaw/curvature가 실제값과 오차 0.05 이내 일치
  2. 편측(좌측)만 검출 — `left_only` 모드로 정확히 폴백, confidence가 양쪽 검출 대비 자동으로 낮음(0.63 vs 0.90)
  3. 인접 차로 라인까지 4개가 잡힌 상황 — 폭 검증으로 내 차선(3.7m 짝)만 정확히 골라내고 옆 차로(7.4m 짝)는 무시함
- **실제 OAK-D 라이브로 크래시 테스트 통과**: `tools/visualize_lane_fit.py --source oak` 20프레임 무오류 실행, 11.1ms/frame. 실내 비-도로 장면에서 오검출 없이 `mode=none, conf=0.00` 정상 출력.

### CSV 디버그 로깅 (2026-08-07)
- [`stack_lane/logging_utils.py`](stack_lane/logging_utils.py) — `CsvFrameLogger`. `visualize_lane_fit.py --log-csv <경로>`로 활성화. 매 프레임 mode/confidence/x·y·yaw·curvature/좌우 후보 개수/hit_ratio/잔차/폭 등을 기록 — 스크린샷으로 추측하는 대신 숫자로 바로 원인 확인 가능.

### 실차선 라이브 테스트 1차 결과 및 버그 수정 (2026-08-07)
1967프레임 실주행(삐딱한 주행 포함) 로그 분석 결과 두 가지 이슈 발견·수정:

1. **근거리 탐색 범위 버그**: `fit_lane()`이 BEV 하단 25%(0~1.5m)에서만 후보를 찾았는데, 이 카메라 최소 가시거리가 2.5m라 그 구간엔 원래 아무 것도 안 보임 → 후보 0개 → `mode=none` 오판정. **`strip_frac` 기본값을 0.25→1.0(전체 높이)으로 변경**해 사각지대 위치를 몰라도 항상 탐지되게 수정. 합성 시나리오(0~2.5m 빈 마스크)로 회귀 검증 완료.
2. **confidence가 MGM 복귀 임계값(0.6)을 못 넘는 문제**: 실주행에서 편측(right_only) 검출이 78.6%로 대부분이었는데, `single_side_penalty=0.7`이 confidence를 최대 0.59로 캡해서 `lane_conf_return=0.6`(adas_mgm/config/params.yaml)을 절대 못 넘음 — 즉 한쪽 차선만 보이는 구간에서 waypoint→lane 복귀가 영영 안 되는 실질적 버그였음. 실제 로그의 편측 검출 품질(hit_ratio~0.48, residual~2.5cm — 노이즈가 크지 않았음)을 역산해 **`single_side_penalty`를 0.7→0.85로 조정** — 실제 로그 재계산 기준 편측 프레임의 99.9%가 0.6 이상으로 개선, 양쪽 검출과의 판별력(0.90 vs 0.77)은 유지.
   - ⚠️ **주의**: `lane_conf_return`/`lane_conf_exit`은 adas_mgm 쪽 설정값(김윤기 담당)이라 stack_lane이 일방적으로 못 바꿈. 이번 조정은 stack_lane의 confidence 계산을 그 값에 맞춘 것 — 두 값은 서로 맞물려 있으므로 **MGM 쪽 임계값이 바뀌면 이 penalty도 재검토 필요** (README/PR에 남겨서 팀과 공유 권장).
   - 편측 좌/우 오프셋(1.85m 평행이동) 자체의 근사 오차도 실제 로그 기준(최대 삐딱함 16.1도)으로 검증: y오차 최대 7.3cm, yaw는 근사와 무관하게 정확 — 수정 불필요로 판단.
- **아직 미검증**: 왼쪽 후보가 왜 적었는지(78.6%가 right_only)는 이번엔 "삐딱한 주행" 때문으로 확인됨 — 정상 주행(차선 중앙 유지) 시 both 비율이 어떻게 나오는지는 재테스트 필요.

## 12. 정상 주행 자세 재테스트 + confidence 파라미터 추가 튜닝 (2026-08-07)

정상 자세(차선 중앙 유지)로 야간 실주행 2455프레임 로그(`lane_debug_normal.csv`) 확보. 시간순으로 프레임을 5구간(both 안정 → both/right 흔들림 → right_only 안정 → right/none 흔들림 → 거의 none)으로 나눠 분석한 결과, **어두워지면서 왼쪽부터 순서대로 사라지고 나중엔 오른쪽까지 소실되는 단조로운 저하 패턴**이 정확히 나타남 (n_left_candidates가 구간별로 1→흔들림→0으로 깔끔히 전환) — 사용자가 말한 "어두워서 왼쪽이 안 잡힘"이 로그로 뒷받침됨. confidence도 이 저하를 정직하게 따라감(구간별 mean 0.53→0.67→0.49→0.002) — 버그가 아니라 의도된 동작.

**추가 발견**: 가장 잘 보이는 초반 구간(both 99.9%)에서도 confidence≥0.7 비율이 1.8%뿐이었음. 성분 분해 결과 `hit_score`(0.90~0.95)는 이미 좋은데 `resid_score`(0.69~0.77)·`width_score`(0.79)가 병목 — 근데 실측 잔차 자체(3.5~4.6cm)는 실제로 정밀했고, 폭 편향(+0.13m)도 §9의 placeholder 호모그래피 편향 때문이라 실제 품질 저하가 아니었음. `residual_tolerance_m`(0.15→0.30), `width_tolerance_m`(0.6→1.0, confidence 전용 — `fit_lane()`의 폭 검증 하드게이트와는 별개 파라미터라 인접 차로 판별력엔 영향 없음)로 완화.

**최종 검증**:
- 합성 회귀 테스트 전부 통과 (양쪽 0.952 > 편측 0.809, 인접 차로 4개 중 width_m=3.70으로 정확히 내 차선 선택)
- 실로그 재계산: 정상주행 잘보이는 구간 mean=0.694/median=0.710(0.7 목표 근접), 정상주행 전체(어두워지는 구간 포함) mean=0.634, 삐딱주행 전체 mean=0.702·0.6이상 96.3%

### 다음 단계
- [ ] 곡선 구간 스트레스 테스트 — `margin_px=60`이 실제 곡률에서 슬라이딩 윈도우가 선을 놓치지 않을 만큼인지 확인
- [ ] §10 캘리브레이션 실측 진행 → `config/homography.json` 교체 (both 모드 width_m 편향 개선 기대 — 현재 placeholder 기준 +0.13~0.18m)
- [ ] 위 항목들 안정화되면 "코드 완성"으로 보고 ROS2 노드(`stack_lane/node.py`)에 배선해서 `/perception/lane_path` 발행

## 13. 과속방지턱 오검출 수정 + ref_point 원본 화면 투영 (2026-08-07)

곡선 스트레스 테스트를 야간에 진행하다가(눈으로 곡선 자체는 확인 못 함) 차선과 과속방지턱이 만나는 지점에서 경로가 프레임마다 크게 흔들리는 문제 발견. 원인: §11 근거리 사각지대 수정(`strip_frac`을 전체 높이로 확장)의 부작용으로, 진행방향에 거의 수직인 과속방지턱 줄무늬까지 후보 히스토그램에 잡히기 시작함.

**2단계 방어로 수정**:
1. [`filter_lane_like_components()`](stack_lane/lane_fit.py) — 연결요소(connected component) 단위로 bounding box가 "좌우로 넓고 전방으로 짧은" 것을 1차로 제거. 단, 마킹이 실제 차선과 마스크 상에서 맞닿아 있으면 하나의 연결요소로 합쳐져 이 필터를 통과하는 한계가 있음(합성 테스트로 확인).
2. **`_climb_windows()`의 `max_window_spread_m=0.5` 윈도우 폭 필터** — 실제 결정적인 수정. 슬라이딩 윈도우 하나가 주운 픽셀의 좌우 폭이 0.5m를 넘으면(진짜 차선 폭보다 훨씬 넓음 = 수직 마킹이 오염시킨 것으로 판단) 그 윈도우를 통째로 버림(중심 갱신·점 누적 모두 스킵). 맞닿는 케이스를 합성으로 재현해 검증: lookahead y 오차가 0.084m → **0.005m**로 개선.
3. 회귀 테스트(양쪽/편측/인접차로 4개) 전부 그대로 통과 — 기존 동작 안 깨짐.

**ref_point 원본 화면 투영**: `H_inv = np.linalg.inv(H)`로 lookahead 지점(x,y)을 원본 카메라 픽셀로 역변환해 `visualize_lane_fit.py` 왼쪽 패널에 초록 별로 표시. 실제 카메라 화면 위에 "지금 향하려는 지점이 여기다"를 직접 볼 수 있게 됨. 라이브 스모크 테스트로 크래시 없음, 시각적으로도 합리적인 위치에 찍히는 것 확인.

- 실제 과속방지턱 지점 라이브 재테스트: BEV 패널에서 과속방지턱 관련 오염 없이 깨끗하게 나오는 것과 원본 화면 ref_point 투영 모두 사용자가 육안으로 확인·승인함.

## 14. 전체 파이프라인 레이턴시 벤치마크 (2026-08-07)

곡선 스트레스 테스트(밝을 때까지 대기 필요)와 별개로, 카메라 밝기와 무관한 작업부터 진행. [`tools/benchmark_pipeline.py`](tools/benchmark_pipeline.py) 신규 작성 — YOLOPv2 추론뿐 아니라 BEV 워프·필터·슬라이딩 윈도우·다항식 피팅까지 포함한 프레임당 총 소요시간 측정.

- **라이브 카메라(150프레임, right_only 검출 지속)**: infer mean=11.06ms, lane_path(BEV+필터+윈도우) mean=2.53ms, **total mean=13.59ms, max=14.68ms**
- **저장 영상(실제 검출 있던 영상, 13프레임)**: total mean=13.91ms, max=14.74ms
- **100ms 예산 초과 프레임: 0%** — §9에서 "모델이 무거워 100ms 못 지킬 것"이라 예상했던 우려가 BEV/슬라이딩 윈도우까지 다 붙인 지금도 기우로 확인됨. 카메라 캡처·ROS 발행 오버헤드를 더해도 여유 충분(~85ms 여유).

## 15. ROS2 노드 배선 완료 (2026-08-07)

`stack_lane/node.py`를 스켈레톤에서 실제 파이프라인으로 교체. OAK-D 카메라 부팅(depthai) → YOLOPv2 추론 → BEV/슬라이딩 윈도우(`estimate_lane_path`) → `/perception/lane_path` 발행까지 전 과정 배선.

- **파라미터화**: `weights`, `device`, `img_size`, `lookahead_m`, `homography_path`(기본값 `config/homography.json` — 실측 파일 있으면 자동 사용, 없으면 placeholder), `camera_fps`(기본 30 — REQUIREMENTS.md가 가정한 100ms보다 빠르지만 MGM이 항상 최신 스냅샷만 pull하므로 문제 없음), `warmup_frames`(기본 30, 노출 적응 대기).
- **모델 워밍업**: `__init__`에서 더미 텐서로 1회 추론해 최초 콜드스타트 지연(~680ms)을 노드 준비 단계에서 흡수 — 실제 발행 프레임이 느려지지 않게 함.
- **카메라는 `tryGet()`(non-blocking)으로 폴링** — 새 프레임 없으면 그 틱은 스킵, 있으면 처리+발행. `/perception/stopline`은 발행하지 않음(§6 — 정지선은 팀 내 다른 담당자에게 재배정됨).

**실제 빌드+실행 검증**:
- `colcon build --packages-select fma_interfaces stack_lane` 성공
- `ros2 run stack_lane stack_lane_node` 실행 → 카메라 부팅~노드 준비 약 3초
- `ros2 topic hz /perception/lane_path`: **평균 30Hz 안정적으로 발행** 확인
- `ros2 topic echo --once` 실제 메시지 확인: `x=3.0, y=1.008, yaw=0.017rad, curvature=0.009, confidence=0.687` — 전 필드 정상

⚠️ **운영 주의점 발견**: 백그라운드에서 `kill`(단일 PID)로 종료 시 depthai 장치를 잡은 프로세스가 안 죽고 남아 다음 실행이 `X_LINK_DEVICE_ALREADY_IN_USE`로 실패하는 경우 있었음. 실제 터미널에서 Ctrl+C(포그라운드 프로세스 그룹 전체에 신호)로 끄면 문제없을 가능성이 높지만, 배포 전 확인 필요 — 안 죽으면 `pkill -9 -f stack_lane_node`로 정리.

### 다음 단계
- [ ] 실제 터미널에서 Ctrl+C 종료가 depthai 장치를 깨끗이 해제하는지 확인 (스크립트 kill과 다를 수 있음)
- [ ] 곡선 구간 스트레스 테스트 — 밝을 때 재시도 (이번엔 너무 어두워서 눈으로 곡선 검증 자체가 안 됐음)
- [ ] §10 캘리브레이션 실측 진행 → `config/homography.json` 교체
