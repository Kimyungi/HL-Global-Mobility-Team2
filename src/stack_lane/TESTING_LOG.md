# stack_lane 구현·테스트 정리 (2026-08-07~08)

> 설계 결정 이력의 원본은 [`PROJECT_BRIEF.md`](PROJECT_BRIEF.md)(§1~§15). 이 문서는
> "무엇을 만들었고 무엇을 테스트했으며 결과가 뭐였는지"를 한눈에 보기 위한 요약.

---

## 1. 구현한 것

### 1.1 차선 인식 파이프라인 (`stack_lane/` 패키지)

```
OAK-D Pro 카메라
  → YOLOPv2 추론 (yolopv2_infer.py)
  → BEV 워프 (bev.py) — 호모그래피는 config/homography.json 있으면 실측, 없으면 카메라 스펙 기반 placeholder
  → 슬라이딩 윈도우 + 중심선 추정 (lane_fit.py)
  → lookahead(3m) 지점 {x,y,yaw,curvature} + confidence (lane_path.py)
  → /perception/lane_path 발행 (node.py)
```

| 모듈 | 역할 |
|---|---|
| `yolopv2_infer.py` | YOLOPv2 TorchScript 추론 (원저자 공식 전처리/후처리 이식) |
| `bev.py` | 호모그래피 로드(실측/placeholder 자동 전환), world↔BEV픽셀 변환 |
| `lane_fit.py` | 다중 후보 탐색 → 폭(3.7m) 검증 → 편측 폴백 → 과속방지턱 등 수직 마킹 필터 |
| `lane_path.py` | lookahead 지점 산출 + confidence 스코어링 |
| `debug_draw.py` | 디버그 시각화(오버레이+BEV+ref_point 투영) — node.py와 tools 공용 |
| `logging_utils.py` | CSV 프레임 로거 |
| `node.py` | 실제 ROS2 노드 — 카메라 구동, 파라미터화, `/perception/lane_path` 발행, 선택적 디버그 이미지/CSV 로깅 |

### 1.2 개발 도구 (`tools/`)

- `visualize_yolopv2.py` — YOLOPv2 세그멘테이션만 확인 (초기 검증용)
- `visualize_lane_fit.py` — 전체 파이프라인 라이브 확인 (BEV·슬라이딩윈도우·ref_point 시각화)
- `capture_calibration_frame.py` / `calibrate_homography.py` — 실측 캘리브레이션 도구 (아직 실측 미실행, placeholder로 대체 운영 중)
- `benchmark_pipeline.py` — 전체 파이프라인 프레임당 소요시간 측정

### 1.3 통합 테스트용 스크립트 (`scripts/`)

- `dummy_estop_clear.py` — stack_estop 미연결 상태에서 통합 테스트하기 위한 테스트 전용 더미 (실제 장애물 감지 없음, 물리 E-stop 필수 병행)
- `log_steering_correlation.py` — MGM 출력(target_y/yaw/state)과 dSPACE 실측 조향각(str)을 시간 정렬해 CSV로 남기는 진단 도구

### 1.4 `adas_mgm` 안전 수정

- **lane_path / gps_path 신선도 watchdog 추가** (`mgm_node.cpp`) — estop과 동일한 패턴. 현재 state가 LANE인데 lane_path가 500ms 이상 안 오거나, WAYPOINT인데 gps_path가 500ms 이상 안 오면 자동으로 estop 강제 → v_ref=0. 기존엔 이 watchdog이 없어서 인지 노드가 죽어도 MGM이 마지막 값을 계속 재사용해 주행을 계속하는 실질적 안전 문제가 있었음 (실차 테스트로 재현·확인 후 수정).

---

## 2. 검증 결과 (전부 통과)

| 항목 | 방법 | 결과 |
|---|---|---|
| 편측/양쪽 검출 판별력 | 합성 시나리오 | 양쪽 conf 0.95 > 편측 conf 0.81, 정상 |
| 인접 차로 오선택 방지 | 합성 시나리오(4개 라인) | 3.7m 짝만 정확히 선택 (7.4m 짝 배제) |
| 과속방지턱 등 수직 마킹 오검출 | 합성 시나리오(맞닿는 케이스) | 필터 적용 전 오차 0.084m → 적용 후 0.005m |
| 근거리 사각지대(2.5m) 대응 | 합성 시나리오 | strip_frac 전체 높이로 수정 후 정상 검출 |
| confidence 캘리브레이션 | 실주행 로그 역산 | 정상조명 median 0.71, 삐딱주행도 0.6 이상 96% |
| 전체 파이프라인 레이턴시 | benchmark_pipeline.py | 평균 13.6ms (100ms 예산의 14%), 100ms 초과 0% |
| ROS2 노드 실제 발행 | `ros2 topic hz/echo` | 30Hz 안정 발행, 필드 전부 정상 |
| lane/gps 신선도 watchdog | 인지 노드 중단 시나리오 | lane→waypoint 전이 후에도 v_ref=0 유지 확인 |
| 터미널 종료 시 CAN 송신 중단 | SIGHUP 세션 종료 재현 | 프로세스·CAN 프레임 모두 즉시 정지 확인 |

---

## 3. 실차 CAN 통신 확인 (bridge_dspace)

- **PCAN 자동 연결**: `can-iface@can0.service`가 이미 설치·활성화돼 있어 PCAN 연결 시 자동으로 1Mbps up (추가 조치 불필요)
- **RX(dSPACE→PC)**: `0x200~0x202` 10ms 주기로 정상 수신, 값도 정상 디코딩됨
- **TX(PC→dSPACE)**: `0x101`+`0x100` 정상 송신, counter 증가·v_ref 값 정상 반영 확인
- **왕복 확인**: dummy_ref_publisher(v_ref=0.3)로 실제 차량 속도가 0.3m/s 근처까지 따라 올라가는 것 확인 (종방향 정상)

---

## 4. 실차 주행 테스트 — 카메라 단독

물리 E-stop 대기 상태에서 초저속(v_base=0.15) + 완화된 lane_conf 임계값(exit=0.15, return=0.25)으로 진행.

- **종방향(속도)은 정상 작동** — MGM이 카메라 confidence 기반으로 lane/waypoint 전이하며 v_ref를 정상적으로 냄
- **횡방향(조향)은 전혀 반응 없음** — 차가 저속으로 굴러가긴 했으나 카메라가 인식한 차선 방향으로 꺾이지 않음

---

## 5. 조향 미반영 진단 (핵심 이슈, 미해결)

### 5.1 초기 가설과 기각 과정

`/adas/target_ref`(우리가 보내는 목표)와 CAN `0x202`(dSPACE 실측 조향각 `str`)을 직접 비교하는 방식으로 좁혀갔다. GPS는 실제로 조향에 성공한 전례가 있어(릴리즈 `gps-bags-20260807`, ±28° 정상 반응 확인), GPS 성공 조건을 하나씩 재현하며 우리 조건과 비교했다.

| 가설 | 테스트 방법 | 결과 |
|---|---|---|
| MGM이 waypoint에 갇혀 lane 값이 전달 안 됨 | target_ref에 `state` 필드 추가 로깅 | state는 대부분 lane 유지, 기각 |
| `/vehicle/vector` 피드백 부재가 원인 | 코드 확인(grep) | stack_lane·MGM 둘 다 애초에 구독 안 함 — 설계상 불필요, 기각 |
| y 부호 규약(좌+/우+) 불일치 | GPS 사례 대조 | GPS도 동일 규약으로 이미 성공 사례 있음 + 팀 회의 결정 사항이라 원인이라 해도 이건 "방향" 문제일 뿐 "무반응" 문제와 별개 |
| state=lane 분기 자체가 dSPACE에 미구현 | CAN 직접 발행 테스트(MGM 우회, state 0 vs 1) | 두 경우 다 무반응, 기각 |
| v_ref=0(정지)이라 조향 로직이 트리거 안 됨 | v_ref 0 vs 0.1(실제 이동)로 재테스트 | 무반응 동일, 기각 |
| n_points=1이라 궤적 생성에 부족 | 점 1개 vs 2개로 재테스트 | 무반응 동일, 기각 |
| 발행 주기(20Hz)가 30ms watchdog보다 느려 홀드 모드에 갇힘 | 100Hz(실제 MGM 주기와 동일)로 재테스트 | 무반응 동일, 기각 |

### 5.2 확정된 사실

- **`str`은 우리가 무엇을 보내든(state/v_ref/점개수/주기 어떤 조합이든) 거의 항상 -0.04~-0.07rad 근처의 좁은 범위(±수 도)에 머문다.**
- **`0x202`의 `counter` 필드는 관측한 모든 세션에서 단 한 번도 증가한 적 없다(항상 0).**
- **PC가 CAN에 아무것도 안 보내는 상태(프로세스 전무, TX 프레임 전무)에서도 차량을 센서모드로 전환하면 저속으로 굴러가는 현상이 재현됨** — 즉 이 거동은 PC측 CAN 입력과 무관하게 dSPACE 쪽에서 자체적으로 발생.

### 5.3 결론

CAN 레벨에서 PC가 통제할 수 있는 모든 변수(state, v_ref, 점 개수, 발행 주기)를 GPS 성공 조건에 맞춰 재현했음에도 조향 반응이 재현되지 않았고, PC가 완전히 침묵한 상태에서도 유사한 이상 거동(센서모드 전환 시 자동 구동)이 관측됨. **이는 PC/ROS/CAN 브리지 쪽 코드 문제가 아니라 dSPACE Vehicle MGM 모델 쪽의 상태(잔류 상태·리셋 필요·GPS 테스트 이후 설정 변경 등)로 원인이 좁혀짐.** PC 쪽에서 추가로 진단·수정 가능한 범위를 벗어난 것으로 판단, 손상민과의 직접 확인이 필요.

### 5.4 다음 단계 (팀 논의 필요 항목)

1. GPS 성공 시점(08/03~08/06) 이후 dSPACE 모델/설정이 바뀌었는지 확인
2. `0x202` counter가 왜 항상 0인지 — 이 프레임이 실시간 계산 결과인지 정적값인지부터 확인
3. "센서모드" 전환 시 CAN 입력 없이도 기본 구동이 나가는 것이 의도된 트림/기본값인지, 리셋 필요한 잔류 상태인지 확인
4. `str` 부호 규약(PROTOCOL.md는 좌+ 가정, 팀 회의 결정은 반대라는 언급 있었음) — 위 문제 해결 후 별도로 재확인 필요

---

## 6. 아직 안 한 것

- [ ] 캘리브레이션 실측 (차량 사용 가능해지면 `tools/calibrate_homography.py`로 진행 → `config/homography.json` 교체)
- [ ] 밝은 환경에서 곡선 구간 스트레스 테스트 (지금까지는 야간 테스트만 진행됨)
- [ ] 조향 문제 해결 후 실제 카메라 기반 차선 추종 주행 재검증
