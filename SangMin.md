# CLAUDE.md — 자율주행 시스템 프로젝트 컨텍스트

> 이 파일은 팀의 아키텍처 설계 결론을 담는다. Claude Code는 모든 세션에서 이 문서를 프로젝트의 기준으로 삼을 것.
> 설계 변경은 반드시 이 문서를 갱신한 뒤 코드에 반영한다.

## 1. 프로젝트 개요

WHEELTEC 플랫폼 기반 자율주행 시스템. 시나리오: 차선 주행, GPS(waypoint) 주행, 장애물 회피, 신호등/정지선 정지, 돌발 장애물 긴급 정지, 라이다 주차.

- 상위: 산업용 PC, **Ubuntu 22.04 + ROS 2 Humble** — 인지(Signal processing) + 판단(Decision)
- 하위: dSPACE — 제어(Control) + 구동(Actuation)
- 통신: Ethernet (UDP), 10ms 주기

## 2. 아키텍처 (v1 — 현재 기준)

계층 모델: **Signal processing → Decision → Control → Actuation**

| 계층 | 이름 | 위치 | 주기 | 내용 |
|---|---|---|---|---|
| Signal processing | ADAS application | PC | 비동기 (각 센서 주기) | 차선 검출(camera 100ms), 신호등·정지선 인식(camera), GPS·IMU 융합(위치·헤딩), 장애물 인지·주차 로컬맵(LiDAR 100ms) |
| Decision | ADAS MGM | PC | **10ms 고정** | 주행 모드 스테이트 머신 + 요구 생성 + ref points 조립 + 종방향 병합 → target ref 확정 → Ethernet TX |
| Control | Vehicle MGM | dSPACE | 10ms | 통신 watchdog → 궤적 생성(quintic, feasibility) → MPC(횡·종 통합, 예측 지평 200ms/N=20 → str_ref, v_ref) → 상태 추정(kinematic bicycle model) |
| Actuation | 하위 제어 | dSPACE | **5ms 독립 태스크** | 엔코더 읽기 → PI → PWM 갱신 (구동 20kHz, 조향 서보 50Hz) |

- dSPACE는 multi-rate: 10ms 태스크(Vehicle MGM)와 5ms 태스크(Actuation)가 한 프로세서에서 병행. **주기는 합산되지 않음** — 10ms 틱 안에서 수신→궤적→MPC가 연쇄 실행.
- Actuation은 상위와 무관하게 항상 5ms로 돌며, 최신 목표값(str_ref, v_ref)을 hold하여 사용.
- 대안 v3: ROS 2 실시간성 검증 실패 시 ADAS MGM을 통째로 dSPACE로 이관 (§7 판정 기준 참조). 로직은 동일, 배치만 변경.

## 3. 통신 계약 (Ethernet, PC ↔ dSPACE) — 최우선 구현 대상

**TX (PC → dSPACE, 10ms):**

| 필드 | 형식 | 설명 |
|---|---|---|
| ref_points[N] | {x, y, yaw, curvature} × N | **vehicle frame** — 생성 시점 차량 위치 = (0,0,0). 모든 모드(차선/GPS/회피/주차)가 동일 포맷 |
| v_ref | float | 종방향 병합의 최종 목표 속도. 정지 = v_ref 0 (별도 정지 명령 없음) |
| flags | uint | state + packet counter. **counter는 watchdog 필수 입력** |

**RX (dSPACE → PC, 10ms):**

| 필드 | 설명 |
|---|---|
| vehicle_vector | 상태 추정 결과 {x, y, yaw, v, str} — PC의 localization 보정에 사용 |

**watchdog (dSPACE):** 패킷 counter가 30ms(TX 3주기) 동안 미갱신 → v_ref = 0 (감속 정지), 조향은 직전 값 유지(급조향 금지). 타임아웃 값은 §7 검증 결과에 따라 조정 가능(예: 50ms).

## 4. 스테이트 머신 (Decision 핵심 — 상세: `docs/state_machine_detail.drawio`)

스테이트 4개: **lane ↔ waypoint ↔ avoid ↔ parking**. 판단 로직은 시스템 전체에서 이 스테이트 머신 한 곳에만 존재한다.

**전이 조건:**

| 전이 | 조건 |
|---|---|
| lane → waypoint | 차선 신뢰도 < 임계, N주기 연속 |
| waypoint → lane | 차선 신뢰도 > 복귀 임계, N주기 연속 (히스테리시스 — 이탈/복귀 임계 분리) |
| lane·waypoint → avoid | 장애물 감지 AND 회피 가능 (TTC·측방 여유 충분) |
| avoid → 복귀 | 기동 완료 → **진입했던 스테이트로** 복귀 (복귀처 변수 1개만 기억) |
| lane → parking | GPS 주차구간 AND 주차공간 인식 |
| parking → lane | 주차 완료 |

**스테이트별 우선권 (매 10ms, 스테이트 내부에서 결정 — 전역 min/max 규칙 금지):**

- **lane · waypoint** — 종방향: 긴급 정지 > 신호등 정지 > 가속구간 > 기본 속도 / 횡방향: 차선 경로 / GPS 경로
- **avoid** — 기동 완료 우선, 정지 요구는 기동 이탈 후 적용. 단 TTC < 임계 → 즉시 정지(안전 바닥). 횡: 회피 경로 / 종: v_ref는 회피 기하로 결정(여유 폭 좁으면 감속)
- **parking** — 경로 침범 정지 > 주차 진행. 신호등·가속구간 요구 비활성. 정적 경계(콘·연석)는 정지 트리거가 아니라 로컬맵 입력

**원칙:**
- **정지는 스테이트가 아니다.** 긴급/신호등 정지는 각 스테이트 우선권 표의 요구일 뿐, v_ref=0으로만 반영. 정지 중에도 주행 스테이트 유지 → 복귀 관리 불필요.
- 회피는 avoid **스테이트**다. "회피 경로 요구"라는 별도 요구는 존재하지 않는다 (parking→avoid 전이 없음 = 주차 중 회피 금지가 구조적으로 보장됨).

## 5. 구현 금지·필수 사항

1. **조립/병합 블록에 판단 로직 금지.** ref points 조립·종방향 병합은 스테이트의 결정을 포맷 변환·표 적용하는 순수 실행부. `if (조건) v_ref = 0` 같은 조건 분기를 여기 넣는 순간 판단이 두 곳으로 흩어진다. 모든 "무엇을 할지"는 스테이트 머신으로.
2. **MGM 10ms 루프는 인지 노드와 별도 프로세스.** 인지 콜백이 루프를 블로킹하면 안 됨. 루프는 매 틱 "최신 인지 스냅샷"을 읽는 pull 방식. SCHED_FIFO 우선순위·CPU 코어 고정 적용 검토.
3. **주기 지터 로깅 필수.** MGM 루프에 주기 실측 로깅을 처음부터 내장 (§7의 판정 근거).
4. **모든 경로 소스는 동일 ref points 포맷.** 차선/GPS/회피/주차 어느 것이 이기든 dSPACE는 구분할 필요 없음 — MPC 무수정 원칙.
5. **PWM 주파수 혼동 금지.** 서보 = 50Hz(펄스폭이 위치), 구동 모터 드라이버 = 20kHz(duty가 전압). 
6. 스테이트 전환 시 ref 불연속 방지(전환 연속 처리), 급격한 v_ref 변화 rate limit — 조립 블록의 허용 업무.

## 5.5 MGM 개발 전략 (이중 트랙)

- **MGM 로직 코어는 `mgm_step()` 순수 함수로 격리** — ROS 의존성 없음. 시그니처: `출력(ref_points, v_ref, flags) = mgm_step(인지 스냅샷, 이전 상태)`. ROS 2 노드는 10ms 타이머에서 이 함수를 호출하는 wrapper일 뿐이며, **wrapper에 판단 로직 금지**.
- 두 구현이 이 인터페이스를 공유: ① ROS 2/C++ 직접 구현(김윤기, 레퍼런스 — 스프린트 기본 탑재), ② Simulink/Stateflow 모델 → 생성 C 코드(김재민, MBD).
- 검증: 동일 rosbag을 두 구현에 재생 → 출력(ref points, v_ref, 스테이트 전이) 비교. 차이 발견 시 분석 결과를 모델 쪽에 피드백.
- 생성 코드가 안정 동작하면 wrapper에서 함수 교체로 탑재 전환. 스펙의 단일 소스는 본 문서 §4 — 두 구현 모두 여기서만 파생하며, 스펙 변경은 문서 갱신이 선행.

## 6. 워크스페이스 구조

```
adas_ws/src/
├── common_interfaces/     # msg/인터페이스 정의 — 모든 스택의 공용 계약 (§3 포맷)
├── bridge_dspace/         # Ethernet UDP 브리지 (TX/RX) — ★ 최우선 구현, 배포 전 완성
├── adas_mgm/              # Decision — core/(§5.5 로직 코어, ROS 무의존) + src/(wrapper) + tools/(back-to-back 하네스)
├── stack_lane/            # 이현준 — 차선 검출(YOLO) → 차선 ref
├── stack_gps/             # 김윤기 — GPS·IMU 융합, RTK, waypoint ref
├── stack_parking/         # 손상민 — 주차공간 인식·로컬맵·주차 경로
├── stack_avoid/           # 이기돈 — 장애물 인지, 회피 가능 판정(TTC·측방), 회피 경로
├── stack_traffic/         # 김재민 — 신호등·정지선 인식 → 정지 요구
└── stack_estop/           # 박찬미 — 돌발 장애물 감지 → 긴급 정지 요구
```

- 각 stack 폴더에 `REQUIREMENTS.md` 포함 (담당자별 요구사항).
- 각 스택의 출력은 `common_interfaces`에 정의된 토픽/메시지로만 — MGM은 그 토픽들만 구독.
- 부트스트랩 순서: ① bridge_dspace로 PC↔dSPACE 왕복 검증 (더미 ref로 바퀴 반응 + vehicle vector 회신 확인) → ② adas_mgm 10ms 루프 골격 + 지터 로깅 → ③ 각 스택 배포·병렬 개발.

## 7. 실시간성 검증 기준 (v1 유지 vs v3 이관)

- ref points는 매 주기 vehicle frame (0,0,0)으로 재생성 → **지연에 의한 추종 오차는 새 ref 수신 시 청산, 누적되지 않음.** 따라서 실질 제약은 watchdog 오탐 하나.
- **판정식: "최악 지연 × 2를 watchdog 타임아웃으로 잡았을 때, 그 타임아웃이 안전한가(PC 사망 시 정지 개시까지의 이동거리가 수용 가능한가)?"**
  - Yes → v1 유지 (필요 시 타임아웃 조정, 예: 30→50ms)
  - No (지연 꼬리가 유계되지 않음) → v3 이관
- 측정 조건: 인지 노드 풀가동 상태에서 장시간(수십 분~1시간) 주기 로깅, 최악값 기준(평균 아님).

## 8. 팀 담당 · 일정

| 이름 | 담당 | 8/10 산출물 |
|---|---|---|
| 김윤기 (팀장) | GPS — 베이스 설치, RTK, GPS 주행 | GPS 단독 주행 |
| 이현준 | 카메라 차선 주행 | 차선 단독 주행 |
| 손상민 | 라이다 주차 + MPC·Vehicle MGM (dSPACE 적용, 검증은 카메라·GPS 주행 연계) | 주차 단독 + MPC 검증 |
| 이기돈 | 장애물 회피 + 하위 세부 제어 보완(김재민에게서 인수) | 회피 단독 주행 |
| 김재민 | 신호등·정지선 + 하위제어 사수(기반 7/6~7/17 완료) | 신호등 정지 |
| 박찬미 | 돌발 장애물 긴급 정지 | 긴급 정지 |

미팅 주 2회(월·목). 마일스톤: 8/10 센서 단독 주행 통합 시연.

## 9. 참조 문서

- `docs/system_architecture_v1.drawio` — 기본 아키텍처
- `docs/system_architecture_v3.drawio` — dSPACE 이관 대안
- `docs/state_machine_detail.drawio` — 스테이트 전이·우선권 표
- `docs/dynamic_architecture.drawio` — 한 제어 주기(10ms) 시퀀스
