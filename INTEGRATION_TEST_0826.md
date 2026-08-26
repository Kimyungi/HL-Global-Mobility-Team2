# 통합 시험 · 회생 코드 작업 준비 (main_test0826)

> 2026-08-26 · 브랜치 `main_test0826` (기점: `5cc0560`, main 보다 1커밋 앞)
>
> 목적 3가지 — ① 전 스택 **통합 기동**, ② 코드 전체의 **오류·버그 가능성** 점검,
> ③ **차가 멈췄을 때 스스로 살아나는 코드(회생)** 추가.
> 설계 근거는 `CLAUDE.md`, 운용 절차는 `HANDOVER.md`. 이 문서는 **이번 작업의 대상 목록**만 맡는다.

---

## 1. 지금의 "정지 → 회생" 지도

정지시키는 자리는 전부 찾았다. 회생(스스로 다시 달리기)이 있는 것과 없는 것이 갈린다.

| # | 정지 원인 | 코드 위치 | 회생 경로 | 상태 |
|---|---|---|---|---|
| 1 | 실제 estop (EstopRequest) | `core/mgm_step.cpp:367` | 장애물이 치워지면 해제 (레벨 신호) | ✅ 자동 |
| 2 | 치울 수 없는 장애물 앞 교착 | `core/mgm_step.cpp:153-215` 후진 탈출 | 10s 갇히면 후진 → 회피 재시도 | ⚠ **기본 잠김** (§2-A) |
| 3 | 신호등 적색 래치 | stack_traffic | fresh 초록 3/5 | ✅ 자동 |
| 4 | 지정 지점 정차 (stop_zone) | `mgm_step.cpp:136-149` | 3s 뒤 스스로 재출발 | ✅ 자동 |
| 5 | 역방향 래치 | `mgm_step.cpp:78-83` | 신뢰 헤딩 정렬 0.5s | ✅ 자동 |
| 6 | 트랙 종점 (at_end) | `mgm_step.cpp:101-104` | **실제 estop 인가로만** 해제 | 수동 (의도된 설계) |
| 7 | 출발 인가 대기 (wait_go) | `src/mgm_node.cpp:418` | `ros2 run adas_mgm go` | 수동 (의도된 설계) |
| 8 | 인지 노드 staleness → estop 보정 | `src/mgm_node.cpp:403-478` | **노드가 되살아나야 함** | ❌ **회생 없음** (§2-B) |
| 9 | 인지 노드 사망 | launch `die_hard` / 무처리 | launch 전체 종료 + can_zero | ❌ **사람이 재기동** (§2-B) |
| 10 | GPS 로버 출력 사망 | `stack_gps` GgaLink | NMEA 무수신 20s → USB 재열거 | ✅ 자동 (유일한 자가치유 선례) |
| 11 | CAN 버스 오류 (bus-off) | 드라이버 `restart-ms 100` | 컨트롤러 자동 복구 | ✅ 있음 (초안 진단 정정) |
| 12 | CAN 어댑터 이탈·TX 영구 실패 | `bridge_dspace` | **관측도 반응도 없음** | ❌ **가장 위험** (§2-C) |

**결론: 회생이 필요한 곳은 8·9·11 세 갈래고, 2는 만들어 놓고 못 쓰는 상태다.**

---

## 2. 회생 코드 추가 대상 (우선순위)

### A. 후진 탈출을 실제로 쓸 수 있게 만든다 — `EstopRequest.rear_clear`
- 코어·단위시험(`test/escape_reverse_test.cpp`, 9종)·메시지 정의는 **이미 있다**.
  비어 있는 것은 **채우는 쪽뿐**: `stack_estop`이 `rear_clear`를 발행하지 않는다
  (`grep -rn rear_clear src/stack_estop` → 0건). 항상 false라 `escape_require_rear_clear`
  기본값에서 기능이 자연히 잠긴다.
- 할 일: 후방 판정을 `stack_estop/node.py`에 넣는다. 후방 단독 라이다 / 4대 통합
  (`multi_lidar_fusion/merged_scan`) 중 선택은 이기돈 결정 — 장착 yaw 확정값은
  `multi_lidar_fusion` 설정에 있다.
- 그 다음 실차 검증 2건: 음수 `v_ref`에 대한 dSPACE MPC·하위 PI 동작(손상민 확인 필요),
  그리고 후진 0.6m 실제 거동. 되돌리기: `escape_after_cycles 0`.

### B. 노드가 죽으면 "멈춘 채로 끝"이 아니라 다시 살아나게 한다
현재 정책은 **fail-stop 일변도**다. 그리고 노드마다 정책이 다르다:

| 노드 | 죽었을 때 | 근거 |
|---|---|---|
| `stack_gps_node` | launch 전체 종료 + can_zero | launch:505 `die_hard` |
| `mgm_node` | launch 전체 종료 + can_zero | launch:575 `die_hard` |
| `stack_avoid_node` | **조용히 사라짐** — MGM watchdog이 estop 보정, 영구 정지 | launch:442 (on_exit 없음) |
| `stack_estop_node` | **조용히 사라짐** — §5.7 ① 로 영구 estop | launch:451 (on_exit 없음) |
| `stack_lane_node` | **조용히 사라짐** — LANE 상태에서만 estop | launch:509 (on_exit 없음) |
| ydlidar | `respawn=True, delay 2.0` (유일) | launch:430 |

- 할 일 ①: 정책을 **의도적으로** 정한다. 세 노드(avoid·estop·lane)가 무처리인 것은
  설계 결정이 아니라 누락으로 보인다. 후보는 `respawn=True` + 재기동 유예 동안
  MGM watchdog이 estop을 유지(이미 그렇게 동작한다) — 즉 **"멈춘 뒤 노드가 살아나면
  스스로 재출발"** 이 자연히 성립한다. 단 카메라·라이다를 잡는 노드는 장치 재오픈이
  실패하면 respawn 루프가 되므로 재시도 횟수 상한이 필요하다.
- 할 일 ②: MGM에 **재출발 조건**을 명시한다. 지금은 watchdog이 풀리는 순간 곧바로
  v_base로 램프업한다. 노드가 되살아난 직후의 첫 프레임으로 즉시 달리는 것이 맞는지
  (예: 신선한 입력 N틱 확인 후 재출발) 결정이 필요하다.

### C. CAN 회생 — 이번 작업에서 가장 위험한 구멍

**먼저 정정 (2026-08-26 재확인):** "bus-off 자동 복구가 없다"는 이 문서 초안의 진단은
틀렸다. `restart-ms 100`은 udev(`tools/can_up.sh:7`)·`PROTOCOL.md:141`·현장 스크립트
전부에 이미 들어 있다 — **컨트롤러 레벨 bus-off 복구는 이미 동작한다.**
따라서 C의 대상은 "버스를 되살리는 일"이 아니라 그 아래 세 가지다.

**① 실패가 아무에게도 전달되지 않는다.**
송신 실패는 1초 throttle 경고 한 줄이 전부고(`can_bridge_node.cpp:100`),
`tx_count_`가 안 오르는 것 말고는 흔적이 없다. 브리지가 바깥에 내보내는 토픽은
`/vehicle/vector` 하나뿐이며(`can_bridge_node.cpp:40`) MGM은 그것조차 구독하지 않는다
(`grep vehicle/vector src/adas_mgm` → 코어 주석 1건뿐). **MGM은 CAN 상태를 알 수단이
구조적으로 없다.**

**② RX를 헬스 신호로 대신 쓸 수도 없다.**
`/vehicle/vector`는 **실측 수신 0건**이다(`core/mgm_types.hpp:180` — stack_avoid의 TTC
자차속도가 항상 폴백을 쓰는 이유). 수신 침묵이 평시 상태이므로 고장 판정에 못 쓴다.

**③ `restart-ms`가 못 덮는 고장이 남는다.**
PCAN 어댑터가 USB에서 이탈하면 인터페이스 자체가 사라진다 — 소켓은 죽은 ifindex에
묶인 채고 write는 `ENETDOWN`으로 영구 실패한다. 이 리포는 USB 이탈 전력이 이미 있다
(OAK-D 재열거, 로버 F9P 무출력 → GgaLink USB reset). 그리고 `read()`가 타임아웃 없이
즉시 -1을 반환해 RX 루프가 **100% CPU busy-spin**에 빠진다
(`can_bridge_node.cpp:112` — errno 무구분).

**그래서 위험이 어디서 겹치는가.** 정지 명령의 통로는 CAN 하나뿐이고, dSPACE counter
watchdog은 미구현이라 무응답 = 마지막 `v_ref` 무기한 유지다. 마지막 안전망인 `can_zero`도
같은 통로를 쓴다 — 실패를 stderr로 알리기는 하나(`can_zero.py:74` "★목표값이 0 이 아닐
수 있다★") **버스가 죽은 상태에서는 0을 실어 보낼 방법이 없다.**

> **정직한 한계: C는 "CAN이 죽은 동안 차를 세우는" 문제를 풀지 못한다.**
> 그것을 풀 수 있는 것은 dSPACE counter watchdog(손상민) 하나뿐이다. C가 하는 일은
> ⓐ 고장을 **즉시 관측**하고 ⓑ 링크를 **빨리 되살리고** ⓒ 되살아난 첫 프레임이
> **정지값이 되게** 만드는 것이다. 즉 C의 절반은 손상민 몫이며, 그 의존을 문서에
> 남긴 채 PC 쪽 절반을 먼저 끝낸다.

**수정 대상 (파일별)**

| 파일 | 무엇을 |
|---|---|
| `bridge_dspace/src/socketcan.hpp` | `CAN_RAW_ERR_FILTER` 옵션 추가 · `sendCanFrame`이 bool 대신 errno를 돌려주게 · 소켓 재오픈 헬퍼 분리 |
| `bridge_dspace/src/can_bridge_node.cpp` | ⓐ `sendFrames` 원자성 — 점 송신이 실패하면 **헤더를 보내지 않는다**(지금은 `ok &=`로 누적만 하고 헤더가 그대로 나가 dSPACE가 반쯤 갱신된 세트를 latch할 수 있다) ⓑ errno 분류 → 일시(EAGAIN/ENOBUFS) vs 치명(ENETDOWN/ENODEV) ⓒ 치명이면 소켓 재오픈 재시도 상태머신 ⓓ `rxLoop` errno 구분으로 busy-spin 제거 ⓔ 헬스 발행 |
| `fma_interfaces` | `CanHealth.msg` 신규 — `link_up` · `tx_ok` · 연속 실패 틱 · 마지막 errno |
| `adas_mgm/src/mgm_node.cpp` | §5.7 **⑥** 로 CAN 헬스 watchdog 추가 — 미수신/불건전이면 `estop=true` 보정. lane·gps·avoid와 **똑같은 자리·똑같은 방식**이라 새 개념이 아니다 |
| `stack_avoid/stack_avoid/can_zero.py` | 실패 시 재시도 + 종료코드로 실패를 알림 (지금은 stderr 한 줄 뒤 `return False`) |
| launch 3종 | 브리지 사망 시 정책 — B의 결정과 함께 |

**코어(`mgm_step.cpp`)는 건드리지 않는다.** CAN 헬스를 wrapper의 estop 보정으로만
처리하면 `CoreSnapshot`이 안 바뀌고, 따라서 덤프 포맷 버전(v6)도 그대로이며
**기존 덤프 재생 결과가 비트 단위로 동일**해야 한다 — 이것이 이번 변경의 회귀 판정
기준이 된다(§4-3). 코어에 필드를 더하는 순간 v7 bump + 과거 덤프 재생 불가가 따라온다.

**복구 후 재출발 규약 (설계 결정 필요).** 링크가 살아난 순간 곧바로 `v_base`로 램프업하면
안 된다 — 사람이 상황을 모르는 채 차가 다시 움직인다. 후보는 estop 보정을 유지한 채
`wait_go` 재인가를 요구하는 것(§2-B②와 같은 질문이므로 함께 결정).

**C 진행 시 예상 결과 (시나리오별 전/후)**

| 상황 | 지금 | C 이후 |
|---|---|---|
| 일시적 버스 에러 (프레임 몇 개 실패) | 경고 1줄, 그대로 진행. 헤더는 나가므로 dSPACE가 **반쯤 갱신된 세트**를 latch 가능 | 헤더 억제로 그 틱은 통째로 버려짐 → dSPACE는 직전 세트 유지(§3 계약대로). 연속 실패가 문턱 미만이면 estop 없이 지나감 |
| dSPACE 전원 off / 종단저항 문제 (ACK 없음 → error-passive→bus-off 반복) | `restart-ms`가 컨트롤러만 되살리고 TX는 계속 실패. **PC는 계속 주행 명령을 만든다** | 연속 실패 감지 → CanHealth 불건전 → MGM estop 보정. 버스가 살아나는 순간 나가는 첫 프레임이 `v_ref 0` |
| PCAN USB 이탈 (ENETDOWN) | write 영구 실패 + **RX 스레드 100% CPU 점유** → MGM 10ms 루프 지터 악화(§7) | busy-spin 제거(errno 구분), 소켓 재오픈 재시도, estop 보정. 어댑터를 다시 꽂으면 udev가 can0를 올리고 브리지가 재접속 |
| 정상 주행 | — | **동작 변화 없어야 한다.** 같은 덤프 `core_replay` 결과 비트 단위 동일이 판정 기준 |
| CAN이 죽은 채 차가 굴러가는 상황 | 마지막 `v_ref` 무기한 유지 | **여전히 못 막는다** — dSPACE watchdog(손상민)만이 해결. C는 창을 좁히고 사람이 알게 할 뿐 |

**검증 방법 (실차 전에 전부 벤치에서 재현 가능)**
1. `sudo ip link set can0 down` — ENETDOWN 경로. CPU 점유율·estop 보정·재오픈 확인.
2. dSPACE 미연결로 브리지 기동 — ACK 없음 경로(no-ACK는 종단만 있으면 재현된다).
3. `vcan` loopback — 정상 경로 회귀.
4. 기존 덤프 `core_replay` 전/후 동일성 — 코어 무수정의 증명.

**작업량 감각:** 파일 6개, 코어 0줄. 브리지가 169줄짜리 단일 파일이라 변경이 국소적이다.
가장 손이 많이 가는 것은 코드가 아니라 **문턱값 결정**(몇 틱 연속 실패를 고장으로 볼지)과
**복구 후 재출발 규약**이다.

---

## 3. 통합 기동 중 확인할 오류·버그 후보

코드를 훑으며 나온 것. 실차 전에 재현·판정한다.

1. **RX 루프 busy-spin** — `can_bridge_node.cpp:106` `read() < 0 → continue`.
   `SO_RCVTIMEO` 덕에 평시엔 EAGAIN(정상)이지만, can0가 내려가면 `ENETDOWN`이
   **즉시** -1을 반환해 타임아웃 없이 100% CPU 회전에 빠진다. errno 구분 필요.
2. **`sendCanFrame` 부분 실패** — 20점 + 헤더를 보내다 중간에 실패해도 `ok &=` 로
   누적만 하고 헤더는 그대로 나간다. dSPACE는 헤더에서 latch 하므로 **반쯤 갱신된
   세트**를 잡을 수 있다. §3이 막으려던 바로 그 상황.
3. **stack_traffic 기동 불가** — `HANDOVER.md`에 실측으로 기록된 미해결 건.
   통합 launch에 넣을지 뺄지 먼저 정해야 한다.
4. **속도 상수 세트 정합** — `v_base`·`target_speed_mps`는 항상 동일해야 하고
   (CLAUDE.md §3), 1.0 상향에는 `ttc_stop` 1.3 · estop 1.20/1.35 · dynamic 1.35가
   세트로 따라온다. 통합 launch 3종에서 값이 갈라지지 않았는지 대조.
5. **`v_narrow` 0.2 위반 (미결)** — "v_ref ≥ 0.5" 규칙과 정면 충돌하며 실측 10.2%에서
   발동한다. 이번 통합에서 정책 결정 필요(이기돈).
6. **실차 미검증 3건** — GPS ref[0] 하한 `rejoin_target_min_m` 1.8(ⓒ),
   곡선 대응 `rejoin_curve_ff`/`rejoin_curve_margin`, 후진 탈출. 각각 되돌리는
   파라미터 한 줄을 미리 손에 쥐고 나간다.

---

## 3.5 C안 구현 완료 (2026-08-26)

| 파일 | 상태 |
|---|---|
| `fma_interfaces/msg/CanHealth.msg` | 신규 |
| `bridge_dspace/src/socketcan.hpp` | `tryOpenCanSocket`(throw 안 함) · `CAN_RAW_ERR_FILTER` · `sendCanFrame` errno 반환 · `isFatalCanErrno`/`isIdleCanErrno` |
| `bridge_dspace/src/can_bridge_node.cpp` | 세트 원자성(헤더 억제) · errno 분류 · 소켓 재오픈(수신 스레드 단독 소유) · busy-spin 제거 · 에러 프레임 처리 · 헬스 발행 |
| `adas_mgm/src/mgm_node.cpp` | §5.7 ⑥ watchdog + 재인가 래치 · `/operator/go` 구독을 `wait_go` 와 분리 |
| `adas_mgm/tools/can_watchdog_check.py` | 신규 — 하드웨어 없이 6단계 시퀀스 검증 |
| `CLAUDE.md` §5.7 · `PROTOCOL.md` · `CAN_BRINGUP.md` 6-2단계 | 갱신 |

**코어 무수정 확인:** `git diff --stat src/adas_mgm/core/` 가 비어 있다. `core_replay` 는
`tools/core_replay.cpp + mgm_core` 만 링크하므로(CMakeLists:31) 재생 출력은 **구조적으로**
동일하다. `CoreSnapshot`·덤프 포맷 v6 그대로.

**검증 결과**
- 단위시험 5종 전량 통과 (escape_reverse / decision_backend ×2 / generated_adapter_safety /
  lane-waypoint parity `ticks=900 mismatches=0`).
- `core_replay` 실덤프 재생 정상 (`run_0825_071202`, 22976틱).
- `can_watchdog_check.py` **PASS** — v_ref: ① 건전 0.5 → ② 링크다운 0.0 →
  ③ 복구 0.5(자동, 래치 없음) → ④ 2초 다운 0.0 → ⑤ 복구 0.0(래치 유지) →
  ⑥ go 재인가 0.5. 로그에 `CAN 고장 1.0s 지속 — 래치` · `래치 해제 — 재인가로 주행 재개`.

**실기 검증 완료 (2026-08-26 오후, PCAN 연결 상태)**
`sudo ip link set can0 down` 으로 60초간 재현 — 4가지 전부 확인:
- errno 분류: `CAN write 실패 N회 연속 — Network is down (치명 — 소켓 재오픈 대기)`
- 래치: 실패 시작 → **1.08초** 뒤 `CAN 고장 1.0s 지속 — 래치`
- busy-spin 제거: 링크 다운 중 브리지 **CPU 3.0%** (구코드였다면 100%)
- 자동 복구: 링크 복구 감지 → **1.0초**(reopen_interval) 뒤 재오픈, tx 100Hz 재개.
  실제 다운 4회 → 복구 4회, 오탐 0.
헬스도 정확: `link_up=false · tx_ok=false · consecutive_tx_fail=5888(=100Hz×59s) ·
last_errno=100(ENETDOWN) · down_duration_s=58.88 · bus_off=false`.
(bus_off 가 false 인 것이 맞다 — 인터페이스 다운은 버스 고장이 아니다.)

**실기에서만 드러난 결함 1건 → 수정 완료.** `ip link set can0 down` 은 인터페이스를
없애지 않으므로 `SIOCGIFINDEX`·`bind` 가 성공한다. 그래서 링크가 죽은 채로
`CAN 재오픈 성공` 이 1초마다 찍혔다 — 사람이 복구된 줄 안다. `isCanLinkUp()`(IFF_UP)
을 추가해 재오픈 전에 링크 상태를 먼저 보게 했다. 재시험에서 오탐 0회 확인.

**되돌리기:**
`ros2 param set /adas_mgm_node can_relatch_sec 0.0` (래치만 끔) /
`reopen_interval_sec 0.0` (재오픈만 끔).

---

## 4. 진행 순서

1. 빌드 정합 확인 (`colcon build --symlink-install 금지` — `HANDOVER.md`).
2. 단위시험 전량 통과: `colcon test --packages-select adas_mgm`.
3. 덤프 재생 back-to-back: 회생 코드를 넣기 **전/후** 같은 덤프로 `core_replay`를
   돌려 **스테이트 시퀀스·immediate_stop 틱이 동일**한지 확인. 이 리포의 관례다
   (§4 히스테리시스·`v_avoid`·후진 탈출 모두 이 방식으로 검증됐다).
4. 벤치 기동 → 실차. 실차 절차·함정은 `HANDOVER.md`, 회피는
   `stack_avoid/RUNBOOK_avoid_field_test.md`.

## 5. 결정이 필요한 것 (이기돈)

- A: `rear_clear` 판정을 후방 단독 라이다로 할지 4대 통합으로 할지.
- B: 세 노드(avoid·estop·lane)를 respawn 으로 갈지 die_hard 로 통일할지.
- B②: watchdog 해제 직후 즉시 재출발 vs 신선 입력 N틱 확인 후 재출발.
- 3-3: 통합 launch 에 stack_traffic 을 포함할지.
- 3-5: `v_narrow` 0.2 를 어떻게 할지.

---

## 6. C안이 인수인계와 방향이 갈리는 지점 (2026-08-26 대조)

김윤기가 같은 문제에 대해 남긴 방향(`HANDOVER.md` §3.6·§3.7·§6-3, `PROTOCOL.md`
"watchdog 상세", `CAN_BRINGUP.md` 6단계)과 §2-C 를 대조했다. **갈리는 것만** 적는다.
새로 찾은 것(헤더 원자성·busy-spin·CanHealth)은 인수인계에 대응 항목이 없으므로 제외.

### 차이 ① 복구 후 재출발 — "래치 없음" vs "재인가"

**먼저 정정 (2026-08-26):** 이 절의 초안은 "채택하면 `CAN_BRINGUP.md` 6단계 시험이
실패로 보인다"고 썼다. **틀렸다.** 6단계는 `pkill -f dummy_ref_publisher` 로 **송신을
멈추는** 시험이지 CAN 링크를 죽이는 시험이 아니다. 브리지는 `/adas/target_ref` 를 받을
때만 쓰므로(`can_bridge_node.cpp:43` — 주기 송신은 통계 로그뿐) 보낼 게 없으면 write
실패도 없고, CAN 헬스는 건전한 채로 남는다 → estop 보정이 안 걸린다 →
**6단계 시험은 그대로 통과한다.** 두 이벤트는 다르다:

| 이벤트 | 누가 감지 | §2-C 의 estop 보정 |
|---|---|---|
| PC 가 송신을 멈춤 (MGM 사망·pkill) | dSPACE watchdog (counter 미갱신) | **안 걸림** |
| CAN 링크·어댑터 고장 (write 실패) | PC 측 CanHealth | 걸림 |

**그래서 실제로 갈리는 것은 좁다** — 링크 고장에서 복구됐을 때 PC 가 스스로 다시
주행 명령을 내도 되는가, 하나다.

| | 김윤기 | §2-C 제안 |
|---|---|---|
| 규정 | `PROTOCOL.md:78` "복구는 자동 — 별도 해제 절차·래치 없음" (**dSPACE 층**) | 링크 복구 후에도 estop 보정 유지, `wait_go` 재인가 (**PC 층**) |
| 전제한 두절 | 프레임 몇 개 수준(수십 ms) | 어댑터 이탈 수준(초 단위, 사람 개입) |

층이 달라 문자적 위반은 아니지만 **철학이 반대**다(그는 래치를 만들지 말라 했고
나는 하나 더한다).

**판단 재료 — 두절 길이가 답을 가른다.**
- 50ms 두절(전기적 잡음): 김윤기가 옳다. 매번 재인가를 요구하면 주행이 불가능하다.
- 8초 두절(어댑터가 빠졌다 다시 꽂힘): 내 쪽이 옳다. 그 사이 차는 마지막 v_ref 로
  굴러갔을 수 있고 사람이 차 옆에 서 있을 수 있다. 자동 재출발은 위험하다.

→ **절충: 시간 문턱 하나로 가른다.** 짧은 두절 = 자동 복귀(그의 규정 유지) /
긴 고장 = estop 유지 + 재인가. 문턱값 결정은 §2-C 의 "문턱값" 항목과 같은 사안이다.

### 차이 ② PCAN USB 이탈 — 운용으로 vs 코드로

- 김윤기(`HANDOVER.md` §3.7): 허브 재열거로 PCAN 이 순간 끊긴다 → **"PCAN 을 PC 직결
  포트로 옮긴다."** 하드웨어 배치 문제로 봤고 코드 대상으로 두지 않았다.
- §2-C: 소켓 재오픈 상태머신으로 **코드가 복구**한다.
- 상충은 아니다 — 그의 대책은 **재발 확률**을 낮추고, 내 대책은 **재발했을 때**를 다룬다.
  다만 직결 포트로 옮기면 재오픈 코드는 거의 실행되지 않는 코드가 된다.
- 내 방향을 지지하는 선례: 같은 성격(USB 장치가 조용히 죽음)을 **코드로 푼 전례**가
  이미 리포에 있다 — `GgaLink` 의 NMEA 무수신 20s → USB 재열거.
- → **결정 필요**: 직결 포트로 옮기는 것이 먼저다. 코드는 그 뒤에도 남는 잔여 위험만큼만.

### 차이 ③ `can_zero` — 강화 vs 폐지 예정

- 김윤기(`HANDOVER.md` §6-3): dSPACE watchdog 이 구현되면 **"§3.6 의 can_zero 가드
  의존을 덜 수 있다"** — 과도기 장치로 본다.
- §2-C: can_zero 에 재시도·종료코드를 **추가**한다.
- 손상민이 watchdog 을 넣으면 이 투자는 상당 부분 사장된다. 반대로 그 일정이
  불확실하면 지금 유일한 안전망이다. → **손상민 일정에 종속된 결정.**
- **2026-08-26: GitHub 이슈 #49 로 손상민에게 일정 문의 · assign 완료** (watchdog 구현
  규정 3가지 · `dspace_sim_node` 레퍼런스 · §6-2 음수 v_ref 실차 확인 동봉).
  답이 오기 전까지 `can_zero` 는 **손대지 않는다**.

### 차이 ④ "재시도"라는 내 표현이 규정에 걸린다 — 그의 방향을 따른다

`PROTOCOL.md:65` / `can_bridge_node.cpp:2-3`: **"브리지는 수신한 TargetRef 를 즉시 송신,
자체 재송신 없음. MGM 이 죽으면 송신도 멈춰야 dSPACE watchdog 이 동작한다 —
keep-alive 를 넣지 말 것."**

§2-C 는 "재오픈 **재시도** 상태머신"이라고 썼다. 소켓 재오픈 자체는 keep-alive 가
아니지만, **실패한 프레임을 다시 쓰는 순간** 이 규정 위반이다. 구현 시 못박을 것:

> 재오픈은 한다. **프레임 재송신·주기 송신은 하지 않는다.**
> TX 는 오직 새 `/adas/target_ref` 수신 시점에만 일어난다.

### 대조했으나 방향이 같았던 것 (조치 불필요)

- 헤더 억제 → counter 미갱신 → dSPACE watchdog 발동. `PROTOCOL.md` 의 "송신이 멈춰야
  watchdog 이 동작한다"와 **같은 방향**이다.
- "dSPACE watchdog 이 근본 해결이고 손상민 몫" — `HANDOVER.md` §6-3 과 동일.
- 검증 하네스는 새로 만들 것 없다: `dspace_sim_node`(watchdog 규정 전부 구현,
  `watchdog_timeout_ms`)· `loopback_test.launch.py` · `CAN_BRINGUP.md` 6단계를 쓴다.
  §2-C 가 추가하는 것은 **no-ACK 재현(dSPACE 미연결)** 하나뿐이다.


---

## 7. 실차 CAN 실측에서 갈라져 나온 것 (2026-08-26)

회생 코드 검증 중 확인한 사실 2건. **둘 다 이 브랜치의 코드 문제가 아니다.**

### dSPACE 가 vehicle_vector 를 간헐적으로만 송신한다 → 이슈 #50

브리지 자체 카운터(`tx=N cycles rx=M cycles`, rosbag 의 `/rosout` 에서 추출)로 확정:

| 시각 | dSPACE 송신 |
|---|---|
| 8/25 04:26~05:00 | 있음 (`rx=107254`) |
| 8/25 06:50~08:27 | 없음 (`rx=0`) |
| 8/26 오전 (재연결 전) | 있음 (`candump` 에 0x200~0x202, payload 전부 0 = 정지 중 정상값) |
| 8/26 재연결 후 ~ | 없음 |

**PC 쪽은 제외된다** — TX 214,506 프레임에 에러 0·`berr-counter tx 0`, 전 프레임 ACK.
ACK 는 상대 컨트롤러가 하드웨어로 응답하는 것이므로 dSPACE 보드는 버스에 물려 있다.

**주행은 RX 없이도 된다** — 8/25 08:05 의 21분 자율주행(`run_mbd_0825_080547`)이
`rx=0` 인 채로 성공했다(전이 7회, v_ref 1.0). 차를 굴리는 것은 PC→dSPACE 방향이다.
잃는 것은 ⓐ TTC 실측(회피 기회 상실) ⓑ dSPACE 사망 감지 ⓒ 조향 개루프 진단
ⓓ 주차·dead-reckoning 입력. 상세는 #50.

**⚠ §5.7 ⑥ 은 이것을 못 잡는다.** 모델이 멈춰도 CAN ACK 는 하드웨어가 계속 응답하므로
우리 송신은 성공으로 보인다. #49(dSPACE watchdog)와 겹치면 **양쪽 다 상대의 죽음을
모르는 상태**가 된다 — #49 에 코멘트로 묶어 두었다.

### PCAN 이 아직 허브 2단 뒤에 있다

`root:3 → 허브(Dev 26) → 허브(Dev 27) → PCAN`. `HANDOVER.md` §3.7 이 지목한 구성이며
대책은 **PC 직결 포트로 이전**(§2-C 차이 ②). 아직 미실시.
