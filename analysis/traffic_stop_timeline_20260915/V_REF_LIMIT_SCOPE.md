# v_ref 출력 변화율 제한 적용 범위

2026-09-15 현재 작업 트리 기준 읽기 전용 조사. 주행 코드/런처는 수정하지 않았다.

## 현재 실차용 경로

MGM 상태별 목표 결정 → `mgm_step.cpp::merge()` → 참조 유효성 검사 →
`mgm_node.cpp`의 TargetRef → `can_bridge_node.cpp` → CAN 0x100.
params.yaml의 backend는 core이며 C++의 base manager가 현재 v2 경로다.

일반 변화율 제한은 merge() 한 곳에 있다. 고정 주기 0.01초에서
`clamp(목표, 이전명령-a_down*0.01, 이전명령+a_up*0.01)`를 적용한다.
기본 설정 a_up=0.5, a_down=1.5: 틱당 증가 0.005, 감소 0.015 m/s.
이는 실제 차량 가속도가 아닌 **부호 있는 속도 명령값**의 변화율이다.
따라서 -1→0에는 a_up, 0→-1에는 a_down이 쓰인다(해당 경로가 제한을 거칠 때).

| 상황 | 현재 v2 적용 |
| --- | --- |
| 차선/GPS 일반 주행의 비영 목표 | 회피 램프 잔여 상태가 없으면 바로 전달 |
| AVOID 경로 선택 | avoid_speed_ramp=true; 증가·감소·정지·재출발 모두 제한 |
| AVOID → GPS/차선 복귀 | 플래그를 유지해 목표에 도달할 때까지 제한; 도달하면 해제 |
| 신호등 거리 계산 활성 | TRAFFIC speed owner + distance_latched일 때 제한 |
| 일반 목표 0, immediate_stop=false | 모든 소스에서 제한(정지 Zone, 일반 주차 단계 정지 등) |
| 주차 비영 목표 | PARKING 소스가 회피 플래그를 지우므로 보통 바로 전달 |
| 자동 후진 ESCAPE 비영 목표 | 회피 플래그를 지우므로 v2에서는 바로 전달 |
| AUTO_ESTOP / SAFE_STOP / 외부 정지 / 비주행 / FINISH | immediate_stop으로 즉시 0 |
| 후진 회복 종료, 주차 초기 준비 등 강제 정지 | immediate_stop이면 즉시 0 |
| 잘못되거나 오래된 참조, 비유한 명령 | 후단 유효성 검사에서 즉시 0 |

위 표는 실제 선택 source와 immediate_stop에 의해 결정된다. GPS_RETURN이라는
상태 이름만으로 제한 여부를 고정할 수 없으며, 직전 AVOID 소스의 플래그가 중요하다.

## CAN까지의 후속 처리

- mgm_node.cpp: `msg.v_ref = out.v_ref`; 무효 참조/비유한 값에는 0을 덮어쓴다.
- can_bridge_node.cpp: `hdr.v_ref = quantize(msg.v_ref, kVelScale)`.
- kVelScale=0.001 m/s; int16 범위에 맞춰 포화/반올림한다. 시간 필터는 없다.
- 하위 제어기 내부 구현은 이 조사 범위 밖이다.

## 다른 구현과 제거 시 경계

- core legacy(`base_state_machine_enabled=false`)도 같은 merge()를 쓰지만,
  v2의 비영 목표 우회가 없어 일반 가감속에도 적용된다.
- generated ADAS_MGR2.c에는 별도 a_up/a_down 및 UnitDelay 기반 제한이 있다.
  현재 v2의 base manager와 함께 선택할 수 없는 별도 실험 백엔드다.
- 따라서 v2만 제거할 때는 merge()의 base manager 분기에서 결정된 목표를 그대로
  전달하고 st.v도 같은 값으로 갱신하는 방식이 적합하다. immediate_stop과
  final_reference_gate는 기존대로 둔다. a_up/a_down을 0으로 설정하면 우회가 아니라
  명령이 이전 값에 묶이므로 비활성화 방법으로 사용할 수 없다.
- a_up/a_down을 CoreParams에서 바로 삭제하면 과거 덤프 ABI와 generated adapter에
  영향이 있다. v2에서 사용하지 않는 것과 공유 인터페이스 삭제는 별개다.
- st.v는 회피 램프뿐 아니라 정지 Zone 대기 타이머, 후진 무장, 음수 명령 종료
  가드에서도 읽는다. 변수 자체를 삭제하지 말고 명령 기록으로 유지해야 한다.
- 특히 manager_step.cpp:138은 실제 속도 대신 st.v로 정지 Zone 대기 시작을 판정한다.
  제한 제거 후 명령 0이 즉시 반영되면 실제 정차 전에 대기 시간이 차감될 수 있어,
  실측 속도 유효성/정차 조건을 함께 검토해야 한다.
- 신호 STOPPED_WAIT는 이미 유효 실측 속도 |v|<=0.001을 조건으로 사용한다.

## 이번 제거와 다른 계산

- 신호등 거리별 목표 속도 계산, 일반/회피 Zone 속도, 후진 속도, 정지 판단은
  상태 로직이 결정하는 목표이며 후단 변화율 제한과 구분한다.
- assemble()의 blend_cycles=10은 x/y/yaw/curvature의 경로 전환 보간이며 v_ref에는
  적용하지 않는다. main_gap의 target rate limit도 회피점 위치 제한이다.

## 검증 영향

v2의 avoid_main_compat_test/fixed_speed_test는 기존 램프를 기대하는 사례가 있어
제거한 계약에 맞춰 수정해야 한다. 신호 감속은 a_down을 크게 설정해 우회한 기존
traffic_state_test만으로 검증하지 말고 실차 기본 설정으로 최종 출력 일치를 확인해야 한다.
legacy/generated를 유지하면 해당 parity 테스트의 기대값은 유지해야 한다.

근거: mgm_step.cpp:734–760, 821; manager_step.cpp:138, 425–438, 611–683;
reference_safety.cpp::final_reference_gate; mgm_node.cpp:1279,1303;
can_bridge_node.cpp:329; can_protocol.hpp::quantize;
generated/adas_mgr2/ADAS_MGR2.c:724–729.
