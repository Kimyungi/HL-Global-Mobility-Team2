# v2 상태 머신 변경·제외 보고서

> 이 문서는 초기 변경 당시의 기록입니다. 이후 확정된 **스테이트 v09.17**의 현재 구현과 미사용 상태는 [최신 명세](STATE_V09_17.md)를 확인하십시오. 아래 ESTOP 보류·빈 turn_zones 설명은 현재 상태가 아닙니다.

> 후속 적용: PR #105 최신 맵·zone [3], PR #106 T 주차, dump v33으로 갱신했습니다.
> 아래 최초 인터뷰 구현 기록보다 [후속 통합 보고서](V2_PR106_INTEGRATION.md)가 우선합니다.

2026-09-16 · 적용 진입점: `scripts/v2 prepare` / `scripts/v2 drive`

요구사항 인터뷰와 이번 추가 답변을 기준으로 런처, 상위 MGM, GPS zone 입력, 신호 모듈 실행 방식을 변경했습니다. 기존 사용자 작업은 보존했습니다. 이전 프로파일의 소스 삭제가 아니라 새 v2 운용 경로에서의 제외입니다.

## 현재 완료 범위와 남은 설정

- **ESTOP 진입·복귀 전이와 외부 회복주행 인터페이스 구현. 내부 회복주행은 보류.** 현재 실행기가 없으므로 ESTOP에 진입하면 목표 0으로 대기합니다. 과거 자동 후진으로 대신 탈출하지 않습니다.
- **기존 GPS 전용 구간 자동 적용 제외.** `turn_zones: []` 상태입니다. 새 좌·우회전/Traffic Zone 좌표를 설정하기 전에는 해당 GPS 강제 및 신호 인식 활성화가 없습니다.
- 회피·주차는 추가 답변대로 **현재 모듈 연결 유지**. 내부 경로 생성·준비·완료 알고리즘을 새로 구현하거나 사용자 작업을 덮어쓰지 않았습니다.
- 새 입력 버스와 로그는 **dump v32**입니다. CAN TargetRef의 기존 5개 상태 투영은 유지하며, ESTOP은 MgmState의 safety=4 / estop_active / estop_request_id로 구분합니다.

## 조건별 변경 내역

| 질문 | 구분 | 항목 | 이전 | 적용 결과 |
|---|---|---|---|---|
| 1, 12–14, 82, 104 | 유지 | [운용 / 인가](../scripts/v2) | prepare → go, stop은 인가 해제, GPS 별도 유지 | GPS 선연결·상시 유지. prepare/go/stop 절차 유지. FINISH는 go만으로 해제되지 않으며 새 세션 필요. 운전자 stop은 ESTOP 회복주행보다 우선. |
| 2–8 | 변경 | [출발 준비](../src/adas_mgm/src/mgm_node.cpp) | New_Avoid_v2 사용 시 후방 LiDAR를 준비 목록에서 제외 | 전·후·좌·우 raw LiDAR 4개가 모두 0.35초 이내 정상이어야 함. 실제 카메라 영상(1초) 또는 유효한 FIXED GPS 경로(0.5초) 중 하나 필요. 차선 검출 자체는 인가 조건이 아님. |
| 9, 66 | 제외 | [독립 E-stop 준비 응답](../src/adas_mgm/launch/REAL_VEHICLE_integration_v2_drive.launch.py) | 독립 EstopRequest의 0.25초 freshness/scan_valid를 출발·정지에 사용 | 새 v2에서 독립 노드 미실행·EstopRequest 미구독. 관련 heartbeat 준비 조건 제외. 새로운 상위 ESTOP은 raw LiDAR를 직접 사용. |
| 10–11, 47–50 | 변경 | [신호 모듈 준비](../src/stack_traffic/stack_traffic/zone_supervisor.py) | 상시 실행 및 메시지 소실에 의한 별도 정지 | zone 밖에서는 인식 프로세스·카메라 OFF. 출발 시 zone 밖 heartbeat 요구 제외. zone 진입 후 시작/재시작 중에도 다른 조건이 허용하면 주행. 시작 준비 timeout 없음. |
| 15–22 | 유지 | [일반 차선 / GPS 선택](../src/adas_mgm/core/manager_step.cpp) | 차선 우선 + 신뢰도 히스테리시스 | 낮은 신뢰도 <0.35를 50회 확인 후 GPS 보완. 경로 무효면 즉시 GPS 대체. 복귀는 ≥0.70을 50회 확인. 일반 GPS 소실 시 유효 차선 즉시 선택. 양쪽 무효는 출력 0, 유효 참조 복구 시 자동 재출발. |
| 23, 25–26 | 변경 | [GPS 주행 품질](../src/adas_mgm/core/reference_safety.cpp) | fix_quality != 0이면 GPS 주행 가능 | GPS 주행은 FIXED=4만 허용. FLOAT 및 그 밖의 품질은 GPS 주행 참조로 사용하지 않음. FIXED 1개 새 유효 샘플로 복구 가능. 일반 GPS 주행 중 품질 소실은 정상 차선 대체 가능. |
| 24, 107–109 | 변경 | [FLOAT 정차 중 차선 복구](../src/adas_mgm/core/manager_step.cpp) | GPS 무효 시 단순 유효 차선으로 즉시 넘어갈 수 있음 | 낮은 차선 신뢰도 때문에 GPS가 필요했지만 FLOAT인 대기 상태에서는 ≥0.70 × 50회 후 차선으로 자동 재출발. FIXED 복구 시에는 GPS로 자동 재출발. 좌·우회전 zone 내부에서는 차선 대체 금지. |
| 27–29, 64 | 유지 | [위치 관측 / zone 안정화](../src/adas_mgm/core/manager_step.cpp) | GPS 주행과 zone에 동일한 경로 유효 판정 사용 | zone 및 센서 생존 판단은 FLOAT의 유효한 위치 관측도 허용하도록 주행 조건과 분리. 진입·이탈 각각 독립 GNSS 관측 5회. GPS 무효는 zone 이탈로 간주하지 않음. |
| 30–32, 구현 중 추가 확인 | 보류 | [회피 모듈](../src/adas_mgm/core/manager_step.cpp) | 개발 중인 New_Avoid_v2와 기존 완료/복귀 연결 | 사용자 추가 답변대로 모듈 연결 및 내부 개발 내용 유지. 확인된 회피 zone 진입으로 AVOID 활성화, zone 밖 신규 회피 금지. 내부 경로 생성·완료·GPS 복귀 알고리즘 재설계는 이번 변경에서 제외. |
| 33–36, 39–46 | 유지 | [신호 판단 / 속도 프로파일](../src/adas_mgm/core/manager_step.cpp) | 적색과 정지선, 소실 edge 기반 거리 | 적색만으로 정지하지 않음. 적색+정지선 → 접근. 소실 edge에서 1.5m 시작. v_base × clamp(remaining/1.5). 실제 정차 확인은 유효한 |v|≤0.001. 적색 해제 시 재출발, 초록 필수 아님. 5프레임 중 3개 적색, 프레임 부재는 적색 해제 표가 아님. |
| 37 | 변경 | [정지선 거리 재설정](../src/adas_mgm/core/manager_step.cpp) | 접근 중 최초 소실만 1.5m로 설정 | APPROACH 동안 유효한 정지선 재검출 → 재소실마다 1.5m로 다시 설정. STOPPED_WAIT에서는 재설정하지 않음. |
| 38, 60 | 변경 | [정지 거리 / 성공 관측](../src/adas_mgm/core/manager_step.cpp) | 정지 및 성공 관측 상한 1.0m | remaining ≤1.1m에서 목표 0. 성공 관측은 실제 정차 + 0<remaining≤1.1m. 기존 프로파일 그대로이므로 1.1m 직전까지 속도가 선형으로 0이 되지는 않음. |
| 47, 53–54 | 변경 | [신호 모듈 생명주기](../src/stack_traffic/stack_traffic/zone_supervisor.py) | GPS zone과 독립적으로 인식 프로세스 상시 실행 | 확인된 공용 turn zone에서만 인식 프로세스 실행. 이탈하면 종료하고 적색·투표·거리 이력 초기화. zone 재진입은 새 인식 프로세스로 시작. 가벼운 프로세스 관리자는 계속 실행되나 모델/카메라는 밖에서 시작하지 않음. |
| 50–51 | 변경 | [신호 메시지 소실](../src/adas_mgm/core/manager_step.cpp) | heartbeat 소실 시 새로운 정지 요구 발생 | 소실만으로 새 정지를 만들지 않음. 이미 성립한 접근/적색 정지는 보존하며, 오래된 적색 해제 메시지로 재출발하지 않음. 새 유효 상태가 있어야 해제. |
| 52, 70, 106, 113 | 제외 | [운용하지 않는 상황](../docs/MGM_STATE_MACHINE_CHANGE_NOTES.md) | 추가 예외 조건 검토 대상 | 적색 정차 상태로 zone 이탈, 신호 대기·감속 중 ESTOP 발생, 미션 zone 안에서 출발은 운용 범위 밖으로 기록. 이를 이유로 검출 무시·강제 재출발 등의 새 예외 코드는 만들지 않음. |
| 54, 지도 추가 확인 | 변경 | [좌·우회전 / Traffic Zone](../src/stack_gps/stack_gps/zones.py) | 기존 GPS 전용 범위: Path 02 전체, 주차 접근 등을 포함 | turn_zones 하나로 GPS 단독 주행과 신호 모듈 활성화 범위를 공유. 사용자 추가 답변에 따라 기존 gps_only_zones 자동 적용 제외. 새 turn_zones는 7개 경로 모두 비어 있으며 추후 실제 좌표 설정 필요. 회피·주차 zone은 유지. |
| 55–59 | 변경 | [차속 소실 중 신호 접근](../src/adas_mgm/core/manager_step.cpp) | 차속 무효로 강제 정지, 거리 적분 중지 | 실측 차속 freshness 0.2초 유지. 유효 실측 → 마지막 유효 실측(시간 제한 없음) → 세션에 실측 이력이 전혀 없으면 직전 출력 목표속도 순서로 적분. 추정값으로 실제 정차를 인증하지 않음. |
| 61, 69, 구현 중 추가 확인 | 보류 | [주차 모듈](../src/adas_mgm/core/manager_step.cpp) | PARKING zone 진입, 기존 모듈 준비/완료 연결 | zone 진입 시 PARKING은 유지. 모듈 연결과 내부 작업 유지. GPS 위주 새 주차 알고리즘은 임의 구현하지 않음. PARKING 중에도 새 ESTOP 진입 허용. |
| 62–65 | 유지 | [전체 센서 소실](../src/adas_mgm/core/manager_step.cpp) | 카메라2·전좌우 LiDAR3·GPS를 감시 | 6개 감시 센서가 모두 소실하면 SAFE_STOP. 후방 LiDAR만 살아 있어도 전체 감시 센서 소실로 판단. 하나 복구 시 별도 go 없이 회복하되 유효 주행 참조와 다른 정지 조건은 별도 확인. |
| 66–67 | 변경 | [상위 ESTOP 상태](../src/adas_mgm/core/estop_state.hpp) | 독립 E-stop 요청, AUTO_ESTOP / 기존 REVERSE_RECOVERY | 상위 SafetyState::ESTOP 추가. 진입과 회복 완료 복귀 전이를 구현. 실제 회복주행은 별도 executor 계약만 추가하고 구현 보류. /planning/estop_recovery가 현재 episode ID와 유효 경로/속도/완료를 전달하도록 연결점 제공. |
| 67–69 | 변경 | [ESTOP 복귀](../src/adas_mgm/core/estop_state.hpp) | 장애물 해제나 기존 후진 제한 시간에 따라 복귀 | 현재 episode의 신선한 회복 완료 응답이 있어야 탈출. 진입 직전 Nav/Avoid/Mission/Signal 상태로 복귀. 주차 중도 허용. 장애물이 사라졌다는 사실만으로 ESTOP을 해제하지 않음. |
| 71–79, 110 | 변경 | [ESTOP 감지 조건](../src/adas_mgm/src/estop_scan.hpp) | 독립 모듈의 별도 거리/동적 위험 로직 | 전방 ≤0.25m, 좌·우 각각 ≤0.15m, 차체 외곽 기준. 한 센서의 새 유효 스캔 3회. 센서별로 계산하며 반복 수신값/타 센서 횟수를 합산하지 않음. 0.35초 초과 또는 무효 입력은 횟수 유지, 정상 clear 관측은 0으로 초기화. |
| 80–81 | 변경 | [ESTOP 재진입](../src/adas_mgm/core/estop_state.hpp) | 지속 위험 레벨로 반복 정지/탈출 가능 | 회복 완료 후 계속 남은 조건은 해당 센서별 재진입 억제. 그 센서가 정상 clear를 보고 새로 3회 검출해야 재진입. 다른 재무장 센서의 새 조건은 진입 가능. |
| 111–112 | 변경 | [ESTOP 중 정지 우선순위](../src/adas_mgm/core/estop_state.hpp) | 전체 센서 소실 정지가 회복을 중단 | 전체 감시 센서 소실만으로 회복을 중단하지 않음. 운전자 stop과 CAN 고장은 회복보다 우선해 목표 0. 유효 회복주행 입력이 없으면 목표 0이며 가짜 후진 경로를 만들지 않음. |
| 66–67 | 제외 | [기존 자동 후진](../src/adas_mgm/launch/REAL_VEHICLE_integration_v2_drive.launch.py) | 독립 E-stop 유지 10초 후 기존 속도·시간 제한 후진 | 새 v2에서 escape_after_cycles=0 강제 및 기존 update_escape 경로 제외. 새로운 회복주행의 속도·거리·종료시간으로 과거 값을 재사용하지 않음. |
| 83–86 | 유지 | [CAN 고장](../src/adas_mgm/src/mgm_node.cpp) | 수신 이력 이후 watchdog, 지속 장애 재인가 | 0.5초 stale, 송신 3회 연속 실패, link down 감지 유지. 장애가 1초 이상이면 회복 후 go 재인가. 첫 CAN health 이전에는 watchdog 미적용. ESTOP에도 정지 우선 적용. |
| 87–89 | 유지 | [역방향 방지](../src/adas_mgm/core/manager_step.cpp) | 2.1rad / 50회 진입·해제 | 유효 GPS heading으로 |yaw|>2.1rad를 50회 확인하면 GPS 출력 정지. 정상각 50회로 해제. GPS 무효 구간은 카운터/래치 해제 근거가 아님. |
| 90–93 | 유지 | [지정 정지점](../src/adas_mgm/core/manager_step.cpp) | 실제 정차 3초, 마지막 처리 ID·시작 위치 억제 | 유효 실제 |v|≤0.001일 때만 300틱 차감. 이동/차속 무효이면 남은 시간 유지. 처음 이미 점유한 stop ID 억제 및 마지막 처리 ID 반복 방지 유지. 새 위치는 추가하지 않음. |
| 94–95 | 유지 | [일반 속도](../src/adas_mgm/launch/REAL_VEHICLE_integration_v2_drive.launch.py) | v_base 2.0m/s, MGM 목표 직접 전달 | 일반 주행 2.0m/s 유지, MGM 가속/감속 램프 추가 없음. 회피·주차 내부 속도 재설계와 새 ESTOP 회복 속도는 보류. |
| 96–97 | 유지 | [경로 완료](../src/adas_mgm/core/route_step.cpp) | 경로별 endpoint 또는 missions_complete 선택지 | 새 v2는 경로 끝 + 필요한 미션 종료를 함께 요구. 미션 실패도 종료 기록으로 취급. missions_complete만으로 끝 지점을 생략하는 기존 대체 조건 제외. |
| 98 | 변경 | [중간 CSV 전환](../src/adas_mgm/core/route_step.cpp) | 실측 차속 유효 및 실제 정차 후 다음 경로 요청 | 중간 경로는 차속 무효/이동만을 이유로 다음 CSV 요청을 막지 않음. 인가·독립 정지 사유·미션·신호 등 나머지 조건은 적용. |
| 99–100, 107–109 | 변경 | [경로 준비 대기 / 연결 구간](../src/adas_mgm/core/mgm_step.cpp) | WAIT_ACK 정지 및 연결 구간 GPS 강제, ACK 시 GPS 복귀 | 일반 구간에서는 정상 차선 주행 중 경로 변경을 병행. ACK 시 차선 모드·신뢰도 카운터 유지. GPS 사용 중에는 준비 전까지 기존의 유효한 경로 사용. 기존 참조는 원래 0.5초 freshness를 연장하지 않으며 무효가 되면 정지. ACK timeout 추가 없음. 연결 구간도 turn zone 밖이면 차선 허용. |
| 99 | 변경 | [새 경로 검증](../src/adas_mgm/core/route_step.cpp) | 요청 번호·경로 번호·새 세대 확인 | 요청 번호/경로/연결 여부 + 새 유효 GPS 세대 확인 유지. 오래된 경로 캐시는 새 경로 ACK로 인정하지 않음. 확인 전의 새 경로 기하가 조향으로 먼저 넘어가는 것을 차단. |
| 101 | 변경 | [경로 오류](../src/adas_mgm/core/manager_step.cpp) | sensor-only 정책에서 route FAULT가 출력 0을 보장하지 않음 | route FAULT를 별도 정지 사유로 포함. 카메라가 정상이어도 목표 0. 원인 수정 후 새 세션으로 초기화해야 함. |
| 102–105 | 유지 | [최종 종료 / 미션 기억](../src/adas_mgm/core/route_step.cpp) | 정차 확인 FINISH, 새 세션 초기화 | 마지막 경로는 유효 실제 |v|≤0.001을 확인해야 FINISH. 미도착 → 새 도착 순서 필요. 같은 세션의 경로 변경은 완료/실패 미션 기록 보존. 새 세션에서 초기화. |
| 구현 범위 | 보류 | [회복주행 실행기](../src/fma_interfaces/msg/EstopRecovery.msg) | 기존 후진 로직을 재활용할 수 있었음 | 이번에는 재활용하지 않음. 회복주행 실행기가 아직 없으므로 ESTOP 진입 후 목표 0으로 대기하며 자동 탈출하지 않음. 실제 회복주행 완성으로 간주하면 안 됨. |
| 구현 범위 | 제외 | [예전 지도와 호환 코드](../src/adas_mgm/launch/REAL_VEHICLE_integration_v2_drive.launch.py) | 동일 저장소에 기존 프로파일·생성 백엔드·실험용 launch 존재 | prepare/drive의 revised_v2_enabled=true 실행 경로에서 제외. 과거 로그 재현·다른 실험 프로파일용 소스 파일은 삭제하지 않음. 사용자 작업 중이던 변경은 보존. |

## 새 turn zone 설정 방법

각 경로의 `src/stack_gps/waypoints/zones_halla_reference_path_XX.yaml`에서 `turn_zones`에 실제 경계를 입력합니다. 기존 `gps_only_zones`를 다시 사용하는 방식은 아닙니다. 회전 구간과 Traffic Zone은 같은 정의를 사용합니다.

```yaml
# 예시 형식입니다. 실제 좌표/인덱스는 아직 지정하지 않았습니다.
turn_zones:
  - zone_id: 100          # 해당 파일의 기존 미션 zone ID와 겹치지 않는 값
    index_range: [START_INDEX, END_INDEX]
# index_range 대신 start: {lat: ..., lon: ...}, end: {lat: ..., lon: ...}도 가능
```

선택한 CSV 안의 인덱스/좌표만 사용해야 합니다. 중복 ID·범위 밖 인덱스·잘못된 형식은 로딩 단계에서 거부합니다. 설정은 새 prepare 세션에서 읽습니다. GPS 물리 연결을 끊을 필요는 없습니다.

## ESTOP 연결 계약

MGM이 `MgmState.estop_request_id`를 발행합니다. 향후 회복주행 실행기는 `/planning/estop_recovery`의 `EstopRecovery`로 같은 `request_id`, 실제 생성 시각 `reference_stamp`, 유효한 1점 경로, `v_suggest`, 완료 `done`을 보내야 합니다. 잘못된 episode ID, 오래된 입력, 잘못된 기하를 주행/완료로 인정하지 않습니다. 입력 freshness는 현재 raw LiDAR 계약과 같은 0.35초로 설정했습니다. 이는 회복주행 지속시간이나 동작 궤적을 정한 것이 아닙니다.

전방/좌우 스캔의 장착 위치·yaw·FOV·range offset은 기존 `lidar_fusion_v2/config/fixed_geometry.yaml`을 읽습니다. 차체 외곽은 현재 차량 모델의 전방 0.760m, 후방 0.090m, 반폭 0.310m 기준입니다. 회복 후 센서별 재무장, 운전자/CAN 정지 우선권을 적용합니다.

## 검증

- 빌드 성공: `fma_interfaces`, `adas_mgm`, `stack_gps`, `stack_traffic`, `stack_avoid_v2`.
- `ctest --test-dir build_v2/adas_mgm --output-on-failure`: **28/28 통과**.
- `revised_v2_test`: **51개 확인, 실패 0**. 새 정책의 FLOAT/차선/zone/신호/ESTOP/CSV 경계 포함.
- Python: **368개 통과, 3개 skip**. GPS·신호·준비·런처·프로세스 생명주기 검증.
- `revised_v2_ros_smoke.py`: 격리 ROS domain 196에서 **12개 합성 입력 시나리오 통과**. 실제 장치는 실행하지 않음.
- `scripts/v2 check`, Python/셸 문법, `git diff --check` 통과.
- HTML 37개 행과 모든 파일 링크 확인. [이번 작업 파일 목록](V2_STATE_MACHINE_CHANGED_FILES.txt).
- 실제 차량·실제 센서·CAN 송신 및 제동 거리 측정은 하지 않았습니다.

## 관련 파일

- [실행용 런북](../RUN_BOOK_FINAL.md)
- [인터뷰 ESTOP 메모](MGM_STATE_MACHINE_CHANGE_NOTES.md)
- [HTML 보고서](V2_STATE_MACHINE_CHANGE_REPORT.html)
- [변경 전 상태 표 — 비교용 보관본](V2_MAIN_STATE_TRANSITIONS.html)
