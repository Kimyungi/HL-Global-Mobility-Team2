# v2 장애물 회피: main 동작 복원 메모

2026-09-13. 비교 기준은 main `c76f287`, 수정 전 integration/v2_main `137d49e`다.
사용자는 main에서는 의도대로 회피했으나 v2에서는 조향 부족 → E-stop → 잠깐 해제/주행 → 재정지를 반복했다고 보고했다.

## 코드에서 확인한 차이와 수정

`stack_avoid`의 gap 탐색/목표 생성과 `bridge_dspace`의 CAN 전송 로직은 비교한 두 버전에서 동일하다.
주요 차이는 MGM이 회피 목표를 조립하고 속도와 종료를 결정하는 방식이다.

| 항목 | main | 수정 전 v2 | 이번 수정 |
|---|---|---|---|
| 회피 목표 조립 | 목표를 20점으로 보간 | 먼 목표 1점을 그대로 사용 | main의 첫 CAN 점과 동일한 단일점 |
| CAN v5 기준점 | 첫 점만 전송 | 첫 점만 전송 | x/20, y/20, yaw=atan2(y,x), curvature=0 |
| 진입 blend | 10틱 | 회피 진입 즉시 교체 | 10틱 복원 |
| 회피 속도 | 제안 속도와 .6m/s 상한; 좁으면 .2 | 양의 목표를 1m/s로 정규화 | main 상한 복원 |
| 가감속 | +.5 / -1.5m/s² | 일반 주행 목표는 ramp 우회 | 회피와 Navigation 복귀까지 ramp 적용 |
| 회피 종료 | maneuver_done 또는 최대 1200틱 | 소실 200틱 | 완료/최대 시간 복원 |
| GPS 복귀 hold | 회피 종료부터 300틱 | 소실부터 300틱 | 종료부터 300틱 복원 |

예를 들어 provider 목표가 (3.76, .66)이면 main의 첫 CAN 기준점은
(.188, .033), yaw 약 .173762rad이다. 수정 전 v2는 (3.76, .66), provider yaw=0을 전달했다.
현재는 blend 완료 후 main과 같은 점/방향각을 전달하며 **v2 reference count=1 계약은 유지**한다.
20점 전체를 CAN으로 보냈다는 과거 주석은 실제 v5 bridge 구현과 달라 정정했다.

속도 증가와 기준점/방향각 변경은 보고된 증상의 유력한 회귀 원인이다.
다만 실제 차량 조향 부족의 인과관계를 확정한 실차 로그는 없다.
TTC 계산식은 main과 동일하므로 새로 생긴 TTC 계산 버그로 분류하지 않는다.
고정 간격에서 실제 속도에 따른 TTC 정지/해제와 1m/s 재명령이 반복을 증폭할 수 있다.
이번 변경은 E-stop을 해제하거나 문턱을 완화하지 않는다.

## 유지하는 v2 동작과 경계

- 병행 Manager, 일반 LINE/GPS 목표점, 주차/신호/외부 정지 우선순위 및 정면 LiDAR 좌표 수정 유지.
- invalid/stale/empty/다점 입력은 여전히 정지한다. NaN 속도 제안을 cap으로 숨기지 않는다.
- 긴급 정지는 ramp 없이 즉시 0. 해제 후 회피 가속은 100Hz 기준 첫 틱 .005m/s다.
- 장애물 소실만으로 종료하지 않는다. 최대 시간은 정지 중에도 진행하며 0이면 제한을 끈다.
- Recovery 진행 중 완료/최대 시간으로 회피를 종료하지 않는다.
- Nav 복구와 장애물 관측이 겹치면 LiDAR fallback을 실제 회피 episode로 유지한다.
- CLEAR_CONFIRM enum 값은 호환 목적으로 남지만 이번 회피에서 진입하지 않는다.
- dump 의미 버전은 v21. 기존 dump는 해당 버전 빌드로 재생한다. ROS/CAN 메시지 형식은 동일하다.

## 검증

- `scripts/v2 build`: 13 packages 성공.
- CTest: 21/21 통과. 신규 `avoid_main_compat_test`는 기준점/yaw, 양쪽 회피,
  진입 blend, 반복 시 중복 축소 방지, .6/.2 상한, E-stop/TTC/외부 정지 및 재출발,
  잘못된 reference, 완료/timeout/GPS hold와 Navigation 유지 조건을 검증한다.
- 수정 전 CTest는 11/20이었다. 기존 공통 fixture와 ROS smoke의 2점 입력을 현재 v2의
  유효 입력 1점으로 정정하고, 회피 속도/종료 기대값을 복원 기준에 맞췄다.
  다점 입력 차단은 별도 부정 테스트로 유지한다.
- Python: 493 passed, 3 skipped.
- localhost ROS: `manager_ros_smoke.py`, `parking_entry_ros_smoke.py` 통과.
- 전체 `scripts/v2 test`는 기존 `route_sequence_ros_smoke.py`의 GPS `n_points:=20`
  설정이 현재 단일점 계약과 맞지 않아 실패한다. 시험적으로 단일점 fixture로 바꾸어도
  01 종점 인계에서 SAFE_STOP_ROUTE_SEQUENCE(512)가 발생했다.
  회피와 별개인 route fixture 개편은 이 PR에 포함하지 않았으며 전체 통합 성공으로 보고하지 않는다.

실차 검증은 남아 있다. 같은 장애물 배치에서 provider 목표, TargetRef 첫 점/yaw/v_ref,
실제 속도, TTC/E-stop, dSPACE str_ref와 실제 str을 함께 기록해야 한다.
PC 기준점 복원 여부와 실제 조향 추종 여부를 구분해 확인한다.
