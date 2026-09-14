# 회피 변경의 E-stop 영향 검토

현재 작업 트리와 HEAD 8932df4를 비교하고 현재 빌드의 mgm_core로 오프라인 검증했다.
누적 변경 전체와 최근 목표점 안정화/신호 해제 변경을 구분했다. 이번 검토에서
제어 코드, 설정, 실차 프로세스는 변경하지 않았다.

## 결과

1. `src/stack_estop`은 HEAD 대비 변경 없음. 통합 런처의 독립 E-stop 거리도
   정적 ON 1.20m, OFF 1.35m/3회 정상 스캔, 동적 거리 1.35m로 유지된다.
   거리는 전방 LiDAR를 회전 변환한 x 기준이며 정적 통로 반폭은 0.30m다.
   동적 판단은 거리 이외의 움직임/트랙 조건도 필요하다. 최종 노드 출력은
   scan timeout OR static stop OR dynamic stop이다. 3.5m 검출 확대는
   stack_avoid 설정이며 독립 E-stop 탐지 거리를 3.5m로 확대한 것이 아니다.

2. MGM의 AUTO_ESTOP에는 누적 회피 변경으로 아래 조건이 추가됐다.
   `회피 활성 && lidar_valid && obstacle_detected && !avoidable`.
   HEAD에서는 해당 회피 분기가 `TTC < 1.3s`만 검사했다. 또한 회피 진입에서
   avoidable 조건을 제거했으므로 경로가 없는 장애물도 회피 대응/정지를 받는다.

3. stack_avoid의 avoidable은 `유효 점 있음 && TTC >= 1.5s`다.
   따라서 현재 장애물이 검출된 회피 구간에서는 경로가 있어도 TTC<1.5s이면
   추가 조건으로 AUTO_ESTOP이 걸린다. MGM의 1.3s만 보면 실제 정지 문턱을
   잘못 해석하게 된다. 실속도 2m/s 가정에서는 앞범퍼 gap<3.0m에 해당한다.
   독립 LiDAR 정지 입력이 false여도 이 MGM 정지는 발생한다.

4. 면 기반 후보/복귀 거리/방향 안정화는 독립 E-stop 코드를 바꾸지 않았지만,
   경로 존재 여부를 통해 avoidable 및 MGM 정지 발생/해제에 간접 영향을 준다.
   경로 회복과 TTC>=1.5s를 함께 만족하면 해당 정지가 해제되고 속도는 램프로
   재개된다. 경로가 회복돼도 TTC가 짧으면 정지가 유지된다.
   무경로+검출이면 AUTO_ESTOP과 reference_motion_blocked가 동시에 참일 수 있다.

5. 최근 신호 해제에서 장애물이 사라지면 잔류 회피 상태를 지우므로 그 상태에
   종속된 참조 보류는 GPS로 교체되어 사라진다. 독립 s.auto_estop=true 또는
   현재 장애물의 짧은 TTC는 신호 해제로 지워지지 않음을 재현했다.
   주차 미션의 일반 E-stop 마스킹, CAN 래치, 후진 복구는 각각 기존 분기다.

## 현재 코어 재현 (2m/s, 센서 정상, AUTO_ESTOP 열은 MGM 결과)

| 입력 | 경로 | TTC | AUTO_ESTOP | v_ref |
|---|---|---:|---|---:|
| gap 3.49m | 있음 | 1.745s | false | 2.0 |
| gap 3.00m | 있음 | 1.500s | false | 2.0 |
| gap 2.99m | 있음 | 1.495s | true | 0 |
| gap 2.61m | 있음 | 1.305s | true | 0 |
| gap 3.49m | 없음 | 1.745s | true | 0 |
| 위 무경로 후 경로 회복 | 있음 | 1.745s | false | 0.005부터 |
| gap 2.99m에서 경로만 회복 | 있음 | 1.495s | true | 0 |
| 독립 LiDAR E-stop=true | 있음 | 1.745s | true | 0 |
| 신호 해제+장애물 사라짐+LiDAR E-stop=true | GPS 있음 | 1.745s | true | 0 |
| 신호 해제+장애물 여전히 검출 | 있음 | 1.495s | true | 0 |

검증은 각 조건을 분리한 합성 입력이다. 실제 LiDAR 기하에서 gap3.49/2.99m의
경로가 생성된다는 증명이나 실차 정지거리 측정이 아니다. `check.cpp`가 실제
빌드된 libmgm_core.a를 호출하고 12개 assertion이 통과했다. `results.csv` 참조.

추가 검증: stack_estop+stack_avoid 테스트 182개 통과(32+150).
운영 정적 설정 1.20/1.35m로 controller를 별도 확인: 1.20 정지,
1.30 유지, 1.35 3회 후 해제, timeout 시 재정지 통과.

2m/s에서 회피 인지 시작 3.5m와 유효 TTC 경계3m 사이 차이는 0.5m/0.25초다.
해당 통로에서 계속 검출되는 경우의 조건 차이이며 실제 회피 완료 시간은 아니다.
독립 정적 E-stop 1.20m는 런처 주석상 1m/s 조정값이 그대로 남아 있다.
2m/s에서 충분한 실제 제동 거리인지 이번 오프라인 시험으로 확인한 것은 아니다.

근거:
- src/adas_mgm/core/manager_step.cpp:390 AUTO_ESTOP 결합
- src/stack_avoid/stack_avoid/node.py:293 avoidable 정의
- src/adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py:568 운영 거리
- src/stack_estop/stack_estop/node.py 거리 히스테리시스 및 최종 OR
- src/adas_mgm/src/mgm_node.cpp:998 유효 독립 estop 입력 구성

재실행:
```
g++ -std=c++17 -Isrc/adas_mgm analysis/avoid_estop_impact_20260914/check.cpp build_v2/adas_mgm/libmgm_core.a -o /tmp/fma_avoid_estop_impact_check
/tmp/fma_avoid_estop_impact_check
```
