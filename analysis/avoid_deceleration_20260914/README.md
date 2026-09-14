# 회피 감속 코드 확인 및 거리 계산 — 2026-09-14

운영 코드/설정을 변경하지 않고 현재 v2 소스와 libmgm_core.a를 확인했다.

- stack_avoid/node.py: v_suggest는 target_speed=2.0. 거리 연속 함수가 아니다.
- narrow_gap은 obstacle_detected && !points. 현재는 경로 생성 실패를 뜻한다.
- MGM 일반 회피 요청: v_suggest, narrow이면 v_narrow=.2, v_avoid=2.0 상한.
- 일반 요청 감소는 a_down=1.5, 10ms당 v_ref .015 감소.
- TTC<1.3 또는 (검출 장애물 && !avoidable)은 AUTO_ESTOP.
- 인지 avoidable은 유효 목표점 && TTC>=1.5이므로 2m/s로 1.99m 검출 시 false.
- 경로 없음은 reference_motion_blocked도 적용된다. 즉 일반 narrow .2 요청보다
  긴급/경로 정지가 우선한다.
- immediate_stop은 명령 램프를 건너뛰고 v_ref=0. 실제 속도가 즉시 0이 되거나
  실제 감속도가 1.5라는 뜻이 아니다. 이 코드만으로 긴급 제동 실거리를 확정할 수 없다.

## 네이티브 코어 대조

check.cpp를 현재 libmgm_core.a에 링크해 하드웨어 없이 실행했다. 5개 확인 통과.
정상 TTC/유효 경로 요청 2.0, 거리1.99m/실속도2.0 요청0(immediate/AUTO_ESTOP),
경로 없는 narrow 요청0(immediate/reference hold)을 확인했다.
현재 인지에서는 나오지 않는 조합(유효 경로+narrow+정상 TTC)을 만들어 일반
감속 분기만 분리했을 때 첫10ms 요청1.985, 1.2초 후 .2를 확인했다.

## 조건부 재계산

차량이 실제로 1.5m/s² 감속을 따른다고 가정한다. 2→.2에 1.2초, 1.32m가 필요.
2m에서 일반 감속을 시작한다는 가정이면 .76m에 도달할 때 속도는 .529m/s.
감속 시작 지연100ms이면 .938m/s, 200ms이면1.217m/s다.
이때 거리/순간속도는 각각1.436/.810/.625초이며, 계속 감속하는 동안의 실제
충돌까지 남은 시간과는 다르다.

.76m에서 처음 감속한다면 그 거리를 이동하는 데 .459초, 도달속도1.311m/s.
같은 감속도로 완전 정지는1.333m가 필요하다. 이는 일반 램프를 이상적으로
추종한 직진 계산이며, 실제 emergency-stop 제동/회피 궤적 추종 결과가 아니다.

따라서 ".76m에서는 이미 .2m/s라 여유가 있다"고 볼 수 없다. 현재 정상 설정은
회피 상한도2.0이고 TTC 정지가 먼저이므로 실속도 감소는 VehicleVector 로그와
실제 제동/조향 지연을 측정해야 확정된다. 숫자: calculation.json.
