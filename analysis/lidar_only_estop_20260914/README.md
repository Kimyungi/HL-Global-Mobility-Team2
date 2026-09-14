# 독립 라이다 E-stop + 무장애물 GPS 복귀

사용자 요청에 따라 v2 base manager의 회피 TTC/avoidable 기반 E-stop을 제거했다.
- manager_step: E-stop은 독립 s.auto_estop만 반영(기존 주차 권한 유지).
- mgm_step: base manager가 재사용하는 속도 계산에서도 TTC 정지를 제외.
  과거 legacy backend의 TTC 규칙은 과거 시험 호환용으로 유지.
- route_step: 회피 TTC가 CSV 전환을 추가로 막던 조건 제거.
- 회피 TTC/avoidable 원본 진단값과 독립 stack_estop 소스/문턱은 유지.
- 로그 safety=AUTO_ESTOP enum은 메시지 호환을 위해 유지하지만 v2에서는
  독립 라이다 E-stop 입력으로만 이 상태에 진입한다.

추가 사용자 지시에 따라 장애물이 없고 회피 참조가 비거나 오래됐으며 GPS가
유효하면 잔류 회피를 해제하고 GPS_BACKUP/GPS_ONLY_NAV로 전환한다.
차선이 없어도 GPS를 사용한다. 신호 해제나 maneuver_done을 요구하지 않는다.
진행 중인 후진을 GPS 전진으로 덮지 않으며, 유효한 회피 곡선의 정상 복귀는
기존 방식이다. GPS도 유효하지 않으면 주행 참조를 만들어내지 않는다.
현재 장애물이 여전히 검출되고 회피 제어점이 없다면 reference_motion_blocked로
속도 출력을 보류한다. 이것은 TTC/avoidable E-stop과 별개인 빈 참조 처리다.

이력: TTC 분기는 Git 8697bcb9(2026-09-11, author Xanadu)의 통합 코드에 존재했다.
그 이후 obstacle_detected && !avoidable 정지는 이번 작업 트리에서 추가된
조건이었다. 이 추가 조건 및 v2의 기존 TTC 정지 결합을 이번 변경에서 제거했다.
Git 작성자 표기는 커밋 메타데이터이며 최초 설계자의 신원까지 증명하지 않는다.

검증: scripts/v2 build 13 패키지 성공, CTest 25/25 통과,
scripts/v2 check 및 git diff --check 통과.
lidar_only_estop_test가 짧은 TTC/avoidable=false의 주행, 독립 E-stop 발동/해제,
빈 참조와 E-stop 구분, 무장애물·무차선의 GPS 대체, GPS까지 없을 때 보류,
신호 해제 중 독립 E-stop 유지, CSV 전환을 검증한다.
RUN_BOOK_FINAL.md 및 HTML의 TTC 표시도 현재 정책으로 정정했다.
실차 실행/인가 없이 오프라인 검증했다. 기존 통합 런처 재시작부터 적용된다.
