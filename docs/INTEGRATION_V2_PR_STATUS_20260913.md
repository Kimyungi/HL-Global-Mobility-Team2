# Integration v2 — PR 검토 기준 (2026-09-13)

사용자 지정 PR 대상 브랜치는 `main`이다. 초기 integration v2 구축 커밋부터 현재 변경과
현장 로그까지 함께 검토한다. PR은 아직 merge하지 않았으며 로컬 main checkout과는 별개다.
이 문서의 검증 결과가 과거 문서의 미빌드 표기나 전체 통과 수치보다 우선한다.
현재 PR은 전체 회귀 검증이 끝나지 않은 Draft다.

## 현재 변경

- 한라대 경로는 `(01 또는 02) → 03 → 04 → 05 → (06 또는 07)`이다.
  시작/종료 경로를 실행 전에 명시하며 최근 현장 선택은 01/07이다.
  업로드된 경로별 CSV/Zone, 경로 순서·종점·새 reference 응답을 연결했다.
- GPS station은 최초 최근접 위치에서 시작해 저장 station 기준
  `abs(v_ref) * sample_time * 2` 범위로 갱신한다. station +2.5m preview의
  두 점 가중치 중 하나가 90% 이상이면 해당 점, 그 외에는 보간한다.
- 카메라는 피팅 중심선의 차량 투영 station에서 +2.5m 목표점 하나를 발행한다.
  MGM의 현재 입력/최종 출력 계약은 1점이며 유효성 gate를 통과해야 한다.
- 비정지 속도 크기는 v_base(현재 1m/s)로 통일했다. 정지 제어는 기존 정책이다.
  일반 회피는 기본 OFF이며 LiDAR E-stop만 제외하는 별도 시험 런처가 있다.
  일반 런처의 LiDAR E-stop, 외부/운전자/CAN/reference 정지는 유지한다.
- 주차 Zone 진입 5회 확인 즉시 ACTIVE/PARKING이다. 주차 상태에서 준비하며 정지 대기,
  현재 요청 ready→ACTIVATE ack/유효 reference 이후 실행한다. 정상 복귀는 done 또는
  현재 CSV 종점이며 미완료 종점은 ROUTE_END=10 실패로 기록한다. Zone 이탈은 종료하지 않는다.
- 4-LiDAR 드라이버 연결·장치 링크 복구, NMEA 읽기 진단 도구, 한라대/용인 런북을 포함한다.
- 현재 dump는 v19다. 9월 12일 현장 run은 v18이며 당시 빌드로 재생해야 한다.
  신호 정지 거리 seed는 1.5m, 잔여거리 목표는 1.0m 그대로다.

## 검증 결과

| 검사 | 결과 |
|---|---|
| v2 colcon build | 13개 패키지 성공 |
| 설치 prefix/메시지/정책 검사 | 성공 |
| 관련 Python 전체 | 471 passed, 3 skipped; 기존 SciPy/NumPy 버전 경고 1건 |
| 신규 즉시 주차 코어 | 75 checks 통과 |
| 즉시 주차 ROS mock | 12개 연결 확인 통과; localhost domain 178 |
| 전체 CTest | **11/20 통과, 9개 실패** |

실패한 CTest는 `fixed_speed_test`, `avoidance_disabled_test`, `route_sequence_test`,
`stabilization_test`, `reference_safety_test`, `zone_manager_test`,
`mission_preparation_test`, `mission_zone_search_test`, `manager_state_test`다.
이 시험들의 공통 `manager_test_fixture.hpp`에는 2점 입력이 남아 있고 현재 core는
1점 입력을 요구한다. 오래된 속도/기하 기대값 등을 포함한 전체 회귀 정리가 남아 있다.
모든 실패가 fixture 변경만으로 해소된다고 검증한 상태는 아니다.

Python 재현은 `scripts/v2 test`의 Python 블록과 동일한 패키지/test 경로를 사용했다.
`scripts/v2 test`는 전체 CTest 실패에서 종료되므로 현재 전체 통과 명령으로 제시하지 않는다.
이번 PR 준비 중 센서·차량·CAN bridge를 실행하지 않았다.

## 실제 주행 기록과 남은 확인

9월 12일 run은 `drive_logs/v2_no_estop_20260912_223321_282247`에 CSV·메타데이터
12개 파일(약 15MB)로 포함했다. 약 49.2GB rosbag DB와 약 609MB raw dump는 로컬 보관이며
Git에 추가하지 않았다. manifest와 SHA256SUMS에 원본 정보가 있다.

그 run은 주차 정책 변경 전 기록이다. T자·평행 모두 PREPARE 진입 약 1.3초 뒤 Zone 이탈로
취소됐으며, 지금의 즉시 ACTIVE 정책을 실차 검증한 기록이 아니다.
소스 기준점은 `8697bcb`와 당시 미커밋 변경이며 현재 PR 전체와 동일한 실행 버전은 아니다.

GPS station의 이동 탐색 범위는 MGM의 최종 목표속도에 의존한다. 목표속도 0인 상태에서
조이스틱 등으로 실제 차량을 움직이면 station 갱신이 제한되는 문제는 후속 작업이다.
주차 공간 검출/경로 생성 성공률과 새 주차 종료 정책의 현장 검증도 남아 있다.
