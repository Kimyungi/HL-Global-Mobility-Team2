# New_Avoid_v2 로컬 반영 — 2026-09-15

## 적용 결과

- 작업 폴더: `/home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main`.
- `integration/v2_main`을 PR #102 머지 커밋 `432ab19`까지 fast-forward했다.
- 기존 로컬 수정은 stash 및 별도 압축 백업 후 재적용했다. Path 04 단독 선택,
  GPS map 시각화, GPS 복귀 중 1m/s 상한과 이전 회피 시험 코드를 보존했다.
- 사용자 RUN_BOOK_FINAL 3번 명령은 기본 `avoid_v2_enabled=true`로
  `stack_avoid_v2/avoid_v2_node`와 MGM의 `/avoid_v2/plan` 입력을 선택한다.
  기존 회피 노드는 실행 조건이 false다. `avoid_compute_backend:=native`는
  이 새 C++ 플래너를 이전 fixed_goals로 바꾸지 않는다.
- 시작 로그와 세션 `avoid_planner_mode.txt`에 실제 선택인 `New_Avoid_v2`를 기록한다.
  새 모드에서는 이전 Python/native 회피 모듈의 사전 로딩도 생략한다.
- 이전 비교 실행은 `avoid_v2_enabled:=false`를 명시한다. 기존 두 로컬 시험 스크립트도
  이 인자를 추가해 명시된 fixed_goals 시험 동작을 유지했다.
- 새 C++ 패키지는 빌드 종류 미지정 시 Release로 빌드한다. 최적화 없는 첫 빌드의
  S자 시험에서 계산 기한 초과를 확인했고, Release 재검증에서는 이 시험이 통과했다.
  경로 알고리즘과 코어 20ms/ROS callback 35ms 제한은 변경하지 않았다.

## 최종 검증

| 확인 | 결과 |
| --- | --- |
| `scripts/v2 build` | 14개 패키지 완료, 새 플래너 Release |
| `scripts/v2 check` | V2_INSTALL_READY, 13개 package prefix, Zone 확인 5/5 |
| MGM CTest | 27/27 통과 |
| prepare·통합 launch·전방 보정·시각화 pytest | 52/52 통과 |
| GPS pytest | 180/180 통과 |
| Release C++ 플래너/MGM 격리 ROS 연결 | 11개 단계 통과 |
| Release 회피 코어 CTest | 8/9 통과, 기존 동일 차량 시나리오 실패 유지 |

격리 ROS 연결은 실제 플래너와 MGM 실행 파일에 합성 센서를 공급한다. GPS 5회 후
회피 진입, 목표점/속도 전달, 플래너 SIGSTOP 시 속도 0, 재개, 후진 중 전진 참조 차단,
Zone 이탈 후 회피 유지, 스캔 소실/HOLD 및 복구를 확인했다. 실차 CAN 주행 시험은 아니다.

동일 차량 시나리오는 78회 중 일반 주행 54회 완주, 진단용 예상 HOLD 18회,
직선 중앙 장애물 배치 6회 미완주/HOLD다. PR #102에 기록된 미해결 결과와 같으며
실패를 통과로 변경하거나 시험에서 제외하지 않았다.

## 실제 재기동 상태

기존 시험 런처에는 SIGINT로 정상 종료 절차를 요청했고 관련 프로세스 종료를 확인했다.
최신 사용자 prepare 명령은 실행했으나 라이다 사전 검사에서 종료했다.
확인 당시 `/dev/lidar_front`, `/dev/lidar_rear`, `/dev/lidar_left`, `/dev/lidar_right`,
`can0`와 USB serial 장치가 없었으며, GPS 상주 서비스는 유지 중이나 FIX=0이었다.
따라서 실제 센서/MGM/CAN 스택 재기동 확인은 완료하지 못했다. 출발 인가는 보내지 않았다.
장치 연결 후 RUN_BOOK_FINAL 1~3번 절차와 동일한 prepare 명령을 사용한다.

## 백업과 원시 로그

백업 폴더: `/home/sangmin/Desktop/v2_local_backup_20260915_205458`.
`local_changes.tar.gz`, `tracked.patch`, `head.txt`, `untracked.sha256.json`과
Git stash `pre-New_Avoid_v2-local-20260915`를 보존했다. 기존 untracked 파일 115개는
누락이 없으며, 의도적으로 수정한 두 비교 시험 스크립트 외에는 해시가 일치했다.
큰 analysis 파일은 제자리에 보존하고 해시를 기록했다.

같은 백업 폴더에 `build_release.log`, `mgm_ctest.log`, `launch_tests.log`,
`gps_tests.log`, `avoid_ctest_release.log`, `wall_mgm_integration_release.log`,
`prepare_check.log`를 보관했다. 미최적화 최초 시험 기록도 별도로 남겼다.
