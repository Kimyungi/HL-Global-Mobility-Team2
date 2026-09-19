# 2026-09-14 21:03 주행: 회피 Zone 진입 후 정지 분석

분석 대상: `drive_logs/v2_20260914_210304_376766`, Path 04 단독 실행. 모든 시각은 KST.

원인은 두 단계다. Zone 진입 전에 공통 USB 허브가 끊어져 GPS·IMU·CAN 입력이 중단됐다. GPS 위치는 복구됐지만 IMU 방향 정렬이 사라졌고, 그 상태로 AVOID_ACTIVE에 진입하면서 회피 경로가 생성되지 않아 정지가 지속됐다.

| 시각 | 확인한 사건 |
| --- | --- |
| 21:04:29.822 | USB 상위 허브 `3-2` 및 하위 CAN, IMU `3-2.1.4`, u-blox GPS `3-2.2.1` 동시 disconnect |
| 21:04:30.267–35.907 | MGM이 선택한 GPS reference가 stale/invalid, 약 5.64초 |
| 21:04:35.907 | GPS reference 복구. IMU 재연결로 heading offset 폐기, 재정렬 대기 |
| 21:04:52.468 | 같은 상위 USB 허브 재단절; GPS·IMU·CAN 함께 분리 |
| 21:04:52.857–55.707 | GPS reference stale/invalid, 약 2.85초 |
| 21:04:55.707 | GPS reference 복구, 차량 방향은 접선 fallback |
| 21:04:56.407 | tick 11188: AVOID_ACTIVE 진입, source GPS(1) → AVOID(2), reference invalid로 정지 유지 |
| 21:04:56.474 | avoid: `fresh GPS pose and waypoint station window required` |
| 21:06:06.688 | 기록 종료까지 회피 경로 0점·방향 무효·정지 유지 |

Zone 시작은 CSV state=4의 idx 55이고 실제 전이 시 idx는 57이다. 5개 독립 GPS 샘플 확인 뒤 전이하는 조건과 일치한다. 두 번의 허브 단절은 Zone 전이보다 먼저 발생했다.

## 로그 및 코드 대조

- `kernel_usb.log`: 22/28/35행 첫 허브·IMU·GPS 분리, 133/140/147행 두 번째 분리. USB 오류 `-71`, 재열거 및 CAN 오류도 기록됨. GPS 안테나 수신 불량만으로 설명되는 현상이 아니다.
- `log_v2/ros/python3_83672_1789387384830.log`: 21:04:35.907 IMU 재연결에 따른 오프셋 폐기. 진입 이후 FIXED, 위성 12개, fix age 약 0.1초지만 헤딩은 접선, IMU는 미정렬.
- `lateral.csv`: 진입 이후 703개 행 모두 quality=4(RTK FIXED), heading_src=접선, fix_age 최대 0.137초.
- `inputs.csv`: 진입 이후 7,029개 snapshot 모두 gps_valid=1, gps_heading_valid=0, avoid_path_n=0, avoid_maneuver_done=0. gps_valid 단독으로 freshness를 판단하면 안 된다. 앞선 USB 단절 동안에도 저장된 gps_valid는 1이지만 선택 reference는 stale였다.
- `replay.csv`: 진입 이후 AVOID(2) 선택, ref_valid=0, reference_motion_blocked=1, v_ref=0. 차량은 진입 직전 USB 단절 영향으로 이미 정지에 가까웠으며 Zone 진입이 최초 감속 시점은 아니다.
- `src/stack_gps/stack_gps/node.py:555`: IMU 재연결 시 heading 정렬을 초기화한다. 재연결 후 yaw 기준이 달라질 수 있어 이전 offset을 버리는 동작이다.
- `src/stack_avoid/stack_avoid/path_io.py:96`: HEADING_TANGENT이면 GPS pose와 waypoint window를 비운다. 실제 차량 방향과 경로 접선은 다르므로 현재 회피 생성기의 유효 입력으로 인정하지 않는다.
- `src/adas_mgm/core/manager_step.cpp:294`: Zone 진입 시 AVOID_ACTIVE를 선택한다. 회피 경로 준비 여부는 진입 조건에 없다.
- `src/adas_mgm/core/reference_safety.cpp:46`: 선택 경로가 무효이면 속도를 0으로 만들고 정지시킨다.

따라서 위치 수신이 돌아와도 방향 미정렬 → 회피 경로 없음 → 정지 → 주행을 통한 방향 재정렬 불가라는 복구 제약이 남았다. Zone 진입이 GPS 장치를 끄는 증거는 없으며, 진입 뒤 지속된 장애는 실제 차량 방향과 회피 reference 부재다.

## 확인 범위와 후속 조치

공통 USB 연결 단절은 커널 기록으로 확정했다. 허브 전원, 상위 케이블 접촉, 허브 자체 또는 전기적 간섭 중 어느 것이 물리 원인인지는 이 로그로 구분할 수 없다. 공통 허브 연결의 안정성을 먼저 확인하고, 재연결 뒤 실제 차량 heading이 유효하게 복구되는 절차를 점검해야 한다. 접선을 실제 heading으로 허용하는 변경은 원인 해결로 볼 수 없다.

코드 측 후속 검토 지점은 IMU 재연결 뒤 heading 복구와 회피 진입 준비 상태 표시다. 현재의 `fresh GPS pose and waypoint station window required` 문구는 위치 소실과 heading 무효를 구분하지 못한다.

검증: raw dump ABI 확인 후 18,217 snapshot 디코딩 및 오프라인 core_replay 수행. 원본 transitions.csv의 모든 전이 행에 대해 공통 14개 필드가 replay와 일치했다. 주행 코드·런처·런북은 수정하지 않았다.
