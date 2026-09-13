# adas_mgm — Decision 계층 (10ms MGM 루프)

> 현재 주차: [Zone 진입 즉시 PARKING](../../docs/MGM_PARKING_ENTRY.md).
> 탐색 중 현재 CSV의 GPS를 추종하고 ready 후 주차 제어로 인계한다. 완료 또는 현재 CSV 종점에서 복귀하며, 종점 실패는 다음 CSV로 자동 전환한다. 아래 과거 PREPARE 주행 정책보다 우선한다.

> **6차 단일 기준:** [MGM_MBD_STATE_MACHINE_SPEC.md](../../docs/MGM_MBD_STATE_MACHINE_SPEC.md). 현재 MBD 정본은 병행 Top/Nav/Avoid/Signal/Safety/Mission이며 legacy 5-state byte는 호환 projection이다.
> Zone 확인은 독립 GNSS sample이며 0=미설정. Parking 제한 -1, Recovery OFF 유지. 현재 bus/dump는 v15(연속 경로 확장, 6차는 v12)이며 이전 버전 설명/시험 절차는 역사적 비교 범위다.
> 실제 운용 전 Zone/Parking/후방 corridor calibration과 현장 검증이 필요하다.

## 2026-09-11 공통 베이스 상태 머신

C++ `backend=core`는 Navigation/Avoidance/Traffic/Safety/Mission 병행 Manager를 기본 사용합니다.
Mission은 GPS의 `MISSION_ZONE` entry에서 `MISSION_PREPARE` 요청을 latch합니다.
Zone 밖에서도 탐색을 유지하며 현재 요청의 ready 이후에만 `MISSION_ACTIVE`로 제어권을 넘깁니다.
기존 GPS 전용·주차 구간을 재사용하고, 명시적 Zone ID/Mission ID는 GPS `zones_file`에서 설정합니다.
확정된 실차 Mission 경계는 별도로 설정해야 하며 임의 좌표는 추가하지 않았습니다.
Zone/Manager 구조는 [설계 문서](../../docs/MGM_BASE_STATE_MACHINE.md)를 참고하세요.
4차에서는 실제 생성 시각에 따른 Reference validity/freshness와 독립 SAFE_STOP reason을 추가했습니다.
invalid/stale 경로는 제어권을 유지한 채 속도를 0으로 차단합니다. timeout은 기존 provider별 설정을 재사용합니다.
메시지·복구 입력·경계 chatter·최신 시험 결과는 [Reference 안전 통합 보고서](../../docs/MGM_REFERENCE_SAFETY.md)에 있습니다.
5차 구현·검증 결과는 [Mission Preparation 보고서](../../docs/MGM_MISSION_PREPARATION.md)를 참고하세요.
`parking_search_timeout`(s), `max_parking_search_distance`(m)는 실측 전 **-1.0(미설정)** 입니다.
둘 다 유한 양수로 설정해야 탐색합니다. 미설정 요청은 `CALIBRATION_REQUIRED`로 취소하고 일반 주행을 유지합니다.
`/operator/cancel_mission`의 Bool true는 Mission만 취소합니다. `/operator/stop`은 기존 임시 정지입니다.
`mission_events_csv_path`에 보정용 이벤트 CSV를 기록할 수 있으며 실차 통합 launch는 `mission_events.csv`를 설정합니다.
아래 기존 5상태 설명은 `base_state_machine_enabled=false`의 legacy 동작과 구별해야 합니다.

구조·규칙의 단일 소스는 워크스페이스 루트 `CLAUDE.md` (§2, §4, §5, §5.5). 이 문서는 실행·측정 절차만 다룬다.

**Integration v2의 통합 실행은 코스에 맞는 새 런북을 사용한다.**

통합 실시간 화면은 `./scripts/v2 view`: 차량 고정 RViz 한 창에 GPS/카메라 목표점,
정지선 거리·신호등, 4-LiDAR/SLAM 지도와 주차 경로·두 카메라 영상을 표시한다.
[통합 화면 사용법](../../docs/INTEGRATION_V2_VIEW.md)을 참고한다.

- [한라대학교 — Integration v2](RUNBOOK_integration_v2_halla.md): 업로드된 기준경로 (01 또는 02)→03→04→05→(06 또는 07)과 경로별 Zone/Mission 인계.
- [용인 Course A — Integration v2](RUNBOOK_integration_v2_yongin.md): 업로드된 2,141점 CSV, 별도 Mission Zone 준비.

두 런북은 v2 전용 설치/launch, 출발 점검, Mission PREPARE→ACTIVE, 종료와 기록 절차를 다룬다.
실차 검증 전 기준이며 Zone/Parking 보정값은 측정한 값을 입력한다.

아래 두 문서는 기존 통합 구성의 측정/운영 절차다. v2의 시작 명령과 신호/주차 상태 설명은
위 코스별 런북 및 6차 명세를 우선한다.

1. [`RUNBOOK_full_measurement_20260904.md`](RUNBOOK_full_measurement_20260904.md) —
   처음 설치하거나 장착 위치가 바뀐 경우의 임계값 측정
2. [`RUNBOOK_full_operation_20260904.md`](RUNBOOK_full_operation_20260904.md) —
   측정 완료 후 차선·GPS·회피·긴급정지·신호등을 함께 실행

기존 구성의 신호등 실차 정지 실행은 운영 런북의
`REAL_VEHICLE_lane_gps_can.launch.py` 명령 블록 하나다. 야간 국소 대비·평행
에지 쌍 정지선 검출과 `stack_traffic_node` 2초 자동 재시작도 이 구성에 포함된다.

## 구조 (§5.5 이중 트랙)

```
adas_mgm/
├── core/               # MGM 로직 코어 — ROS 헤더 include 절대 금지
│   ├── mgm_types.hpp   #   CoreSnapshot/CoreOutput/CoreState/CoreParams (Simulink 버스 1:1)
│   ├── mgm_step.hpp    #   CoreOutput mgm_step(const CoreSnapshot&, CoreState&)
│   └── mgm_step.cpp    #   판단(스테이트 머신) + 실행(ref 조립·종방향 병합)
├── src/mgm_node.cpp    # ROS 2 wrapper — msg 변환·10ms 틱·발행·지터 로깅만 (판단 로직 금지)
└── tools/              # back-to-back 하네스 (ROS 무관)
    ├── dump_format.hpp     # 스냅샷 덤프 파일 포맷
    ├── core_replay.cpp     # 덤프 → mgm_step 오프라인 재생 → CSV
    └── make_sample_dump.cpp# 합성 시나리오 덤프 생성 (MBD 합격 기준 입력)
```

코어는 `mgm_core` 정적 라이브러리로 빌드되며 rclcpp를 링크하지 않는다. ROS 없이 단독 확인:

```bash
g++ -std=c++17 -Wall -Wextra -c core/mgm_step.cpp -I.   # 통과해야 정상
```

## 실험용 generated backend (4상태 v1.88, opt-in)

ROS 노드의 기본 backend는 위 병행 Manager를 실행하는 C++ `core`이며, 기본 빌드에는 생성
backend가 링크되지 않는다. `ADAS_MGR2` v1.88을 실행하려면 아래 두 단계를 모두
명시해야 한다. v1.88은 TRAFFIC 상태가 없으므로 generated backend는
`traffic_state_enabled=false`일 때만 기동한다.

1. 지원 호스트에서 CMake opt-in:

   ```bash
   colcon --log-base log_generated build \
     --build-base build_generated \
     --install-base install_generated \
     --symlink-install --packages-up-to adas_mgm \
     --cmake-args \
       -DADAS_MGM_ENABLE_GENERATED_BACKEND=ON \
       -DBUILD_TESTING=ON
   ```

2. 먼저 **CAN을 실행하지 않는 bench**에서 backend와 제한 범위 확인을 함께 지정:

   ```bash
   source install_generated/setup.bash
   ros2 launch adas_mgm generated_backend_bench.launch.py \
     backend:=generated \
     generated_backend_acknowledge_limited_scope:=true
   ```

bench launch는 `mgm_node`만 실행하며 `bridge_dspace`, CAN 인터페이스 및 차량
launch를 실행하지 않는다. 기본 실행은 언제나 `backend:=core`,
`generated_backend_acknowledge_limited_scope:=false`이다. 출력도 운영
`/adas/target_ref`가 아니라 격리된 `/bench/adas/target_ref`로 강제 remap된다.
bench 결과는 다음 토픽에서 확인한다.

```bash
ros2 topic echo /bench/adas/target_ref
# 또는 발행 주기만 확인
ros2 topic hz /bench/adas/target_ref
```

생성 backend 빌드는 Linux x86-64에서 GNU/Clang C·C++ 컴파일러를 사용할 때만
지원한다. opt-in을 켠 채 미지원 환경에서 구성하면 CMake가 즉시 실패한다.
`BUILD_TESTING=ON`이면 opt-in이 꺼져 있어도 패리티 테스트용 생성 라이브러리가
빌드될 수 있지만, `ADAS_MGM_ENABLE_GENERATED_BACKEND=OFF`인 `mgm_node`에는
링크되지 않는다.

v1.88은 `LANE`, `WAYPOINT`, `AVOID`, `PARKING` 네 상태와 지정 구간 3종,
TTC/narrow 처리, 회피 복귀·timeout, 종점·역방향 래치를 포함한다. 단, 모델 생성
후 C++ core에 추가된 **후진 탈출(rear escape)** 은 포함하지 않는다. 생성 backend는
`escape_after_cycles=0`일 때만 기동하며, 후진 탈출을 켠 구성은 시작 시 명확히
거부해야 한다. 이 기능까지 필요하면 `estop_rear_clear` 입력과 escape 파라미터·상태를
모델에 추가하여 다시 생성하고 패리티를 검증해야 한다.

생성 API는 전역 상태 기반이라 adapter도 프로세스당 단일 인스턴스·단일 10ms
스레드 사용을 강제한다. CAN을 사용하는 현장 시험은 bench 검증과 패리티 테스트를
먼저 통과한 뒤 `RUNBOOK_mbd_lane_gps.md`의 확인 토큰·종료 가드를 그대로 따른다.

CMake cache는 이전 값을 유지한다. 한 번 ON으로 빌드한 디렉터리에서 옵션을
생략해도 자동으로 OFF로 돌아가지 않으므로, 검증과 운영 빌드는 아래처럼 서로
다른 build/install/log 디렉터리를 사용하고 옵션 값도 항상 명시한다.

```bash
# 기본 production/core 빌드
colcon --log-base log_core build \
  --build-base build_core \
  --install-base install_core \
  --symlink-install --packages-up-to adas_mgm \
  --cmake-args \
    -DADAS_MGM_ENABLE_GENERATED_BACKEND=OFF \
    -DBUILD_TESTING=OFF

# 지원 호스트의 generated 패리티 및 bench 빌드
colcon --log-base log_generated build \
  --build-base build_generated \
  --install-base install_generated \
  --symlink-install --packages-up-to adas_mgm \
  --cmake-args \
    -DADAS_MGM_ENABLE_GENERATED_BACKEND=ON \
    -DBUILD_TESTING=ON
colcon --log-base log_generated_test test \
  --build-base build_generated \
  --install-base install_generated \
  --packages-select adas_mgm --event-handlers console_direct+
```

생성 파일은 MathWorks Academic License 고지를 유지하며 비상업적 학업 용도로만
사용한다. opt-in 빌드는 생성 코드를 `mgm_node`에 정적으로 링크하므로 결과
바이너리에도 해당 제한이 적용된다. 생성 헤더의 `Validation result: Not run`은
그대로이며, 패리티 테스트가 MathWorks 코드 생성 검증 보고서를 대신하지 않는다.
생성 예제 `ert_main.c`는 ROS 실행 경로에 포함하지 않는다.

## back-to-back 검증 (§5.5)

```bash
# 1) 주행/루프백 중 스냅샷 기록 (params.yaml 또는 -p 로)
ros2 run adas_mgm mgm_node --ros-args -p snapshot_dump_path:=/tmp/snap.bin

# 2) 오프라인 재생 → CSV
ros2 run adas_mgm core_replay /tmp/snap.bin ref.csv        # 레퍼런스 코어
ros2 run adas_mgm parity_replay /tmp/snap.bin /tmp/parity_diff.csv
# rear escape 비활성 덤프에서 state/path/v_ref/ref_points 불일치 0이면 합격

# 실차 없이 파이프라인 점검용 합성 시나리오
ros2 run adas_mgm make_sample_dump /tmp/sample.bin
ros2 run adas_mgm core_replay /tmp/sample.bin out.csv
ros2 run adas_mgm parity_replay /tmp/sample.bin /tmp/sample_diff.csv
```

같은 덤프를 레퍼런스 코어로 두 번 재생하면 diff가 0이어야 한다(결정론). 덤프는 같은 머신·같은 ABI에서만 호환.

## SCHED_FIFO 권한 (§5.2)

루프 스레드는 SCHED_FIFO(우선순위 80)를 시도하고, 실패 시 경고 로그 후 일반 스케줄러로 동작한다.
권한 부여는 **rtprio limit 방식 권장**:

```bash
# /etc/security/limits.conf 에 추가 후 재로그인
yungi    -    rtprio    90
ulimit -r   # 90 확인
```

`sudo setcap cap_sys_nice+ep <binary>` 방식은 **비권장** — capability가 붙은 실행 파일은 로더가
`LD_LIBRARY_PATH`를 무시하므로 ROS 2 노드가 공유 라이브러리를 찾지 못해 실행이 깨진다.

## 지터 baseline 측정 (§5.3 → §7 판정 근거)

1. `config/params.yaml`:
   - `jitter_csv_path: "/home/yungi/mgm_jitter_baseline.csv"`
   - `cpu_core: 3` (예 — 다른 프로세스가 덜 쓰는 코어)
2. rtprio 권한 확인(위) 후 실행, 시작 로그에 SCHED_FIFO 경고가 **없는지** 확인.
3. 1시간 이상 방치 (지금은 인지 부하 없음 = baseline. 스택 개발 완료 후 풀가동 상태에서 재측정하여 대조).
4. 결과 정리: CSV의 `late_max` 열 최댓값 = **최악 lateness**. 로그의 `worst` 값과 일치해야 함.
5. 판정(§7): 최악 지연 × 2 를 watchdog 타임아웃으로 잡았을 때 안전한가 → v1 유지 / v3 이관.

기록 양식: `최악 lateness ____ us (측정일 ____, 부하: baseline/풀가동, 시간 ____ h)` — 결과는 CLAUDE.md §7 옆에 남길 것.

연속 CSV 운용: [경로 순서/전환 계약](../../docs/MGM_ROUTE_SEQUENCE.md).
한라대는 `route_sequence_file:=<v2>/src/stack_gps/waypoints/halla_route_sequence.yaml`로
`route_start_id:=01 route_end_id:=07`이면 01→03→04→05→07 순서다. 두 선택 인자는 필수다. 경로별 Mission 완료를 보존하며 마지막 파일만 FINISH다. 현재 raw dump는 v15이다.

현재 Parking 탐색은 [Zone 탐색 정책](../../docs/MGM_ZONE_SEARCH.md)을 따른다. source Zone 이탈 실패 뒤 현재 CSV를 계속 주행하며 시간·거리 제한을 쓰지 않는다.
