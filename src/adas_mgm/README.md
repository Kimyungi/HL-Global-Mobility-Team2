# adas_mgm — Decision 계층 (10ms MGM 루프)

구조·규칙의 단일 소스는 워크스페이스 루트 `CLAUDE.md` (§2, §4, §5, §5.5). 이 문서는 실행·측정 절차만 다룬다.

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

## 실험용 generated backend (LANE/WAYPOINT, bench only)

ROS 노드의 기본 backend는 기존 4상태 C++ `core`이며, 기본 빌드에는 생성
backend가 링크되지 않는다. `ADAS_MGR2` v1.68을 실행하려면 아래 두 단계를 모두
명시해야 한다.

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

2. **CAN을 실행하지 않는 bench에서만** backend와 제한 범위 확인을 함께 지정:

   ```bash
   source install_generated/setup.bash
   ros2 launch adas_mgm generated_backend_bench.launch.py \
     backend:=generated \
     generated_backend_acknowledge_limited_scope:=true
   ```

bench launch는 `mgm_node`만 실행하며 `bridge_dspace`, CAN 인터페이스 및
`REAL_VEHICLE_lane_gps_can.launch.py`를 실행하지 않는다. 실제 차량 또는 CAN이
연결된 환경에서 이 실험 backend를 사용하지 말 것. 기본 실행은 언제나
`backend:=core`, `generated_backend_acknowledge_limited_scope:=false`이다.
출력도 운영 `/adas/target_ref`가 아니라 격리된 `/bench/adas/target_ref`로 강제
remap된다. bench 결과는 다음 토픽에서 확인한다.

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

v1.68은 `LANE`과 `WAYPOINT` 두 상태의 공통 동작만 검증한 실험 모델이다. 다음
운영 4상태 기능은 포함하지 않는다.

- `AVOID`/`PARKING` 상태와 해당 경로·속도 처리
- TTC 즉시 정지와 좁은 통로 속도 제한
- 역방향 및 종점 래치, `estop_latch_release`
- 회피 복귀 hold와 회피 timeout

생성 C 자체의 `gps_at_end`는 운영 core의 래치가 아니라 현재 입력값을 직접
사용한다. 실험 runtime은 이 차이를 숨겨 운행하지 않고, `AVOID`/`PARKING`, TTC
안전 바닥, `gps_at_end`, 신뢰 가능한 역방향 입력이 들어오면 지원 범위 이탈로
판정해 비어 있지 않은 0속도 참조를 영구 래치한다. 다시 시험하려면 원인을 제거한
뒤 노드를 재시작해야 한다. 생성 API는 전역 상태 기반이라 adapter도 프로세스당
단일 인스턴스·단일 10ms 스레드 사용을 강제한다.

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
#    김재민 생성 코드 쪽 재생 결과 → gen.csv
diff ref.csv gen.csv                                        # 불일치 없으면 합격

# 실차 없이 파이프라인 점검용 합성 시나리오
ros2 run adas_mgm make_sample_dump /tmp/sample.bin
ros2 run adas_mgm core_replay /tmp/sample.bin out.csv
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
