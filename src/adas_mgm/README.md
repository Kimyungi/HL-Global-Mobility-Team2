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
