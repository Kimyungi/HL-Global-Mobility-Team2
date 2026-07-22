# MBD 착수 킷 — 김재민 (Simulink/Stateflow → 생성 C 코드 트랙)

> 스펙의 단일 소스는 `CLAUDE.md` §4·§5.5. 이 문서는 착수에 필요한 파일·설정·합격 기준만 정리한다.
> 레퍼런스 구현: `src/adas_mgm/core/` (김윤기). 모델이 이것과 **같은 입력에 같은 출력**을 내면 합격.

## 1. 입·출력 버스 정의 = `src/adas_mgm/core/mgm_types.hpp`

이 헤더가 곧 Simulink 버스 정의다. 필드 그대로 버스를 만들 것 (형·순서·이름 일치 권장).

| 구조체 | Simulink 대응 | 비고 |
|---|---|---|
| `CoreSnapshot` | 입력 버스 | 인지 6종의 코어 필요분. bool→boolean, float→single, int32_t→int32 |
| `CoreOutput` | 출력 버스 | state, path_source, immediate_stop, v_ref, ref_points[20] |
| `CoreState` | 모델 내부 상태 | Stateflow 차트 상태 + Data Store (params 포함) |
| `CoreParams` | tunable parameter | params.yaml과 1:1 |
| `CorePoint` / `CorePath` | 서브 버스 | CorePoint = RefPointWire와 동일 레이아웃 (float×4) |
| 상수 | `MGM_NUM_POINTS=20`, `MGM_PERIOD_S=0.01` | 고정 배열 크기·주기 |

## 2. Stateflow 차트 스펙 = `docs/state_machine_detail.drawio` + CLAUDE.md §4

- 스테이트 4개 (lane=0, waypoint=1, avoid=2, parking=3), 전이 라벨·우선권 표 그대로.
- avoid 복귀처 변수 1개(`avoid_return`)만 기억. parking→avoid 전이 없음.
- 판단 뒤의 실행부(ref 조립 블렌드·v_ref rate limit)는 Stateflow 밖의 일반 블록으로 —
  로직 배치는 레퍼런스 `core/mgm_step.cpp`의 `transition`/`prioritize`/`assemble`/`merge` 4함수 구조 참고.

## 3. 코드 생성 설정 (Embedded Coder)

- step 함수 이름 **`mgm_step`** (레퍼런스와 동일 — wrapper 교체 탑재의 전제).
- **정적 메모리만** — 동적 할당(malloc) 금지 옵션.
- 자료형: single/boolean/int32/uint8 — double 사용 금지 (레퍼런스 코어도 전부 float).
- 솔버: discrete, 고정 스텝 0.01s.

## 4. 합격 기준 — back-to-back (하네스: `src/adas_mgm/tools/`)

입력 덤프와 정답 CSV는 결정론 도구로 재생성한다 (바이너리 커밋 안 함):

```bash
colcon build --packages-select adas_mgm && source install/setup.bash
ros2 run adas_mgm make_sample_dump sample.bin      # 합성 시나리오 1100틱(11s) — 전 스테이트·전이 통과
ros2 run adas_mgm core_replay sample.bin golden.csv  # 레퍼런스 정답 출력
```

생성 코드를 같은 방식으로 재생(core_replay.cpp에서 `mgm_step` 호출부만 생성 코드로 링크)하여
`diff golden.csv gen.csv`가 0이면 합격. 실차 rosbag 덤프(`mgm_node`의 `snapshot_dump_path` 파라미터로 기록)도 같은 절차.

### 정답 CSV 스테이트 전이 체크포인트 (빠른 육안 확인용)

| tick | state | 의미 |
|---|---|---|
| 319 | 0→1 | 차선 신뢰도 저하 20주기 → waypoint |
| 450 | 1→2 | 장애물+회피 가능 → avoid |
| 549 | 2→1 | 기동 완료 → 진입했던 waypoint로 복귀 |
| 579 | 1→0 | 신뢰도 복귀 20주기 → lane |
| 660 | 0→2 | avoid 진입 + TTC<0.8 → immediate_stop=1, v_ref=0 |
| 679 | 2→0 | 복귀 |
| 700 | 0→3 | 주차구간+공간 인식 → parking |
| 899 | 3→0 | 주차 완료 |
| 950 | — | estop → immediate_stop=1, v_ref 즉시 0 (스테이트는 lane 유지 — "정지는 스테이트가 아니다") |

## 5. 주의

- 스펙 변경은 CLAUDE.md §4 갱신이 선행 — 모델·레퍼런스 어느 쪽도 임의 변경 금지.
- 덤프 바이너리는 같은 머신·같은 ABI에서만 호환 (`tools/dump_format.hpp` 참조).
- float 연산 순서 차이로 마지막 자리 수 diff가 나면 허용 오차 비교(예: 1e-5)로 완화하되, 스테이트·immediate_stop·path_source는 **완전 일치**여야 한다.
