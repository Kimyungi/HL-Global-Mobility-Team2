# v2 생성 이전 미커밋 변경 22개 — 출처와 main 반영 근거

2026-09-13 조사. 모든 시각은 한국 시각(KST, UTC+09:00)이다.

## 판정 기준과 결과

사용자 기준은 **중복 제외 → v2 main 생성 이전 변경은 main → 이후 변경은 재확인**이다.
`integration/v2_main`의 로컬 branch 생성 reflog는 **2026-09-11 22:25:06**, 시작점은
`origin/main`의 `c76f287`이다. reflog의 Git 계정 이름은 `Xanadu`다.
첫 v2 내용 커밋은 `8697bcb`(2026-09-11 22:33:52)로, branch 생성 시각과 구별한다.
원격 최초 push 시각은 이번 판정 근거로 사용하지 않았다.

109개는 커밋 수가 아닌 `feat/state-machine` 작업 폴더의 변경 파일 경로 수다.
이미 처리한 main 독립 변경 20개는 `f99ff0b`에 반영됐다. 현재 v2와 같은 38개,
v2 조상의 같은 내용 27개, 과거 v2 문맥이 섞인 2개는 재반영 대상에서 제외한다.
나머지 **22개 모두 생성/변경 기록이 위 기준 이전**이며, 기준 이후로 재확인할 항목은 없다.

## 누가 언제 작성했는가

아래 22개 **현재 파일 내용에 대응하는 Git 커밋/업로드 기록은 확인되지 않았다.**
따라서 업로더·업로드 시각으로 표현하지 않고, 실제 로컬 작성 도구 기록과 파일 시각을 기록한다.
21개는 Codex의 파일 생성 또는 수정 도구 호출 원문을 확인했다. 로컬 파일 소유 계정은
22개 모두 `sangmin`이다. 소유 계정 및 Git 설정 이름만으로 실제 사람을 특정하지 않는다.

`작성/변경 확인 시각`은 신규 파일이면 생성 도구 호출, 기존 MGM launch면 이번 독립 변경의
수정 도구 호출 시각이다. 주차 launch만 쓰기 도구 기록이 없어 파일 시스템 시각을 표시했다.
`최종 수정`은 이번 머지용 사본이 아닌 원래 작업 폴더의 mtime이다. 초 단위로 표시했다.

| # | 파일 | 작성/변경 확인 시각 | 원본 최종 수정 | 확인된 작성 수단 |
|---:|---|---|---|---|
| 1 | `src/adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py` | 09-07 18:29:44 | 09-11 21:53:36 | Codex 수정 |
| 2 | `src/stack_parking/launch/parallel_parking_test.launch.py` | 09-04 22:15:54 | 09-04 22:15:54 | 미확인; 파일 생성/수정 시각 |
| 3 | `docs/lane_gps_2mps_test.md` | 09-07 18:30:49 | 09-10 19:42:32 | Codex 생성 |
| 4 | `docs/lunar_lake_gpu_setup.md` | 09-07 18:34:47 | 09-09 21:31:22 | Codex 생성 |
| 5 | `docs/traffic_incremental_80.md` | 09-08 20:31:37 | 09-08 20:37:17 | Codex 생성 |
| 6 | `docs/traffic_training_4000.md` | 09-08 20:10:54 | 09-08 20:10:55 | Codex 생성 |
| 7 | `scripts/gpu/add_intel_gpu_runtime_repo.sh` | 09-09 21:28:33 | 09-09 21:28:33 | Codex 생성 |
| 8 | `scripts/gpu/check_xpu.py` | 09-07 18:34:10 | 09-09 21:31:22 | Codex 생성 |
| 9 | `scripts/gpu/install_supported_xe_kernel.sh` | 09-09 20:53:53 | 09-09 21:10:29 | Codex 생성 |
| 10 | `scripts/gpu/remove_test_kernel.sh` | 09-09 20:53:53 | 09-09 20:53:53 | Codex 생성 |
| 11 | `scripts/gpu/rollback_mesa_xorg.sh` | 09-09 21:26:15 | 09-09 21:26:15 | Codex 생성 |
| 12 | `scripts/gpu/setup_lunar_lake.sh` | 09-07 18:34:10 | 09-08 09:42:44 | Codex 생성 |
| 13 | `scripts/gpu/use_xorg_for_xe.sh` | 09-09 21:26:15 | 09-09 21:26:15 | Codex 생성 |
| 14 | `scripts/run_lane_gps_2mps.sh` | 09-07 18:30:49 | 09-10 19:42:32 | Codex 생성 |
| 15 | `scripts/training/add_coco_traffic.py` | 09-08 20:06:17 | 09-08 20:06:17 | Codex 생성 |
| 16 | `scripts/training/build_mixed_traffic.py` | 09-08 20:07:17 | 09-08 20:07:17 | Codex 생성 |
| 17 | `scripts/training/prepare_incremental_80.py` | 09-08 20:27:17 | 09-08 20:27:17 | Codex 생성 |
| 18 | `scripts/training/prepare_reviewed_traffic.py` | 09-08 20:02:28 | 09-08 20:02:28 | Codex 생성 |
| 19 | `scripts/training/preserve_traffic_head.py` | 09-08 20:27:17 | 09-08 20:36:04 | Codex 생성 |
| 20 | `scripts/training/test_preserve_traffic_head.py` | 09-08 20:28:09 | 09-08 20:36:04 | Codex 생성 |
| 21 | `scripts/training/train_incremental_80.py` | 09-08 20:29:32 | 09-08 20:29:32 | Codex 생성 |
| 22 | `scripts/training/train_traffic_4000.py` | 09-08 20:07:48 | 09-08 20:07:48 | Codex 생성 |

주차 launch의 `parallel_opposite_straight_m` 원래 값 `1.0`은 `48ba0bc7`,
`Xanadu`, 2026-09-04 01:08:07의 Git 기록을 갖는다. 이것을 미커밋 `0.25` 변경의
작성자라고 볼 수는 없다. `0.25` 변경은 파일 시스템 기록상 기준 이전이지만 실제 작성자는
미확인이다. 다른 줄을 수정한 `sonsm0318`의 이력도 이 변경의 업로드 근거로 사용하지 않았다.

## 반영 범위

| 범위 | 파일 수 | main에 반영하는 내용 |
|---|---:|---|
| 주행 시험 | 3 | 선택형 2m/s runner·문서와 이를 위한 launch 인자/차선 CSV 연결 |
| 주차 단독 시험 | 1 | `parallel_opposite_straight_m` 기본값 1.0→0.25m 및 설명 일치 |
| PC GPU 환경 기록 | 8 | 당시 커널·Xorg·Intel runtime 준비/복구 스크립트와 문서 |
| 신호등 학습 도구 | 10 | 4000장 단일 클래스 및 후속 80클래스 보존 방식의 도구·문서 |

공유 MGM launch의 과거 v2 Zone/Mission 파라미터, mission/zone CSV 및 v2 메시지 기록
추가는 중복으로 제외했다. main에 필요한 독립적인 속도/TTC/전방 관측 범위 override와
차선 CSV 인자만 가져왔다. main의 기본 주행 속도와 `wait_go=true`는 유지한다.
2m/s runner는 별도 명시 실행용이다. 0.25m는 평행주차 **단독 시험 launch**에만 적용된다.

학습 두 방식은 같은 파일의 중복본이 아니다. `traffic_training_4000.md`는 이전
단일 클래스 시험 기록이며, 후속 방식과 교체 이유는 `traffic_incremental_80.md`에 있다.
두 도구를 저장하는 것이 두 모델을 동시에 운용하거나 차량의 모델을 교체한다는 뜻은 아니다.
GPU 스크립트 역시 당시 PC와 준비 파일을 전제로 한 기록이며 이번 머지에서 실행하지 않았다.

이번 provenance 문서와 `test_launch_overrides.py`는 위 22개를 반영하기 위해
2026-09-13에 새로 작성한 기록/검증 파일이다. 과거 22개로 계산하지 않는다.
원본 두 작업 폴더의 미커밋 파일은 보존하며 격리 worktree에서 main 반영을 준비했다.

## 검증

- Parking/Avoid/Estop 기존 시험 및 학습 head 보존 시험: **82 passed**.
- 실제 launch 파라미터 해석 검사: **7 passed**. main 기본값, 시험 override 전달,
  CSV와 debug image 분리, 주차 단독 시험 기본값/override를 확인했다.
- shell 7개 `bash -n`, Python 도구 9개 AST 구문 검사 통과. `git diff --check` 통과.
- launch 검사는 Node/Include/OpaqueFunction을 실행하지 않았다. 차량/CAN/센서 기동,
  GPU/OS 설치, 전체 모델 학습과 실차 주행은 이번 검증 범위에 없다.
- v2 main의 실행 중 소스·install과 원격 `137d49e`를 이 main 반영으로 갱신하지 않는다.
