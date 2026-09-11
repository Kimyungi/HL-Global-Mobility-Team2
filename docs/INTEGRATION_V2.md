# Integration v2_main — 실차 검증 전 통합 기준

2026-09-11. 별도 `integration/v2_main` 브랜치를 사용한다. 기반은 최신
`origin/main`의 `c76f287`이며, 기존 main 브랜치·작업 폴더·설치 환경은 보존한다.
v2는 실차 검증 전 통합 후보이며 운영 main을 대체하지 않는다.

## 코드 반영 전 결정

- MGM 6차 병행 Manager, Zone 안정화, Mission PREPARE/ACTIVE, Reference gate,
  Calibration/Recovery 진단과 관련 메시지·provider·시험·명세를 함께 반영한다.
- 기존 작업에 포함된 MGM reference hold 및 통합 런처의 계측/파라미터 노출도
  현재 시험된 기준의 일부로 포함한다. 속도나 센서 threshold를 새로 정하지 않는다.
- main PR #84의 GPS parking_points/코스 자료는 보존하고 새 Zone 생산과 함께 검증한다.
  이 자료를 새 Mission 탐색 위치 또는 보정값으로 인증하지 않는다.
- 별도 Traffic 학습/라벨링, GPU 환경, 2m/s 시험, Parking 단독 시험 변경은 제외한다.
  Traffic 실행 코드는 main 버전을 사용하고 v2 인터페이스로 함께 빌드한다.
- 생성 v1.88은 역사적 parity 시험에만 사용한다. v2 운영 backend는 core다.

## 분리 경계

소스는 별도 git worktree, 빌드는 `build_v2`, 설치는 `install_v2`, 빌드 로그는
`log_v2`, 차량 로그는 v2 작업 폴더의 `drive_logs`를 사용한다.
MGM과 공유 메시지/provider뿐 아니라 CAN bridge 및 저장소 내 센서 의존성까지
동일 v2 소스에서 빌드한다. 기존 main overlay를 underlay로 source하지 않는다.
패키지 이름과 토픽 계약은 유지한다. 런처 이름만 바꾸거나 legacy 모드 플래그만
끄는 방식으로 기존 구동 코드가 보존되었다고 간주하지 않는다.

전용 v2 진입점은 깨끗한 ROS 환경에서 v2 설치 prefix를 확인한다.
기본 명령은 하드웨어 없는 검사이며 실차 실행은 명시적 vehicle 하위 명령과
기존 REAL_VEHICLE_CONFIRM 토큰을 요구한다. v1/v2는 같은 센서 포트/CAN을
사용하므로 동시에 실행하지 않는다. 다른 DDS domain만으로 하드웨어가 분리되지 않는다.

Zone 확인 수 0, Parking 제한 -1, Recovery OFF를 유지한다.
정의된 Zone에서 확인 수가 미설정이면 일반 주행은 정지하며, Parking 탐색은
보정 전 취소된다. 실차 운용값을 임의로 채워 실행 가능 상태로 만들지 않는다.

현재 상태 머신 정본은 [MGM_MBD_STATE_MACHINE_SPEC.md](MGM_MBD_STATE_MACHINE_SPEC.md)다.
이번 분리는 경로 생성·인지·조향·속도 merge 알고리즘을 변경하지 않는다.

## 작업 폴더와 사용법

현재 v2 worktree는 `/home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main`이다.
기존 작업 폴더는 `feat/state-machine`의 미커밋 상태 그대로 유지한다.
기존 main의 commit/런처/구동 코드를 바꾸지 않았고 기존 설치 디렉터리에도 쓰지 않았다.
git worktree는 저장소 이력을 공유하지만 checkout/build/install은 별도다.

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
./scripts/v2 build    # 저장소 내 의존성 포함 13개 패키지, generated 운영 backend OFF
./scripts/v2 check    # 하드웨어 없는 prefix/메시지 계약 및 보정 상태 확인 (기본 동작)
./scripts/v2 test     # core/Python/ROS 합성 시험, 센서/CAN 실행 없음
./scripts/v2 bench    # MGM만, localhost domain 175, /integration_v2/* 토픽
```

다른 경로에 checkout해도 스크립트가 자기 워크스페이스를 찾는다. 기존 `~/FMA_ws`를
덮거나 symlink를 교체하지 않는다. 부모 shell의 AMENT_PREFIX_PATH/PYTHONPATH/
LD_LIBRARY_PATH를 가져오지 않고 `/opt/ros/humble` + 자기 `install_v2/local_setup.bash`만
적용한다. 시스템/user Python 의존성 설치 자체를 복제하는 가상환경은 아니다.
v2 설치 후 기존 main의 `setup.bash`를 이어서 source해 섞는 운용은 사용하지 않는다.

`check`는 12개 ROS 패키지 prefix와 새 메시지 필드를 확인한다. 별도의 plain CMake
`ydlidar_sdk`까지 포함하면 빌드 패키지는 13개다. `check` 성공은 센서 연결이나 차량
구동 검증을 의미하지 않는다. `test`의 ROS 합성 시험은 localhost domain 174를 사용한다.

실차 시험 진입점은 다음 명령이다. **이번 작업에서는 실행하지 않았다.** 먼저 선택한
코스의 Zone 확인 표본 수와 Mission 탐색 시간/거리, 장착 상태를 측정해 설정해야 한다.
기본값을 유지하면 정의된 Zone에서 정지하며 Parking 탐색도 보정 미설정으로 취소된다.
아래 `<...>`는 측정/선택한 값으로 바꾼다. 시험용 숫자를 운영값으로 복사하지 않는다.

```bash
./scripts/v2 vehicle \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  waypoint_csv:=<사용할_코스_CSV_절대경로> \
  zone_enter_confirm_samples:=<측정한_양수_표본수> \
  zone_exit_confirm_samples:=<측정한_양수_표본수> \
  parking_search_timeout:=<측정한_초> \
  max_parking_search_distance:=<측정한_미터> \
  lane_weights:=<사용할_기존_가중치_절대경로>
```

카메라 가중치는 Git에 포함되지 않을 수 있으므로 기존 파일의 **경로를 명시적으로**
지정하거나 v2의 해당 models 디렉터리에 준비한다. 기본 homography는 v2 checkout의
`src/stack_lane/config/homography.json`이며 장착 상태가 다르면 기존 인자로 지정한다.
가중치를 다운로드하거나 calibration 파일을 새로 만들지는 않았다.

전체 실행 구성은 기존 통합 launch를 재사용한다. 기본 Parking은 true, Traffic은 false다.
필요한 기존 인자를 그대로 전달할 수 있다. 로그는 이 v2 worktree의
`drive_logs/v2_<날짜_시각>/`에 모으며, backend=core, wait_go 및 기존 CAN 종료 guard를 유지한다.
v2에서 같은 이름의 예전 런처를 직접 실행하는 대신 전용 `scripts/v2 vehicle`을 사용한다.

실차 launch 이후 **v2 폴더의 별도 터미널**에서 `./scripts/v2 state`로 병행 상태를 확인한다.
현장 점검 후 `./scripts/v2 go` (Traffic도 점검할 때 `go --require-traffic`)로 기존 출발
점검을 사용한다. `./scripts/v2 stop`은 기존 operator stop 입력을 보낸다.
vehicle/go/state/stop은 같은 기본 ROS domain 0을 사용한다. 벤치 domain 175의 MGM에는
이 차량 명령을 연결하지 않는다. 차량 종료는 기존 Ctrl-C/CAN zero guard 절차를 따른다.

## 반영 및 향후 병합

[INTEGRATION_V2_SCOPE.md](INTEGRATION_V2_SCOPE.md)에 포함 69개 파일과 제외 목록을 기록했다.
MGM core만 따로 main에 넣으면 기존 provider 메시지 계약이 맞지 않으므로 이 변경은
`integration/v2_main`에서 관련 패키지 전체로 관리한다. 여기서 후속 변경과 실차 검증을
누적한다. 기존 main으로의 최종 승격 여부는 현장 결과를 확인한 뒤 별도 결정한다.
원격 push/main merge는 수행하지 않았고, v2의 upstream도 아직 설정하지 않았다.

## 이번 검증 결과

- 기존 main overlay 없이 **13개 패키지 빌드 성공** (51.6초).
- **CTest 15/15**, **Python 337 passed / 3 skipped**, **ROS synthetic 32 PASS**.
  main PR #84 GPS 시험, main Traffic, v2 인터페이스와 새 런처 검사 4개를 포함한다.
- 외부 overlay 환경변수를 넣은 부모 shell에서도 `scripts/v2 check`는 v2의
  12개 패키지 prefix와 메시지 계약으로 정상 진입했다.
- 실차 launch는 graph/기본 경로/확인 토큰 거부 및 `--show-args`만 검사했다.
  벤치는 MGM 하나만 포함하며 C++에 선언된 모든 주행 토픽이 `/integration_v2`로 remap된다.
- 실제 센서·CAN bridge·차량은 실행하지 않았다. 실차 calibration/추종/정지는 미검증이다.

Python skip 3개는 기존 DepthAI API 조건, 경고 1개는 설치된 SciPy/NumPy 조합이다.
SDK의 기존 CMake/컴파일 경고와 다른 패키지의 미사용 MGM CMake 옵션 경고도 있었으나
빌드/시험 실패는 없다. 환경 패키지나 SDK 코드는 이번 분리에서 변경하지 않았다.
작업 당시 상세 로그/입력 patch/원본 hash는 `/tmp/mgm-v2-integration/`에 보관했다.
임시 로그는 영구 시험 기록 대신 위 명령으로 재현할 수 있다.
