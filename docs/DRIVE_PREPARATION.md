# 주행 준비와 GPS 상주 연결

`prepare`는 GPS·센서·MGM·RViz를 준비하고 출발 인가를 기다리는 통합 런처다.
GPS 수신기/RTCM만 별도 프로세스에 남기므로 `stop`이나 준비 런처 Ctrl-C 뒤에도
베이스 보정 연결과 NMEA 수신은 계속된다. 기존 `drive`는 같은 런처의 별칭이다.

## 사용

```bash
# 통합 준비: 기본 한라대 01 → 03 → 04 → 05 → 06, RViz 포함
scripts/v2 prepare REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX

# 다른 터미널에서 출발 / 중지 / 같은 경로로 재출발
scripts/v2 go
scripts/v2 stop
scripts/v2 go
```

`start_waypoint:=02 end_waypoint:=06`처럼 경로를 선택할 수 있다. 준비 런처를
종료하고 다시 열어도 이미 실행 중인 GPS 서비스를 재사용한다. `go`는 기존
operator stop을 해제하고 재인가하며, CSV/미션 세션을 새로 초기화하지 않는다.

GPS만 미리 켜 FIXED를 기다리고 싶을 때는 다음을 사용한다.

```bash
scripts/v2 gps-start     # GPS/RTCM 연결 + 수신량과 FIXED 상태를 1초마다 표시
scripts/v2 gps-off       # GPS까지 완전히 종료할 때만 사용
```

`gps-start` 화면에서 `RTCM 560 B/s | FIXED (FIX=4)`처럼 수신량 바로 옆에
현재 상태가 표시된다(숫자는 예시). FLOAT는 `FLOAT (FIX=5)`, 관측이 끊기면
`NO FIX (FIX=0)`로 바뀐다. Ctrl-C는 상태 표시만 종료하며 GPS 연결은 유지된다.
이후 같은 터미널에서 `prepare`를 실행하면 된다. 별도 상태 확인 명령은 필요 없고,
다른 터미널에서 한 번만 조회하고 싶을 때는 `scripts/v2 gps-status`도 사용할 수 있다.

GPS 서비스가 없으면 `prepare`에서 자동으로 시작하므로 `gps-start`는 선택 사항이다.
정지/런처 종료는 GPS를 종료하지 않는다. 컴퓨터 종료·수신기 전원 해제는 별개다.

## 출발 조건

아래 허용 조건은 라이다 4대와 회피·E-stop 입력이 모두 정상일 때 적용된다.
라이다가 하나라도 미수신/무효이면 카메라와 GPS 상태에 관계없이 대기한다.

| 현재 입력 | 출발 인가 | 일반 구간 경로 선택 |
|---|---|---|
| 카메라 없음 + GPS FIXED(4) | 허용 | GPS |
| 카메라 영상 있음 + GPS FLOAT/DGPS/약한 GPS | 허용 | 사용 가능한 차선 경로, 없으면 유효한 GPS 경로 |
| 카메라 영상 있음 + 차선 미검출 | 허용 | 준비된 유효 GPS 경로 |
| 카메라 없음 + GPS가 FIXED 아님 | 대기 | 재인가 전 현재 입력 확인 |

출발 점검은 **라이다 정상 AND (카메라 실제 프레임 수신 OR GPS 현재 FIXED(4))**다.
통합 런처에서는 전후좌우 raw scan 4개(0.35초 이내), 회피·E-stop 입력을 확인한다.
차선 confidence와 신호등 입력은 이 출발 조건과 별개다.
현재 v2 주행 SAFE_STOP은 카메라 2대·라이다 4대·GPS가 전부 무효일 때만 발생한다.
출발 인가의 라이다 필수 조건과 제어점 부재 시 속도 0 처리는 별도다.
센서별 판정 및 진단 필드는 `RUN_BOOK_HALLA_FINAL.md`를 따른다.
`--require-traffic`을 직접 지정하면 그 추가 점검은 적용한다. 기존 `--skip-gps`와
`--skip-lane`은 해당 출발 센서를 선택에서 제외하며, 둘 다 생략해 인가를 우회하지 않는다.
`--force`도 MGM의 출발 준비 조건과 실제 제어 게이트를 해제하지 않는다.

카메라 기동 신호 `/perception/lane_camera`는 새 영상의 캡처 시각을 담은 Header다.
차선 검출이나 워밍업 완료와 독립적이며, 같은 영상/stamp를 반복해도 신선해지지 않는다.
MGM의 `lidar_ready`, `lidar_missing_topics`, `camera_available`, `gps_fixed_ready`, `start_ready`, `go_authorized`로 확인한다.
RViz 상단의 PREP/LIDAR/CAM/GPS/GO 표시로 준비와 인가 상태를 확인할 수 있다.
CLI는 송신 완료만으로 성공 처리하지 않고 MGM의 인가 응답을 기다린다.

GPS FIXED는 **출발 조건**이다. 주행 중 FLOAT가 되었다는 이유만으로 인가를 취소하지
않고, 실제 주행 경로 유효성에 따라 전환한다. 일반 CSV 구간은 GPS 위치가 없더라도
유효한 카메라 경로로 진행할 수 있다. CSV 카탈로그는 위치 측정과 별도로 수신하되,
GPS 관측 없이 종점·다음 CSV·미션 완료를 만들어내지는 않는다.

GPS 전용 Zone, CSV 연결 구간, 주차 GPS 탐색, 회피 후 GPS 정렬 단계는 해당 경로가
필요하다. E-stop·CAN·외부 중지와 실제 기준점 유효성 검사도 계속 적용한다.
GPS 경로 CSV는 준비되어 있다는 전제이며, 두 조향 기준점이 모두 무효인 경우
카메라 켜짐만으로 가상의 직선 경로를 만들지 않는다.

## GPS 연결 유지 방식

`stack_gps/config/persistent_gps.yaml`이 물리 수신기 설정의 기준이다.
기본은 `/dev/ttyRover` 115200baud, `/dev/ttyRadio` 38400baud → 로컬 RTCM 2101이다.
`prepare`의 `rtcm_device`, `rtcm_host`, `start_rtcm`으로 보정 연결 설정을 지정할 수 있다.
이미 실행 중인 서비스와 설정이 다르면 포트를 재개방하지 않고 차이를 알린다.
설정 변경은 `gps-off` 이후 다시 준비한다.

상주 서비스는 후속 route 노드와 분리된 세션에서 실행하며, 사용자별 로컬 소켓으로
GPS 관측을 전달한다. 하나의 물리 포트를 두 프로세스가 열지 않도록 서비스 소유권과
시리얼 독점 개방을 사용한다. 경로 노드의 종료/재시작은 서비스를 종료하지 않는다.
GPS 노드·차선 카메라 노드가 종료되어도 전체 주행 런처를 내리지 않고 재시도한다.

수신기의 실제 FIX 품질과 원래 관측 시각을 유지한다. 수신 중지나 전파 상태 변화로
실제 품질이 떨어진 것을 이전 4로 덮어쓰지 않는다. 이 변경은 **주행 중지 때문에
보정 연결을 끊거나 GPS를 재초기화하지 않는 것**이며 RF 상태까지 강제할 수는 없다.
로그와 소켓 위치는 `gps-status`가 표시한다.

`vehicle` 등 과거 launch는 기본 direct 링크를 유지한다. 상주 GPS를 사용 중이면
`gps_link_mode:=persistent`로 연결하거나 통합 `prepare`를 사용한다.
새 메시지·core snapshot 필드 때문에 재빌드가 필요하며 raw dump는 v26이다.

## 검증

실제 장치 없이 가짜 GPS 로컬 소켓에서 클라이언트 재시작/관측 시각 보존/FLOAT 전환/
명시 종료를 검증한다. localhost ROS 시험은 실제 MGM과 go CLI를 실행하여 GPS 단독,
카메라 기동·차선 미검출, GPS 상실/카메라 상실, stop/go 재인가를 확인한다.
전체 실차 주행과 장시간 RTK 유지 시험은 수행하지 않았다.

검증 결과: 관련 패키지 7개 빌드, MGM CTest 22개, GPS/준비 관련 Python 묶음
219개, 나머지 통합 Python 묶음 446개(3 skip), RViz 6개 및 실제 MGM/CLI ROS
출발 시나리오 10개를 통과했다. 두 Python 묶음에는 일부 공통 검사가 포함된다.
`prepare --show-args`, 설치된 go CLI, v2 설치 계약과 shell 문법도 확인했다.
