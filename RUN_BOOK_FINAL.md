# RUN_BOOK_FINAL

GPS를 먼저 연결하고, 그 연결을 유지한 채 통합 주행을 시작하는 순서다.
모든 명령은 차량 PC의 `~/FMA_ws` 기준이다. 변경사항을 처음 받은 PC에서는
먼저 `cd ~/FMA_ws && scripts/v2 build`를 한 번 실행한다.

## 1. 초반 GPS 세팅 — 터미널 1

```bash
cd ~/FMA_ws
scripts/v2 gps-start
```

GPS와 베이스 보정 데이터(RTCM) 연결을 시작하고 수신량과 현재 FIX 상태를
1초마다 함께 표시한다. CAN과 카메라는 이 명령으로 실행되지 않는다.
이미 GPS가 연결되어 있으면 기존 서비스를 재사용한다.

예상 출력의 주요 부분은 다음과 같다. 수신량, 누적 바이트, PID는 예시다.
첫 줄의 `-- B/s`는 수신량 계산을 위한 첫 관측을 기다린다는 뜻이다.

```text
GPS 수신 상태를 1초마다 표시합니다. Ctrl-C: 표시만 종료, GPS 연결 유지.
이후 scripts/v2 prepare … 실행 / GPS 완전 종료: scripts/v2 gps-off
RTCM     -- B/s | NO FIX (FIX=0) | 누적 0 B | GPS PID=123
RTCM    560 B/s | FLOAT (FIX=5) | 누적 560 B | GPS PID=123
RTCM    580 B/s | FIXED (FIX=4) | 누적 1140 B | GPS PID=123
```

- `RTCM … B/s`: 수신기 쪽으로 전달된 보정 데이터의 초당 바이트 수다.
  500 이상이라는 숫자 자체가 FIXED를 의미하지는 않는다.
- `FIXED (FIX=4)`: 현재 RTK FIXED 상태다.
- `FLOAT (FIX=5)`: 현재 RTK FLOAT 상태다.
- `NO FIX (FIX=0)`: 유효한 FIX가 없거나 최근 GPS 관측이 끊긴 상태다.

별도로 상태 확인 명령을 입력할 필요는 없다. 위 상태 전환 순서와 FIXED 도달
시간은 수신 환경에 따라 달라진다. 화면의 상태는 실제 수신값을 따른다.

GPS 연결을 확인한 뒤 **Ctrl-C**를 눌러 상태 표시를 닫는다.
`상태 표시 종료. GPS/RTCM 연결은 계속 유지됩니다.`가 출력되며,
GPS 수신기와 베이스 보정 연결은 그대로 유지된다.

## 2. GPS 연결 후 통합 주행 시작

**터미널 1**에서 이어서 실행한다.

```bash
cd ~/FMA_ws
scripts/v2 prepare REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX
```

기존 GPS 연결을 재사용하고 센서·MGM·RViz를 실행한다.
실제 CAN 송신을 활성화하며, 출발 인가를 기다린다.
기본 경로는 한라대 `01 → 03 → 04 → 05 → 07`이다.
이 터미널은 실행 상태로 둔다.

**터미널 2**를 열어 출발을 인가한다.

```bash
cd ~/FMA_ws
scripts/v2 go
```

인가 성공 시 다음 메시지가 출력된다.

```text
출발 인가 완료 (MGM 확인). 실제 이동은 선택된 경로와 제어 조건에 따릅니다.
```

출발 준비 조건은 **실제 카메라 영상 수신 또는 GPS FIXED(4)**다.
차선이 검출되어야만 카메라 준비로 인정하는 방식이 아니다.
카메라가 없으면 GPS로, GPS가 약하면 사용 가능한 카메라 경로로 주행한다.
카메라 영상만 있고 차선이 없으면 준비된 유효 GPS 경로를 사용한다.
실제 이동에는 유효한 주행 경로와 E-stop·CAN·미션 조건이 적용되며,
GPS 전용 구간과 CSV 연결 구간 등에서는 GPS 경로가 필요하다.

## 주행 중지와 종료

터미널 2에서 `scripts/v2 stop`으로 주행을 중지하고,
`scripts/v2 go`로 같은 경로의 주행을 다시 인가한다.
주행 중지나 `prepare` 터미널의 Ctrl-C는 GPS 서비스를 종료하지 않는다.
GPS까지 완전히 종료할 때만 `scripts/v2 gps-off`를 실행한다.

GPS 장치 설정은 [persistent_gps.yaml](src/stack_gps/config/persistent_gps.yaml)에 있다.
기본 연결은 `/dev/ttyRover` 115200 baud와 `/dev/ttyRadio` 38400 baud다.
자세한 동작과 검증 범위는 [주행 준비 문서](docs/DRIVE_PREPARATION.md)를 참고한다.
