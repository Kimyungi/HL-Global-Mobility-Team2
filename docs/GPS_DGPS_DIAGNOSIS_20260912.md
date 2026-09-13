# 한라대 현장 DGPS 지속 원인 분석 — 2026-09-12

차량 PC `sangmin-17Z90TL-H-AUB9U1`, `integration/v2_main` HEAD `8697bcb`와
현재 미커밋 연속 경로/Zone 탐색 변경을 대상으로 조사했다. 차량과 베이스의 설정,
MGM 상태, 출발 인가, 센서 실행 상태는 이번 분석에서 변경하지 않았다.
수신 전용 TCP/ROS 관측, 실행 환경·소스 비교, 하드웨어 없는 시험만 수행했다.

## 결론

MGM/경로 상태가 FIXED(4)를 DGPS(2)로 덮어쓰는 로직이나, 현재 GPS 노드의 v1/v2
overlay 혼용은 발견되지 않았다. 실제 발행 함수와 ROS 직렬화 시험에서도 4는 4로 유지된다.
현재 수신 경로에서 가장 먼저 해결·확인할 문제는 **기준국 좌표 메시지 1005/1006 미관측**이다.

18:00:32 KST부터 차량 라디오 중계 TCP를 60초 동안 수신했다. 31,312바이트에서 독립
파서 `pyrtcm.RTCMReader(validate=1)`가 정상 RTCM 300개를 해석했으며 오류는 0개였다.
1074/1084/1094/1124/1230은 각각 60개, 1005/1006은 0개였다. 관측 메시지의 기준국 ID는 모두 0이다.
이는 해당 관측 구간의 결과이며, 베이스가 어떤 상황에서도 좌표 메시지를 보내지 않는다는 증명은 아니다.

ZED-F9P의 정적 베이스 RTK에는 관측 메시지와 기준국 위치 1005 또는 1006이 함께 필요하다.
1005 출력은 설정한 출력 포트/주기와 베이스의 Survey-in 완료 또는 고정 좌표 설정 상태를
확인해야 한다. 좌표 메시지는 관측 메시지보다 낮은 주기로 전송할 수도 있다.
[u-blox Integration Manual §3.1.5.4–5, Appendix C.1](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)

**베이스 송출 설정/베이스 위치 확정 문제인지, 라디오 전달 중 누락인지는 아직 분리하지 못했다.**
베이스 PC의 실제 설정과 송출 전 원문에는 이 세션에서 접근하지 못했다.
사용자가 베이스에서 확인한 22,965바이트 파일도 차량 PC에는 전달되지 않아 분석하지 못했다.

## 실행 중인 소프트웨어와 장치

| 항목 | 확인 결과 | 판정 범위 |
|---|---|---|
| GPS 실행 프로세스 | PID 85230, v2 `install_v2/stack_gps/lib/stack_gps/stack_gps_node` | 구 main GPS 실행 파일 아님 |
| GPS Python 경로 | 실행 프로세스 환경을 복제하여 import 확인. `build_v2/stack_gps/stack_gps`가 현재 v2 `src/stack_gps/stack_gps`로 resolve | node.py/GgaLink의 실제 해석 경로 확인 |
| 소스 변경 시각 | node.py/GgaLink 모두 현재 GPS 프로세스 시작 이전에 수정됨 | 분석 중 소스를 바꿔 이전 프로세스와 다르게 만든 상황 아님 |
| ROS 메시지 라이브러리 | `/proc/85230/maps`의 fma_interfaces 라이브러리 전부 v2 build/install | 구 인터페이스 바이너리 로딩 흔적 없음 |
| GPS publisher | `/perception/gps_path` publisher 하나, `stack_gps_node` | 다른 GPS publisher가 품질을 덮는 상황 관측 안 됨 |
| GPS 실제 파라미터 | `/dev/ttyRover`, 115200, `127.0.0.1:2101`, stale_timeout 1.5s | GetParameters 읽기 전용 응답 확인 |
| MGM 실제 파라미터 | core, base_state_machine_enabled=true, wait_go=true, GPS freshness 0.5s | generated/legacy 경로 오실행 아님 |
| 코스 | halla_route_sequence.yaml, start 01/end 07 | 현재 지정 코스와 일치 |
| 로버 장치 | `/dev/ttyRover` → `/dev/ttyACM0`, u-blox USB 장치 | 라디오/라이다를 GPS로 연 상태 아님 |
| 라디오 장치 | `/dev/ttyRadio` → `/dev/ttyUSB5`, 중계 PID 85159 | 로버와 별개 포트 |
| 시리얼 중복 점유 | 로버는 GPS PID 85230만, 라디오는 중계 PID 85159만 FD 소유 | 다른 probe/기록기가 원문을 나눠 읽는 상황 관측 안 됨 |
| 카메라 USB 실제 속도 | 두 Luxonis 장치 모두 sysfs 480Mbps, traffic 로그 usb_actual=HIGH | 과거 USB3/SUPER 설정 재발은 현재 확인 안 됨 |
| raw NMEA/C/N0/UBX | 이번 분석에서는 직접 취득하지 않음 | 포트 중복 읽기를 피함. 수신기 내부 설정과 RF 품질은 미확인 |

현재 조회는 GPS 파라미터 서비스 최초 탐색이 2초 안에 완료되지 않아 재시도했다.
6초 탐색을 허용한 재시도에서 위 값들을 정상 수신했다. 이를 GPS 수신 중단으로 해석하지 않았다.

### main과의 비교

로컬 `main=19464ae`, 로컬에 저장된 `origin/main=c76f287` 각각과 현재 작업 파일을 비교했다.
아래 파일은 두 기준 모두와 동일했다. 원격의 최신 상태를 새로 fetch한 결과라는 뜻은 아니다.

- `src/stack_gps/stack_gps/gga_link.py`
- `src/stack_gps/stack_gps/usb_reset.py`
- `src/stack_gps/tools/base_station/rtcm_server.py`
- `src/stack_gps/tools/base_station/setup_base.py`
- `src/stack_gps/tools/base_station/read_base_position.py`

node.py에는 v2 경로/Zone/측정 메타데이터 변경이 있지만, RTCM host/port와 시리얼 연결 방식,
`fix_quality` 대입문은 main과 같다. 차량 런처가 베이스의 setup_base.py를 실행하는 경로도 없다.

차량 라디오 중계 PID의 셸에는 기존 FMA_ws ROS 환경 변수가 남아 있었다. 다만 실행 파일은
v2 경로의 rtcm_server.py이며 이 파일은 ROS나 stack_gps 모듈을 import하지 않는다.
소스는 main과 동일한 serial→TCP 원문 중계다. GPS/MGM 프로세스의 환경은 v2로 분리되어 있었다.
따라서 해당 셸 환경만으로 GPS quality가 2로 변환된다고 볼 근거는 없다.

## 품질 값이 지나가는 경로

```text
F9P NMEA GGA의 quality 필드
  → GgaLink.parse_gga(): int(f[6])
  → GgaLink._fix에 같은 quality 저장
  → StackGpsNode.tick(): msg.fix_quality = quality
  → fma_interfaces/GpsPath 직렬화
  → MGM이 GPS 사용 가능 여부 판단
```

- `gga_link.py::parse_gga()`는 GGA의 6번 필드를 정수로 읽는다. 4→2 매핑은 없다.
- node.py의 stale/no-fix 분기는 `fix_quality=0`을 만든다. 2를 만드는 분기가 아니다.
- node.py 로그는 `self.link.latest_fix()`의 quality를 바로 읽어 2=DGPS, 4=FIXED로 표시한다.
  이 로그는 MGM의 LINE/GPS/주차 상태를 역으로 읽어 만드는 출력이 아니다.
- CSV 선택, Zone, 헤딩 접선/융합 여부는 경로와 진단 값을 바꾸지만 quality를 바꾸지 않는다.
- `/perception/gps_fix`의 `sensor_msgs/NavSatFix.status=0`은 일반 STATUS_FIX 표현이다.
  이 값만으로 RTK FIXED 여부를 판정하면 안 된다. 현재 구분 필드는 GpsPath.fix_quality다.
- 로그의 `age 0.1s`는 **PC가 마지막 GGA를 받은 뒤 지난 시간**이다. RTCM 보정 나이가 아니다.
- `RTCM 500B/s`는 시리얼 쓰기 경로로 넘긴 바이트 수다. 수신기가 그 메시지를
  보정 계산에 승인했다는 뜻은 아니다. 승인 여부는 UBX-RXM-RTCM 등 별도 관측이 필요하다.

## 오프라인 검증

센서 포트, 실제 네트워크 송신, ROS 출발 publisher 없이 수행했다.

| 시험 | 결과 |
|---|---|
| 실제 parse_gga + 별도 pynmeagps 파서 + 실제 StackGpsNode.tick + 실제 ROS 직렬화/역직렬화 | 22개 통과: quality 0/1/2/4/5 × sequence 사용/미사용 × GPS-only 안/밖, 추가 stale FIXED 2개 |
| 실제 GgaLink._run의 시리얼/TCP만 mock, 입력 프레임을 조각내 전달 | 3개 통과: DGPS/FIXED/FLOAT 유지, 합성 1005 원문이 시리얼 쓰기까지 동일하게 전달됨 |
| 기존 Zone stability/route plan/path engine/USB recovery 테스트 | 67 passed |

합성 1005에는 시험용 좌표만 들어 있으며 **실제 수신기나 네트워크에는 보내지 않았다**.
이 시험은 실행 코드가 특정 RTCM 번호를 제거하지 않는지 검사한다.
실제 라디오 손실이나 실제 수신기 승인까지 증명하는 시험은 아니다.

## 수신 자료와 재시작 전후

| 자료 | 관측 |
|---|---|
| 17:10:56–17:48:36 GPS 상태 로그 | DGPS 1,032개, GPS 99개, FIXED/FLOAT 0개 |
| 17:52:58–18:03:44 재시작 후 GPS 상태 로그 | DGPS 324개, FIXED/FLOAT 0개 |
| 18:00:32부터 60초 차량 RTCM 원문 | 31,312바이트, 각 1074/1084/1094/1124/1230 60개, 1005/1006 없음, 파서 오류 0 |
| 18:04 ROS 관측 8초 | GPS 메시지 79개 모두 quality=2, GPS publisher 하나 |

베이스 관측 메시지는 각 위성군에 두 신호를 담고 있으며 반복된 GPS 주간 시각 값도 갱신된다.
이는 자료가 완전한 RTK 입력이라는 인증이 아니며, 특히 기준국 좌표 미관측은 해소되지 않았다.
기준국 ID 0은 이번 메시지에서 서로 일치한다. ID가 0이라는 이유만으로 오류로 분류하지 않았다.

## 별도로 발견한 소프트웨어 검토 사항

### 1. 기존 GGA 파서의 체크섬 미검증

현재 `parse_gga()`는 NMEA `*XX` 체크섬을 검사하지 않는다. 올바른 quality=4 문장에서
체크섬을 그대로 두고 quality 문자만 2로 바꾼 합성 손상 문장을 기존 파서는 받아들였고,
독립 NMEA 파서는 체크섬 오류로 거부했다. 기존 main에도 있는 동작이다.

따라서 실제 NMEA 원문의 체크섬까지 검증한 수신기 품질 관측은 이번 분석 범위에 없다.
지속 DGPS가 이 결함 때문이라고 단정할 자료도 없다. 이번에는 운용 코드를 변경하지 않았다.
후속 수정에서는 체크섬 오류를 무효 샘플로 처리하고 진단 개수를 남기는 동작을 검토할 수 있다.

### 2. 출발 시 FIXED 요구와 주행 중 GPS 사용 조건이 다름

`src/adas_mgm/tools/go`는 quality==4일 때 출발 검사를 통과시킨다.
반면 `mgm_node.cpp`의 GPS 입력 사용 조건은 freshness와 `fix_quality != 0`이다.
따라서 go 검사 이후 RTK가 DGPS/FLOAT로 떨어졌다는 이유만으로 GPS를 무조건 사용할 수 없게
하는 전용 조건은 없다. Reference 등 다른 gate는 별도로 적용된다.

이것은 수신기를 DGPS로 만드는 원인이 아니라 **정밀도 저하 시 운용 정책**의 별도 문제다.
새로운 주행 중 RTK 상실 정책을 임의로 추가하거나 기존 상태/기준값을 바꾸지 않았다.

### 3. 현재 위치/경로 및 CAN 상태는 별도 확인 필요

18:04 관측에서 path 01 idx 26/종점, cross_track 약 35.34m, 헤딩 접선 fallback이었다.
이는 현재 GNSS가 계산한 위치와 경로 사이 차이이며, 실제 차량 위치 오차의 진실값이라고
단정할 수 없다. FIXED 복구 후 실제 위치·선택한 출발점·베이스 좌표 일치를 확인해야 한다.
CSV 끝점이나 Zone을 수정해서 RTK FIXED를 만들 수는 없다.

같은 관측에서 CAN tx_ok=false, errno=105, consecutive_tx_fail=37412였다.
이전 정상 송신 확인 후 다시 실패가 관측된 것으로, GPS 품질 생성 경로와는 별개다.
MGM은 Top ENABLE(0), TargetRef.v_ref=0, Estop 요청 true였다.
이 보고서는 주행 준비 완료나 출발 승인 보고서가 아니다.

## 아직 직접 확인하지 못한 항목과 다음 분기

차량에서 접근 가능한 코드·프로세스·입력은 위와 같이 확인했다. 다음은 베이스 또는 수신기
원문 접근이 필요한 항목이다. 현재 GPS 포트를 다른 프로세스로 동시에 열어 확인하지 않았다.

1. **베이스 송출 전 1005 확인:** 베이스의 TCP 수신 파일을 해석한다. 베이스에도 없다면
   TMODE, 고정 좌표/Survey-in 완료, NAV-PVT TIME, 실제 출력 포트의 TYPE1005 enable/주기를 확인한다.
2. **베이스에는 있고 차량에는 없다면:** 같은 시간대 전송 자료로 라디오·UART 구간을 비교한다.
   현재 차량 TCP 자료는 보정이 라디오를 통과한 뒤의 관측이다.
3. **좌표 메시지가 차량까지 도착한 뒤에도 해결되지 않으면:** 로버의 실제 NMEA/UBX 원문과
   NMEA 체크섬·여러 talker의 GGA 혼재 여부,
   USB RTCM 입력 활성, RTCM 수신 승인/CRC, RTK fixed 허용 설정, 안테나와 C/N0를 확인한다.
   기존 rtk_probe.py는 포트를 여는 도구이므로 GPS 노드와 동시에 실행하지 않는다.
4. 베이스를 고정 좌표로 복구할 때는 등록 당시 실제 설치 위치/높이와의 일치를 먼저 확인한다.
   코스 ENU 원점과 베이스 안테나 좌표는 서로 다른 값이다. 이번 조사에서는 어느 쪽도 바꾸지 않았다.

이 단계까지 조사한 결과만으로 베이스의 특정 설정값이 틀렸다고 확정하거나,
소프트웨어에서 quality를 FIXED로 강제하는 것은 타당하지 않다.

## 증거 파일

차량 v2 작업 폴더의 `drive_logs/gps_audit_20260912/`에 저장했다. Git에서 제외되는 현장 자료다.

- `vehicle-rtcm.bin`, `vehicle-rtcm.json`: 실제 차량 TCP 수신 원문과 독립 파서 결과.
- `runtime.json`: 파라미터 응답, ROS 품질/현재 상태, publisher 관측.
- `history.json`: 재시작 전후 상태 로그 집계 및 원본 로그 경로.
- `quality_flow.py`, `quality-flow.json`: 실제 발행 함수와 직렬화의 오프라인 시험.
- `link_flow.py`, `link-flow.txt`: I/O를 전부 mock한 실제 GgaLink 루프 시험.

이번 분석의 저장소 변경은 이 보고서 추가뿐이다. 기존 미커밋 변경은 보존했고 commit/push는 하지 않았다.
