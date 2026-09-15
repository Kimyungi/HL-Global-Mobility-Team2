# RUN_BOOK_FINAL

## v2 목표 속도 직접 전달 (2026-09-15)

MGM은 상태별로 결정한 `v_ref`를 후단 가감속 제한 없이 하위 제어기로 전달한다.
신호등 거리 기반 감속, 회피 진입·재출발·GPS 복귀, 일반 목표 0 요청 모두 적용한다.
실제 가감속은 하위 제어기가 담당한다. `a_up/a_down`은 legacy/generated에서만 사용한다.

신호등의 거리 추정과 목표 속도 계산은 유지한다. 현재 정지 문턱 설정은 1.0m이며,
해당 문턱에서 내부 목표가 0이면 최종 `v_ref`도 같은 틱에 0이다. 실제 차량의
정차 위치를 보장하는 변경은 아니므로 하위 제어기의 응답은 실차에서 확인해야 한다.
E-stop·운전자/CAN 정지·무효 참조의 즉시 0 출력은 유지한다.
정지 Zone 대기 시간은 유효한 실측 속도 `|v_act| ≤ 0.001m/s`인 동안만 차감한다.

`scripts/v2 build` 후 MGM을 재시작해야 적용된다. 새 로그는 dump v30이며,
기존 v29 이하 로그는 당시 버전으로 재생한다. 경로점 보간과 CAN 형식은 변경하지 않는다.


GPS를 먼저 연결하고, 그 연결을 유지한 채 통합 주행을 시작하는 순서다.
이 노트북의 저장소 위치는 `/home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main`이다.
아래 명령은 모두 이 폴더에서 실행한다. 손상민의 PR #98(`8796cdd`)에 있는
Path 4 `state=4` 표식과 1m 단축된 경로·GPS-only 종료 좌표를 반영했다.

## 0. 회피 Zone 전이 변경 빌드 — GPS 실행 전

**CSV `state=4`가 회피 시작 지점이다.** 한라 Path 4의 idx 55를 통과한 GPS 관측이
5회 확인되면 장애물이 없어도 `AVOID_ACTIVE`에 진입한다. 진입 전 일반 구간에서는
장애물 검출만으로 회피 상태에 들어가지 않는다. 독립 라이다 E-stop은 계속 적용된다.

회피를 시작한 뒤에는 장애물 소실·시간 경과·구간 이탈만으로 종료하지 않는다.
실제 장애물 회피 후 플래너의 웨이포인트 복귀 완료를 확인하고 `GPS_RETURN`으로
전환한 뒤, 현재 GPS 횡오차 0.1m 이하·헤딩 오차 20° 이하이면 일반 주행으로 돌아간다.
완료된 같은 시작 표식은 다시 실행하지 않는다. 장애물을 아직 만나지 않은 동안에는
회피 상태에서 기존 GPS station+1m 참조를 추종하며 회피를 기다린다.

Path 4 마지막 행은 idx 191, ENU 끝점 `(-63.042107, -71.940893)`이다.
CSV와 Zone YAML은 이 저장소의 같은 버전을 사용한다. 아래 빌드 후 통합 런처를
다시 시작해야 변경된 MGM 로직이 적용된다.

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
scripts/v2 build
scripts/v2 check
```

기본 목표속도는 일반 주행·회피 모두 **2m/s**다. 주차도 `v_base`를 공유하므로
전진 2m/s, 후진 -2m/s가 적용된다. 협소 통로 상한은 0.2m/s, E-stop 후
탈출 후진은 -0.8m/s다. 가감속 및 정지 조건에 따라 실제 명령은 낮아질 수 있다.
현재 장애물 검출 거리는 앞범퍼 기준 3.5m 미만이다. 2m/s로 접근할 때
3.5m의 TTC는 1.75초다. TTC와 인지 avoidable 값은 진단 정보이며,
v2 MGM은 이 값으로 추가 E-stop을 만들지 않는다. 독립 라이다 E-stop 요청을 반영한다.
2m/s 실차 검증은 아직 수행하지 않았다.

## 1. 초반 GPS 세팅 — 터미널 1

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
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

## 2. 라이다 4대 확인 — 통합 런처 실행 전

GPS 연결을 유지한 상태로 다음 명령을 실행한다.

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
ls -l /dev/lidar_front /dev/lidar_rear /dev/lidar_left /dev/lidar_right
```

링크가 없거나 USB 허브/포트를 옮겼다면, 통합 런처가 종료된 상태에서
하드웨어 serial로 전후좌우를 다시 식별하고 링크·접근 권한을 복구한다.
`ttyUSB0` 같은 번호만 보고 위치를 지정하지 않는다.

```bash
sudo /usr/bin/python3 scripts/v2_recover_lidars.py --apply
```

`LIDAR_LINKS_READY`와 4개 장치의 `OK`를 확인한다. 이 작업은 GPS 서비스를
종료하지 않는다. 링크 존재 확인은 실제 스캔 수신 확인과 별개다.
통합 런처는 링크 누락/접근 불가 시 CAN 기동 전에 실패 사유를 표시한다.

## 3. GPS 연결 후 통합 주행 시작

**터미널 1**에서 이어서 실행한다.

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
scripts/v2 prepare \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  start_waypoint:=01 end_waypoint:=07 \
  parking_enabled:=true parking_zone_entry_active:=true \
  avoidance_enabled:=true avoid_zone_only:=true \
  traffic_enabled:=true rviz:=true
```

기존 GPS 연결을 재사용하고 센서·MGM·RViz를 실행한다.
실제 CAN 송신을 활성화하며, 출발 인가를 기다린다.
위 명령은 `REAL_VEHICLE_integration_v2_drive.launch.py`를 실행하여
한라대 **`01 → 03 → 04 → 05 → 07`** 순서로 주행한다.
`src/stack_gps/waypoints/halla_route_sequence.yaml`에서 각 경로의 CSV와 Zone YAML을
함께 읽으며 PR #98의 수정 경로와 이번 회피 시작 조건을 사용한다.
이 터미널은 실행 상태로 둔다.

| 경로 | 적용 미션·구간 |
| --- | --- |
| 01 | 시작 경로. 종점 및 해당 경로의 미션 완료 후 03으로 전환 |
| 03 | GPS-only Zone 3(idx 28~116), 주차 접근 Zone 4(idx 133~145), T자 주차 표식(idx 145) |
| 04 | `state=4`(idx 55)부터 회피 상태 진입 → 웨이포인트 복귀 시 종료, 평행 주차(idx 163), 1m 단축된 최종 직선 |
| 05 | 주차 후 복귀 경로, 신호 예상 표식 `state=3`(idx 300) |
| 07 | 선택한 종료 경로 |

주차는 각 Zone YAML의 주차점과 기존 MGM 진입 조건으로 실행한다.
`state=4`는 실제 회피 진입 신호다. `avoid_zones: []`여도 CSV 표식을 읽어
시작 조건을 구성하며, GPS의 `avoid_zone`은 표식 이후 현재 CSV 끝까지 true다.
이는 시작 지점 통과를 놓치지 않기 위한 신호이며 CSV 끝이 회피 종료 조건은 아니다.
`avoidance_enabled:=true`는 기능 사용, `avoid_zone_only:=true`는 표식/구간에 따른
진입·복귀 완료 정책을 선택한다. 평행 주차 등 미션의 기존 우선권은 유지한다.
신호 표식도 위치만으로 정지를 강제하지 않으며 실제 카메라 인식 조건을 적용한다.

04 끝점 `(-63.042107, -71.940893)`과 05 시작점 `(-64.335, -70.648)`은
주차 후 인계를 위한 별도 위치다. 두 점을 직선 주행 구간으로 추가하지 않는다.
경로 전환에는 기존 종점·주차 완료 조건이 적용된다.

기동 로그에서 `[v2 drive] route: 01 -> 03 -> 04 -> 05 -> 07`을 확인한다.
같은 로그에 표시되는 세션 폴더의 `route_selected.yaml`에 실제 사용한 CSV와
Zone YAML 절대 경로가 기록된다. 시작·종료 경로를 바꾸려면 `start_waypoint`와
`end_waypoint`를 수정하고 런처를 다시 실행한다. 이 런처에서는 `waypoint_csv`,
`zones_file`, `route_sequence_file`을 별도 인자로 덮어쓰지 않는다.

**터미널 2**를 열어 출발을 인가한다.

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
scripts/v2 go
```

인가 성공 시 다음 메시지가 출력된다.

```text
출발 인가 완료 (MGM 확인). 실제 이동은 선택된 경로와 제어 조건에 따릅니다.
```

출발 준비 조건은 **라이다 정상 수신 AND (실제 카메라 영상 수신 OR GPS FIXED(4))**다.
라이다는 `/lidar/a1/scan`(전방), `/lidar/a2/scan`(후방),
`/lidar/b1/scan`(좌측), `/lidar/b2/scan`(우측) **4개 모두** 최근 0.35초
이내의 유효 스캔을 수신해야 한다. 회피·E-stop 입력도 신선하고 유효해야 한다.
드라이버 프로세스가 존재하거나 장치 링크가 있다는 것만으로 통과하지 않는다.

`go` 출력의 `라이다 정상=True`와 미수신/무효 항목이 비었는지 확인한다.
부족한 토픽은 `MgmState.lidar_missing_topics`에도 표시된다. `--force`나
`--skip-avoid`로 MGM의 라이다 필수 조건을 생략할 수 없다.

카메라는 차선 검출 여부와 별개로 실제 영상 수신을 검사한다. 라이다가
정상이면 카메라 없이 GPS FIXED로, GPS가 약하면 사용 가능한 카메라로
출발 인가할 수 있다. 실제 이동에는 유효 경로와 E-stop·CAN·미션 조건도
적용된다. GPS 전용 구간과 CSV 연결 구간에서는 GPS 경로가 필요하다.

현재 v2 설정은 `safe_stop_all_sensors_only=true`다. 주행 중 SAFE_STOP은
**카메라 2대 모두 고장 AND 라이다 4대 모두 고장 AND GPS 고장**일 때만
사유 2로 발생한다. 어느 하나라도 유효 입력을 주면 이 SAFE_STOP은 해제된다.

- 차선 카메라: `/perception/lane_camera`의 실제 프레임 시각, 1.0초 이내.
- 신호등 카메라: `/perception/traffic_camera`의 실제 수신 프레임 시각, 0.5초 이내.
  차선/신호 검출 실패나 초기 모델 준비 대기는 카메라 고장과 구분한다.
- 라이다: 위 raw scan 4개를 각각 0.35초 기준으로 판정한다.
- GPS: 0.5초 이내의 유효 fix/위치. FIXED(4)만 요구하지 않으며 헤딩/경로 유효성과 구분한다.

`/adas/mgm_state.sensor_alive_mask`는 정상 센서를 bit 0=차선 카메라,
1=신호등 카메라, 2~5=a1/a2/b1/b2, 6=GPS로 표시한다. 0이면 모두 고장이다.
기존 사유 1/4/8/16/32/64/128/256/512/1024/2048은 이 정책에서 SAFE_STOP을 만들지 않는다.
사용자·CAN 정지, 신호 대기, AUTO_ESTOP, 출발 인가 조건은 별도다.
유효 제어점이 없으면 `reference_motion_blocked=true`, 속도 0으로 새 경로를 기다린다.
**SAFE_STOP 판정 변경은 빈 회피 경로의 생성 실패 자체를 해결하지 않는다.**

## 독립 라이다 E-stop

v2는 `stack_estop` 요청으로만 라이다 E-stop을 적용한다. MGM의 회피 TTC/회피불가
추가 정지, 속도 계산의 TTC 정지, CSV 전환의 TTC 제한을 제거했다.
로그의 `safety=AUTO_ESTOP` 이름은 기존 메시지 호환을 위해 남아 있으나,
이 상태의 E-stop 입력은 독립 라이다 요청이다. 기존 주차/후진 중 처리 권한은 유지한다.
정적 거리 문턱 1.20m, 해제 1.35m 이상 또는 장애물 없음 3회 확인은 그대로다.
CAN/사용자 정지, 신호 정지, 센서 전체 상실, 유효 참조 확인은 각 역할을 유지한다.

## 신호 해제 후 GPS 재출발

v2 신호 해제 조건은 `(!red) || (!red && green)`, 즉 **`!red`**다.
적색 판정이 false이면 초록불 유무와 관계없이 `SIGNAL_IDLE`로 해제하고,
GPS 전용 구역은 `GPS_ONLY_NAV`, 일반 구역은 `GPS_BACKUP`으로 전환한다.
적색과 초록이 동시에 true이면 해제하지 않는다. 이 전환은 신호 상태를 빠져나오는
시점에만 적용하며, 평소 신호가 없는 주행 전체를 GPS로 고정하는 것은 아니다.
이후 차선 복귀는 기존 신뢰도 확인 조건을 따른다.

통합 런처는 `stack_traffic.resume_on_red_absence=true`를 사용한다.
정상 수신 영상마다 기존 5프레임 투표창을 갱신하며 적색 3표 미만이면 적색 판정을
해제한다. 따라서 정상 영상에서 신호등이 보이지 않는 경우도 적색 해제가 될 수 있다.
카메라 읽기 실패는 새로운 무적색 영상으로 투표하지 않고 직전 적색 상태를 보존한다.

신호 해제 시에도 표식으로 시작한 회피 상태는 유지한다. 장애물이 사라졌다는
이유로 회피 상태를 초기화하지 않으며, 실제 웨이포인트 복귀 완료 조건을 적용한다.
GPS 참조가 없으면 속도 0으로 대기한다. CAN/사용자 정지·AUTO_ESTOP·주차 권한은
각자의 조건을 계속 적용한다. 이번 변경으로 CAN 고장 래치가 자동 해제되지는 않는다.
기존 런처를 재시작하면 적용되며, 이미 실행 중인 노드에는 소급 적용되지 않는다.

## 장애물 회피 중 경로 재생성

장애물 검출 시작 범위는 앞범퍼 기준 전방 **3.5m 미만**, 차량 중심선 **좌우 0.5m**다.
회피 목표점 탐색 범위는 `avoid.offset_max_m=2.5`, 즉 **좌우 ±2.5m(전체 5m)**다.
최근접 거리는 검출·TTC에만 쓴다. 목표점은 전방 계획 영역에 걸친 모든 연결 면의
앞뒤 범위·끝점을 기준으로 생성하며, 면 방향과 끝점의 둥근 여유 영역을 반영한다.
HTML의 **면 기준 회피 · 30° 장애물** 및 **45° 장애물 · 2.7m 복귀** 예제로
후보 생성과 진입·복귀 곡선 검사 결과를 구분해 볼 수 있다.
HTML에서는 탐색 반폭 조절기로 후보 범위를 실험할 수 있으며 실차 설정과는 별도다.
한 번 검출한 장애물은 기존 0.4m 히스테리시스에 따라 전방 3.9m 미만까지
검출을 유지한다. 검출 영역은 회피 경로의 차량 폭·충돌 여유와 별도로 설정한다.

목표점은 같은 회피 구간에서 **GPS 웨이포인트 기준 기존 좌우 방향**을 우선한다.
그 방향의 새 후보 중 직전 ENU 목표점과 가까운 점부터 진입·복귀 곡선을 검사하고,
모두 실패할 때 다른 방향을 검사한다. 매 스캔 경로를 재생성하며 양쪽 모두 실패하면
이전 경로를 재사용하지 않는다. 회피 초기화/경로 변경 시 방향 기억도 초기화한다.
플래너 로그의 `goal_side`는 1=왼쪽, -1=오른쪽, 0=GPS 중심선 ±5cm,
`None`=미선택이며, `side_switched`는 이번 계산에서 선택 방향이 바뀌었는지 나타낸다.
HTML의 **목표점 안정화 · 45° ±1cm 관측**에서 **다음 관측** 버튼으로 확인한다.

일반 주행 중 CSV state=4 또는 지정 회피 구간의 진입이 확인되면,
장애물 유무와 관계없이 회피 상태가 차선/GPS 주행 선택보다 우선한다. 회피 목표점이
없거나 오래됐고 장애물이 여전히 검출되면 `AVOID_ACTIVE`에서 새 참조를 기다린다.
이때 사용할 제어점이 없으면 `reference_motion_blocked=true`, `v_ref=0`이지만
AUTO_ESTOP이나 SAFE_STOP 사유 4를 추가하지 않는다.
회피 상태에서 참조가 비거나 오래되면 장애물 검출 해제 여부와 관계없이
새 참조를 기다린다. 복귀 완료를 확인하기 전에 GPS로 우회하지 않는다.

회피 경로는 GPS 웨이포인트의 누적 station+1m 시작점 → 장애물 옆 회피점 →
회피점의 GPS station+2.7m 복귀점을 3차 곡선으로 연결한다. +1m 시작 곡선이
불가능할 때만 현재 차량 위치에서 재계획한다. 실제 제어점은 회피점이며,
그 점을 지나면 복귀 웨이포인트로 바뀐다. 매 유효 스캔마다 경로를 다시 계산하고
지나간 구간을 제거한다. 뒤쪽 미관측을 이유로 직선 꼬리를 유지하지 않는다.

GPS 위치·헤딩과 웨이포인트 창이 유효해야 회피 경로가 생성된다. 창이 소실되면
CAN 위치나 단일 GPS 기준점으로 대체하지 않는다. 경로가 복구되고 다른 정지
조건이 해제되면 기존 인가 상태에 따라 회피 주행을 재개한다.

`avoid planner:` 로그의 GPS_station, anchor, side, return과
current_pose_fallback을 확인한다. RViz `/perception/avoid_path`는 갱신된 경로만
표시한다. 상세 정의는 [회피 경로 문서](docs/AVOID_STATION_PATH.md)에 있다.
주차 미션의 기존 우선순위는 별도로 적용된다.
회피 시작·종료 조건은 [회피 Zone 상태 전이](docs/AVOID_ZONE_ENTRY.md)에 정리했다.

## 주행 중지와 종료

터미널 2에서 `scripts/v2 stop`으로 주행을 중지하고,
`scripts/v2 go`로 같은 경로의 주행을 다시 인가한다.
주행 중지나 `prepare` 터미널의 Ctrl-C는 GPS 서비스를 종료하지 않는다.
GPS까지 완전히 종료할 때만 `scripts/v2 gps-off`를 실행한다.

GPS 장치 설정은 [persistent_gps.yaml](src/stack_gps/config/persistent_gps.yaml)에 있다.
기본 연결은 `/dev/ttyRover` 115200 baud와 `/dev/ttyRadio` 38400 baud다.
자세한 동작과 검증 범위는 [주행 준비 문서](docs/DRIVE_PREPARATION.md)를 참고한다.


`Waiting for at least 1 matching subscription(s)...`가 계속 나오면 정지 명령은
아직 전달되지 않은 상태다. 해당 터미널의 Ctrl-C는 대기 명령만 취소한다.
**GPS를 유지하며 통합 주행과 로깅을 끝내려면 `prepare` 터미널에서 Ctrl-C**를
누르고, CAN 목표값 0 송신과 노드 종료 메시지를 확인한다. 이후
`scripts/v2 gps-status`로 GPS 서비스가 유지되는지 확인한다.

## 회피 레퍼런스 패스 HTML 실험실

HTML 실험실과 관련 도구는 이번 PR에 포함하지 않고 이 노트북의 로컬 파일로만
유지한다. 아래 명령은 해당 파일이 남아 있는 이 노트북에서만 사용할 수 있다.

`AVOID_REFERENCE_LAB.html`에서 장애물 2개·대각선 벽·차량 위치와 방향을 바꾸며
전방 라이다 관측, 검출 영역, 목표점 후보와 3차 레퍼런스 패스를 확인할 수 있다.
오프라인 모의 화면이며 차량에 연결하지 않는다.

```bash
xdg-open /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main/AVOID_REFERENCE_LAB.html
```

로컬 설명 및 검증 문서: `tools/avoid_reference_lab/README.md`.
