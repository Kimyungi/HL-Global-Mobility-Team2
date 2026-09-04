# 처음 가는 지역의 GPS 웨이포인트 기록 가이드

**기준:** 2026-09-04 `main`의 `record_waypoints.py`, `walk_test.launch.py`,
`stack_gps_node`

이 문서는 처음 방문한 지역에서 베이스 좌표를 만들고, 차량이 따라갈 코스를 CSV로
기록하고, 통합 주행에 사용할 수 있는지 검증하는 전체 절차다. 위에서부터 순서대로
실행한다.

> 웨이포인트 절대좌표는 **그날 사용한 베이스의 확정 좌표에 묶인다.** 베이스 위치나
> 플래시 좌표를 바꾸면 같은 CSV를 사용할 수 없다. 새 지역에서는 베이스 측량을 먼저
> 끝낸 뒤 그 베이스로 RTK FIXED를 만든 상태에서 코스를 기록한다.

> 기록 중에는 `stack_gps_node`, `rtcm_client_inject.py`, 통합 실차 launch를 동시에
> 실행하지 않는다. 로버 `/dev/ttyRover`는 한 프로세스만 열 수 있으며,
> `record_waypoints.py`가 RTCM 주입과 NMEA 기록을 모두 담당한다.

---

## 0. 준비물과 결과물

준비물:

- 베이스: EVK-F9P, GNSS 안테나, 베이스 PC, 전원
- 로버: 차량용 FST-UEF9P와 안테나, 차량 PC
- 베이스와 차량을 연결할 로컬 Wi-Fi/Tailscale 또는 Holybro SiK 라디오 2대
- 처음 측량하는 지역이면 인터넷과 사용할 수 있는 NGII VRS 계정
- 차량으로 기록하면 운전자와 안전 감시자

최종 결과물:

```text
$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_<지역_코스>_YYYYMMDD_HHMMSS.csv
```

예시 이름은 `halla_main`, `wonju_course_a`처럼 영문 소문자·숫자·밑줄만 사용한다.

---

## 1. 차량 PC 최초 준비

이미 정상 빌드된 차량 PC라면 2단계로 이동한다.

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/FMA_ws"
python3 -m pip show pyserial >/dev/null || python3 -m pip install --user pyserial
colcon build
source "$HOME/FMA_ws/install/setup.bash"
```

로버를 연결하고 고정 장치명이 있는지 확인한다.

```bash
ls -l /dev/ttyRover
```

없으면 기록을 시작하지 않는다. `HANDOVER.md`의 F9P udev 설치 절차를 먼저 수행한다.

---

## 2. 새 지역인지 판단

베이스 PC에서 다음 파일을 연다.

```bash
cd "$HOME/FMA_ws/src/stack_gps/tools/base_station"
sed -n '1,240p' BASE_LOCATIONS.md
```

- 지역과 **동일한 안테나 설치 위치·높이**가 등록돼 있으면 새 지역이 아니다.
  `BASE_MOVE.md`를 따라 기존 좌표를 플래시에 다시 넣고 4단계로 이동한다.
- 표에 없는 위치라면 새 지역이다. 아래 3단계로 측량한다.
- 같은 건물이어도 안테나를 다른 옥상, 다른 삼각대 위치 또는 다른 높이에 세우면
  새 지점이다.

---

## 3. 새 지역 베이스 좌표 측량 — 지역당 최초 1회

상세 기준은 `../base_station/BASE_SURVEY.md`다. 아래 명령은 그 절차의 필수 실행
순서다.

### 3-1. 기존 플래시 좌표 백업

베이스 PC의 터미널 B0에서 실행한다.

```bash
cd "$HOME/FMA_ws/src/stack_gps/tools/base_station"
python3 read_base_position.py
```

출력 좌표가 `BASE_LOCATIONS.md`에 없으면 먼저 표에 기록한다. EVK 플래시에는 좌표가
한 벌만 저장되므로 다음 단계에서 기존 값이 덮인다.

### 3-2. 안테나 고정과 베이스 모드 해제

안테나를 하늘이 열린 고정 지점에 설치한다. 측량 후에는 위치와 삼각대 높이를 바꾸지
않는다. 설치 위치가 재현되도록 사진도 남긴다.

```bash
cd "$HOME/FMA_ws/src/stack_gps/tools/base_station"
python3 setup_base.py --disable
```

### 3-3. 현장 개략 좌표 확인

휴대전화 지도 등으로 현장의 위도·경도 십진도를 확인한다. 아래 두 값은 반드시 새
현장 값으로 바꾼다.

```bash
export FMA_APPROX_LAT="<현장_개략_위도>"
export FMA_APPROX_LON="<현장_개략_경도>"
python3 -c 'import os; a=float(os.environ["FMA_APPROX_LAT"]); b=float(os.environ["FMA_APPROX_LON"]); assert -90<=a<=90 and -180<=b<=180; print("개략 좌표:",a,b)'
```

### 3-4. NGII VRS 주입 (베이스 PC 터미널 B1)

NGII 계정은 셸 히스토리에 비밀번호가 남지 않도록 입력받는다.

```bash
cd "$HOME/FMA_ws/src/stack_gps/tools/base_station"
export NGII_USER="<본인_NGII_아이디>"
export FMA_APPROX_LAT="<현장_개략_위도>"
export FMA_APPROX_LON="<현장_개략_경도>"
read -rsp "NGII 비밀번호: " NGII_PASS; echo
export NGII_PASS
python3 ntrip_inject.py --lat "$FMA_APPROX_LAT" --lon "$FMA_APPROX_LON"
```

이 터미널은 10분 측량이 끝날 때까지 켜 둔다. 계정당 동시 접속은 1개이므로 401이면
다른 PC의 접속과 캐스터 설정을 확인한다.

### 3-5. 10분 측량 (베이스 PC 터미널 B2)

```bash
cd "$HOME/FMA_ws/src/stack_gps/tools/base_station"
python3 measure_base_position.py --duration 600
```

`carrSoln=FIXED`일 때만 표본이 쌓여야 한다. 완료 후 출력되는 위도, 경도,
**타원체고**와 마지막 `setup_base.py` 명령을 복사해 보관한다. 표준편차 경고가 있거나
FIXED 표본이 부족하면 안테나 시야를 개선하고 3-4부터 다시 한다.

### 3-6. 좌표 등록과 플래시 저장

측량 결과를 즉시 `../base_station/BASE_LOCATIONS.md`에 새 행으로 추가한다. 위치 ID,
장소, 안테나 설치 위치·높이, 위도, 경도, 타원체고, 측량일을 모두 남긴다.

그다음 B2가 출력한 실제 숫자로 실행한다.

```bash
cd "$HOME/FMA_ws/src/stack_gps/tools/base_station"
python3 setup_base.py --lat <측량_위도> --lon <측량_경도> --height <측량_타원체고>
```

`fixType=5 (TIME — 베이스 정상)`이 나와야 한다. `--svin`은 재부팅할 때 기준 좌표가
바뀌므로 운영 웨이포인트에 사용하지 않는다.

---

## 4. RTCM 연결 시작

### 4-1. 베이스 PC 터미널 B1

앞 단계의 NTRIP 프로그램은 `Ctrl-C`로 종료한 후 실행한다.

```bash
cd "$HOME/FMA_ws/src/stack_gps/tools/base_station"
python3 rtcm_server.py --radio /dev/ttyRadio
```

10초 통계가 `RTCM` 수백 B/s여야 한다. `0 B/s`면 계속 진행하지 않는다.

### 4-2. 차량 PC 터미널 V1 — 라디오를 사용할 때

```bash
source /opt/ros/humble/setup.bash
python3 "$HOME/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py" \
  --port /dev/ttyRadio --tcp-port 2101
```

이 프로그램은 수신 라디오의 RTCM을 차량 PC의 `127.0.0.1:2101`로 중계한다.
V1은 웨이포인트 기록이 끝날 때까지 켜 둔다.

### 4-3. Wi-Fi/Tailscale만 사용할 때

베이스 PC에서 주소를 확인한다.

```bash
hostname -I
```

차량 PC에서 그 주소의 2101 포트에 연결할 수 있어야 한다. 라디오 V1은 실행하지 않고,
아래 기록 명령의 `FMA_RTCM_HOST`에 베이스 PC 주소를 넣는다.

---

## 5. 기록할 경로를 먼저 계획

기록 전에 다음을 결정한다.

- 시작점과 진행 방향
- 차선 중심 또는 차량이 실제로 따라야 할 기준선
- 폐곡선이면 시작점과 종료점을 같은 위치·같은 진행 방향으로 만들 방법
- T자/평행 주차 탐색 구간의 대략적인 시작·끝 위치
- 차량과 보행자를 통제할 감시자 위치

한 CSV에는 한 방향의 연속 경로만 기록한다. 왕복 경로를 한 파일에 겹쳐 넣지 않는다.
다른 방향이나 다른 코스는 이름을 바꿔 새 파일로 기록한다.

기본 간격은 0.2m다. 차량용 일반 코스는 0.3m를 권장하며 급커브는 기록 속도를 낮춘다.
풀조향 궤적으로 기록하면 실제 추종 때 보정 여유가 없어지므로 급커브는 조향 한계의
70~80% 정도로 부드럽게 돈다.

---

## 6. 웨이포인트 기록 (차량 PC 터미널 V2)

먼저 중복 포트 사용자가 없는지 확인한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
ros2 node list | grep stack_gps || true
pgrep -af 'rtcm_client_inject|record_waypoints' || true
```

`stack_gps_node`, 기존 `record_waypoints.py`, `rtcm_client_inject.py`가 보이면 해당
프로그램을 실행한 터미널에서 `Ctrl-C`로 종료한다. 라디오 중계용 V1의
`rtcm_server.py`는 종료하지 않는다.

### 라디오를 쓰는 표준 명령

`FMA_TRACK_NAME`만 새 지역과 코스에 맞게 바꾼 뒤 전체를 실행한다.

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/FMA_ws/src/stack_gps/tools/waypoints"

FMA_TRACK_NAME="new_area_course_a"
FMA_RTCM_HOST="127.0.0.1"

python3 -c 'import re; n="'"$FMA_TRACK_NAME"'"; assert re.fullmatch(r"[a-z0-9_]+",n), n; print("코스 이름:",n)'
python3 record_waypoints.py \
  --host "$FMA_RTCM_HOST" \
  --port 2101 \
  --serial /dev/ttyRover \
  --spacing 0.3 \
  --name "$FMA_TRACK_NAME"
```

Wi-Fi/Tailscale를 직접 쓰면 `FMA_RTCM_HOST="<베이스_PC_IP>"`로 바꾼다.

정상 화면은 다음과 같다.

```text
[record] 베이스 ... 주입 시작
[record] 기록 파일: .../waypoints_<이름>_<시각>.csv
[record] 기준점 고정: ...
[record] FIXED  점 ...개  경로 ...m  RTCM ...B/s
```

기록 방법:

1. `FIXED`가 연속으로 표시될 때까지 정지한다.
2. 시작점에서 약 3초 정지한 뒤 천천히 출발한다.
3. 계획한 진행 방향으로 경로 중심을 한 번만 이동한다.
4. `FLOAT`, `NOFIX`, `FIX 아님 — 기록 일시정지`가 나오면 즉시 멈춘다.
5. FIXED가 돌아오더라도 그 구간이 비었거나 크게 건너뛰었으면 전체 기록을 버리고
   다시 기록한다.
6. 종료점에서 약 3초 정지한 뒤 `Ctrl-C`를 한 번 누른다.

종료 로그에 점 개수, 경로 길이, 저장 경로가 나온다. 점이 2개 미만이라는 경고가
나오면 실패다.

---

## 7. 생성 CSV 자동 검사

방금 저장된 파일을 자동 선택해 기본 형식을 확인한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"

FMA_TRACK_NAME="new_area_course_a"
FMA_NEW_CSV="$(ls -1t "$HOME"/FMA_ws/src/stack_gps/waypoints/waypoints_"$FMA_TRACK_NAME"_*.csv | head -1)"
export FMA_NEW_CSV
echo "$FMA_NEW_CSV"

python3 -c 'import csv,os; p=os.environ["FMA_NEW_CSV"]; r=list(csv.DictReader(open(p))); assert len(r)>=2, "웨이포인트 2개 미만"; assert all(int(x["quality"])==4 for x in r), "FIXED 아닌 점 포함"; assert [int(x["idx"]) for x in r]==list(range(len(r))), "idx 불연속"; print("CSV 기본 검사 통과:",len(r),"점, 시작",r[0]["lat"],r[0]["lon"],"끝",r[-1]["lat"],r[-1]["lon"])'
```

CSV 열은 다음 의미다.

| 열 | 의미 |
|---|---|
| `idx` | 0부터 시작하는 경로 인덱스 |
| `utc` | GNSS GGA UTC 시각 |
| `lat`, `lon` | WGS84 십진도 |
| `height_m` | GGA MSL 고도와 지오이드 분리값을 합한 타원체고 |
| `east_m`, `north_m` | 첫 점 기준 로컬 미터 좌표 |
| `quality` | 4=RTK FIXED; 기록 파일에는 4만 있어야 함 |

---

## 8. 도보 재현과 화면 검사

기록 프로그램을 종료한 상태에서 차량 PC 터미널 V2에 GPS 검증 launch를 띄운다.
라디오 V1을 쓰면 `rtcm_host:=127.0.0.1`, Wi-Fi/Tailscale면 베이스 PC IP를 쓴다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
FMA_TRACK_NAME="new_area_course_a"
FMA_NEW_CSV="$(ls -1t "$HOME"/FMA_ws/src/stack_gps/waypoints/waypoints_"$FMA_TRACK_NAME"_*.csv | head -1)"

ros2 launch stack_gps walk_test.launch.py \
  waypoint_csv:="$FMA_NEW_CSV" \
  rtcm_host:=127.0.0.1 \
  serial_port:=/dev/ttyRover
```

차량 PC의 새 터미널 V3에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
FMA_TRACK_NAME="new_area_course_a"
FMA_NEW_CSV="$(ls -1t "$HOME"/FMA_ws/src/stack_gps/waypoints/waypoints_"$FMA_TRACK_NAME"_*.csv | head -1)"
cd "$HOME/FMA_ws/src/stack_gps/tools/waypoints"
python3 live_view.py --csv "$FMA_NEW_CSV"
```

안테나를 기록 경로의 시작점부터 진행 방향으로 천천히 이동하며 확인한다.

- 상태가 `FIXED`인가
- 최근접 `idx`가 대체로 증가하는가
- 실제 경로 중심에서 횡오차가 cm~수십 cm 범위인가
- 내가 경로 오른쪽에 있으면 오른쪽 창의 경로도 차량 기준 오른쪽에 보이는가
- 경로가 되돌아가거나 갑자기 먼 점으로 점프하지 않는가
- 폐곡선이면 시작과 끝이 자연스럽게 연결되는가

하나라도 실패하면 실차 주행에 사용하지 않는다. V2와 V3를 각각 `Ctrl-C`로 종료하고
원인을 고친 뒤 다시 기록한다.

위성지도는 참고용으로만 사용한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
rviz2 -d "$HOME/FMA_ws/src/stack_gps/tools/waypoints/gps_view.rviz"
```

위성 타일 정합은 수 m 오차가 날 수 있으므로 합격 판정은 `live_view.py`로 한다.

---

## 9. 주차 구간 인덱스 확정

통합 주차 launch는 CSV의 인덱스 범위를 사용한다.

- T자/직각 주차: `t_parking_zone_ranges:="[시작,끝]"`
- 평행 주차: `parallel_parking_zone_ranges:="[시작,끝]"`
- 여러 구간: `"[시작1,끝1,시작2,끝2]"`
- 사용하지 않는 종류: `"[0]"`

8단계의 `live_view.py` 화면에서 차량 또는 안테나를 주차 탐색 시작 위치에 두고 최근접
`idx`를 기록한다. 끝 위치에서도 `idx`를 기록한다. 시작은 끝보다 작아야 하고 T자와
평행 구간은 겹치면 안 된다.

예를 들어 T자 시작/끝이 120/140, 평행 시작/끝이 260/285라면 다음과 같다.

```bash
export FMA_T_ZONE="[120,140]"
export FMA_PARALLEL_ZONE="[260,285]"
python3 -c 'import ast,os; a=ast.literal_eval(os.environ["FMA_T_ZONE"]); b=ast.literal_eval(os.environ["FMA_PARALLEL_ZONE"]); pa=list(zip(a[::2],a[1::2])) if a!=[0] else []; pb=list(zip(b[::2],b[1::2])) if b!=[0] else []; assert all(s<e for s,e in pa+pb); assert all(e<x or y<s for s,e in pa for x,y in pb); print("T:",pa,"parallel:",pb)'
```

이 값은 `RUNBOOK_full_operation_20260904.md`의 통합 launch에 넣는다. 숫자를 추측해서
입력하지 않는다.

정지 지점, 회피 허용 구간, GPS 전용 구간은 주차 인덱스와 별개다. 통합 launch를
`go` 없이 띄우고 RTK FIXED 정차 상태에서 `ros2 run stack_gps mark_zone ...`으로
기록하며, 자세한 명령은 `RUNBOOK_lane_gps.md`의 지정 구간 절차를 따른다.

---

## 10. 파일 등록과 통합 런북으로 이동

새 CSV와 베이스의 짝을 `../base_station/BASE_LOCATIONS.md`의 “위치 ↔ 코스 대응”
표에 추가한다. 다음 네 정보를 함께 남긴다.

- 베이스 위치 ID와 설치 사진/설명
- 정확한 CSV 파일명
- 진행 방향과 기록일
- 검증한 T자·평행 주차 인덱스

그다음 통합 운행 문서의 `FMA_COURSE`, `FMA_T_ZONE`, `FMA_PARALLEL_ZONE`을 방금
확정한 값으로 바꾼다.

```text
$HOME/FMA_ws/src/adas_mgm/RUNBOOK_full_operation_20260904.md
```

CSV와 `BASE_LOCATIONS.md`는 팀 자산이므로 검증 후 Git 커밋 대상이다. 로그와 bag은
별도 보관하고 웨이포인트 CSV에 수동으로 행을 삽입하거나 삭제하지 않는다.

---

## 문제 해결

| 증상 | 원인과 조치 |
|---|---|
| `/dev/ttyRover` 없음 | udev 규칙 확인 후 로버 USB 재연결 |
| `Address already in use` | 2101 포트의 이전 `rtcm_server.py` 종료 |
| `Permission denied: /dev/ttyRover` | udev/그룹 적용 후 재로그인, 임의로 sudo 실행하지 않음 |
| 기록기가 시리얼을 못 엶 | `stack_gps_node` 또는 `rtcm_client_inject.py`를 `Ctrl-C`로 종료 |
| RTCM 0 B/s | 베이스 UART2, 라디오/네트워크, 베이스 모드 순서로 확인 |
| 계속 FLOAT | 하늘 시야, 베이스-로버 거리, RTCM 수신, OAK-D USB3 간섭 확인 |
| 측량 표본 0개 | `setup_base.py --disable` 누락 또는 NTRIP 미주입 |
| 기록 점이 거의 없음 | FIXED가 아니거나 이동 거리가 `--spacing`보다 작음 |
| 경로 중간이 긴 직선으로 건너뜀 | FIX가 풀린 구간이므로 파일을 버리고 다시 기록 |
| live view에서 idx가 역행 | 왕복/교차 경로가 한 CSV에 들어갔는지 확인 후 재기록 |
| RTK FIXED인데 코스가 통째로 밀림 | CSV와 베이스 위치/플래시 좌표의 짝이 틀림 |

베이스 측량 문제는 `../base_station/BASE_SURVEY.md`, 등록 지점 재사용은
`../base_station/BASE_MOVE.md`, GPS 주행 검증은 `../../DRIVE_GUIDE.md`를 따른다.
