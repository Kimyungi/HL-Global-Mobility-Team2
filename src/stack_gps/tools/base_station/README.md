# RTK 베이스 스테이션 구축 가이드

> **확정 베이스 좌표 — 현재 운용 지점: 1층 야외 (2026-07-24, 10분 측량, std 수평 0.4/0.6cm·높이 1.3cm):**
> `lat 37.303847372 / lon 127.907283480 / 타원체고 183.8623 m`
> 웨이포인트 지도는 전부 이 좌표 기준 — **안테나를 옮기지 않는 한 절대 변경 금지.**
> 옮기면 재측량(1단계) 후 이 블록과 수신기 설정(2단계)을 함께 갱신할 것.
>
> **지점별 좌표 기록** (베이스를 옮겨 설치할 때는 해당 지점 좌표로 2단계 재설정):
> - **1층 야외** (현재, 2026-07-24): `lat 37.303847372 / lon 127.907283480 / 타원체고 183.8623 m`
> - **옥상 지점** (2026-07-23, 맑음, std 수평 1.2cm/높이 2.1cm): `lat 37.303966817 / lon 127.906898421 / 타원체고 194.2744 m`

**담당: 김윤기** · stack_gps 산출물 "베이스 설치·RTK" (REQUIREMENTS.md)

EVK-F9P를 자체 RTK 베이스로 만들고, 차량의 FST-UEF9P 로버까지 RTCM3 보정을
배달하는 전 과정. 모든 스크립트는 ROS 무의존(표준 python3 + pyserial + pyubx2).

```
[베이스: EVK-F9P+안테나(고정)] --USB--> [베이스 PC] --WiFi/TCP--> [차량 PC] --RS232--> [FST-UEF9P]
     RTCM3 생성 (setup_base.py)      rtcm_server.py         rtcm_client_inject.py    RTK 연산은 내부에서
```

- 베이스는 **인터넷 불필요** (인터넷은 1단계 좌표 측량 때 NGII VRS용으로 딱 한 번).
- 베이스↔차량은 인터넷 없는 로컬 WiFi 공유기면 충분.
- 단일 베이스 RTK는 기준선 ~10km 이내 — 베이스는 시연 현장 근처에 설치.

## 준비물

| 항목 | 비고 |
|---|---|
| EVK-F9P + GNSS 안테나 | 안테나는 하늘 시야 확보된 **고정** 지점 (옥상 난간 등, 움직이면 안 됨) |
| 베이스 PC | 상시 전원. udev 규칙 필요 (아래) |
| WiFi 공유기 | 인터넷 연결 불필요, 베이스 PC·차량 PC를 같은 AP에 |
| USB-RS232 어댑터 (차량) | FST-UEF9P는 RS-232 레벨 (TTL 아님!) 38400bps |
| pyubx2 | `pip3 install --user pyubx2` (pyserial 포함) |

베이스 PC 최초 1회 — udev 규칙 설치 (이 폴더에 포함):

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station
sudo cp 99-ublox-f9p.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
# EVK 연결 후: /dev/ttyF9P (UART1, 115200), /dev/ttyF9P_uart2 (UART2, 38400) 생성 확인
```

## 1단계 — 베이스 안테나 절대좌표 측량 (인터넷 필요, 최초 1회)

베이스를 세울 자리에 안테나를 **최종 위치로 고정**한 뒤 측량한다.
(측량 후 안테나를 옮기면 좌표가 무효 — 다시 측량해야 함.)

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station

# 터미널 A: NGII VRS 보정 주입 (이 측량 단계에서만 사용, 인터넷 필요)
export NGII_USER=kyg100800 NGII_PASS=ngii
python3 ntrip_inject.py

# 터미널 B: RTK FIXED 샘플 10분 수집·평균
python3 measure_base_position.py --duration 600
```

- ⚠ `ublox_gps` ROS 노드(start_rtk.sh)는 꺼둘 것 — UART1 포트가 겹친다.
- carrSoln=FIXED가 떠야 샘플이 쌓인다. FLOAT에 머물면 하늘 시야/NGII 계정 확인.
- 출력된 **위도/경도/타원체고**를 팀 문서에 기록 — 이 좌표는 이후 절대 불변.
  (높이는 해발고도가 아니라 WGS84 타원체고다. 스크립트 출력을 그대로 쓰면 됨.)

## 2단계 — EVK-F9P를 베이스 모드로 설정 (플래시 저장, 최초 1회)

```bash
python3 setup_base.py --lat <1단계 위도> --lon <1단계 경도> --height <1단계 타원체고>
```

하는 일: TMODE3=FIXED + 항법 1Hz + UART2를 RTCM3 전용 출력으로 전환
(1005/1074/1084/1094/1124/1230) + **플래시 저장**. 이후엔 전원만 넣으면 베이스로 동작한다.

- 검증: 스크립트가 NAV-PVT `fixType=5 (TIME)` 를 확인해준다 — 베이스 정상 신호.
- 로버 실습으로 되돌리려면: `python3 setup_base.py --disable`
- 좌표 없이 급하게 돌릴 때(비권장): `python3 setup_base.py --svin` —
  재부팅마다 좌표가 다시 잡혀 **웨이포인트 재현성이 깨진다.** 임시 테스트 전용.

## 3단계 — RTCM 배포 서버 (베이스 PC, 운용 중 상시 실행)

```bash
python3 rtcm_server.py          # /dev/ttyF9P_uart2 @38400 → tcp 0.0.0.0:2101
```

- 10초마다 통계 출력. 정상이면 **수백 B/s** 수준의 RTCM 유입이 보인다.
  `0 B/s ⚠` 이면 2단계 미완 또는 포트 오류.
- 베이스 PC IP를 고정(공유기 DHCP 예약)해두면 차량 쪽 설정이 편하다.

## 4단계 — 차량(로버) 연결 검증

FST-UEF9P를 로버 PC에 연결한다. 연결 방식은 둘 다 가능:

- **USB (개발·테스트 권장)**: FST의 USB를 직결 → udev 규칙이 `/dev/ttyRover`로
  고정해준다 (u-blox 네이티브 USB라 baud 무관, NMEA 출력·RTCM 입력 동시 지원)
- **RS-232 (차량 최종 탑재)**: USB-RS232 어댑터 경유, 38400bps 고정 → `/dev/ttyUSB0`

```bash
python3 rtcm_client_inject.py --host <베이스PC_IP> --serial /dev/ttyRover --monitor
```

- `RTK 상태: RTK FIXED` (GGA quality 4)가 뜨면 **베이스-로버 링크 완성**.
- FLOAT(5)에 머물면: 로버 하늘 시야, 서버 B/s, 기준선 거리 순으로 점검.
- FST 도착 전 리허설: EVK를 잠시 `--disable`로 로버로 되돌려 쓸 수는 없다
  (베이스가 사라지므로). 리허설엔 실습용 F9P가 하나 더 필요하다.

## 팀원 PC를 베이스 PC로 세팅하기 (처음부터 끝까지)

EVK-F9P 수신기는 이미 베이스로 설정 완료(플래시 저장)라서, **팀원 PC에서는
수신기 설정을 건드릴 필요가 전혀 없다.** 1·2단계 스크립트 실행 금지 —
잘못 실행하면 확정 좌표가 날아간다. 할 일은 "USB에서 나오는 RTCM을 WiFi로
중계"하는 것뿐이며, ROS도 colcon 빌드도 필요 없다.

```bash
# ① 저장소 클론 (이미 있으면 git pull)
git clone <팀 저장소 URL> ~/FMA_ws

# ② 파이썬 패키지 (rtcm_server.py는 pyserial만 필요)
pip3 install --user pyserial

# ③ udev 규칙 (최초 1회)
cd ~/FMA_ws/src/stack_gps/tools/base_station
sudo cp 99-ublox-f9p.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger

# ④ EVK를 USB로 연결 → 포트 생성 확인
ls -l /dev/ttyF9P_uart2      # 없으면 케이블 재연결 후 다시 확인

# ⑤ 서버 실행 (운용 내내 켜둠)
python3 rtcm_server.py
```

정상 판정: 10초마다 나오는 통계에 `RTCM ~500 B/s` 수준이 찍히면 성공.
`0 B/s ⚠`가 계속되면 EVK 연결/포트를 확인한다 (수신기 자체는 전원만 들어오면
송출하므로, 0 B/s는 거의 항상 케이블·포트 문제다).

체크리스트:
- [ ] 안테나는 **확정 좌표를 측량한 바로 그 위치**에 고정 (이 문서 맨 위 좌표)
- [ ] 베이스 PC와 로버 PC가 **같은 WiFi 공유기**에 접속 (인터넷 불필요)
- [ ] 베이스 PC의 IP 확인(`hostname -I`) 후 로버 쪽에 알려주기
  (공유기 관리 페이지에서 DHCP 예약으로 IP 고정 권장)
- [ ] 노트북 절전(덮개 닫힘 포함) 끄기 — 서버가 멈추면 로버 FIX가 풀린다

로버 PC(윤기 노트북 또는 차량 PC)에서는:

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station
python3 rtcm_client_inject.py --host <베이스PC_IP> --monitor
```

## 5단계 — 운용 절차 (시연 당일)

1. 베이스 안테나를 **측량했던 바로 그 위치**에 설치, EVK 전원 인가
2. 베이스 PC에서 `rtcm_server.py` 실행 (인터넷 불필요)
3. 차량 PC에서 `rtcm_client_inject.py` 실행 → RTK FIXED 확인
4. stack_gps 노드 기동 (FST NMEA → `/perception/gps_path`) — 추후 구현

⚠ **베이스를 옮기면 반드시 1단계부터 재측량.** 저장된 FIXED 좌표가 실제 위치와
다르면 로버는 FIX가 떠도 위치 전체가 그 오차만큼 밀린다. 웨이포인트 지도도 같은
베이스 좌표 기준일 때만 유효하다.

## 트러블슈팅

| 증상 | 점검 |
|---|---|
| `/dev/ttyF9P` 없음 | udev 규칙 설치·재트리거, `lsusb`로 1546:0507/0508 확인 |
| measure가 첫 줄에서 멈춤 (UBX 무수신) | 이전 세션 잔재 설정 가능성 — UART1 출력 프로토콜 꺼짐(`CFG_UART1OUTPROT_UBX=0`). 스크립트가 자동으로 켜지만, 구버전이면 VALSET으로 1 설정 |
| 보정 주입해도 carrSoln=NONE 고착 (diffSoln=1이어도) | diffSoln=1은 SBAS일 수 있음. RXM-RTCM으로 RTCM 유입 확인 — 0건이면 **UART2 baud 불일치** (실제 사례: 잔재 설정 115200 vs 주입기 38400). UART2를 38400으로 통일 |
| VALSET 응답 없음 | 포트에 다른 프로세스(ROS 노드 등)가 붙어 있는지 확인 |
| fixType=5 안 뜸 | 안테나 하늘 시야, 입력 좌표 오타(도 단위, 타원체고) 확인 |
| 서버 0 B/s | UART2 baud 불일치(기본 38400), 2단계 재실행 |
| 로버 FLOAT 고착 | 기준선 거리, 로버 안테나 설치(차량 지붕 중앙, 금속 그라운드) |
