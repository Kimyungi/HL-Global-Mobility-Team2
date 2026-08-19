# RTK 베이스 스테이션 구축 가이드

EVK-F9P 를 자체 RTK 베이스로 만들고 차량의 FST-UEF9P 로버까지 RTCM3 보정을 배달하는
전 과정. 모든 스크립트는 ROS 무의존(표준 python3 + pyserial + pyubx2).

## 이 폴더의 문서

| 하려는 것 | 문서 |
|---|---|
| **지금 어느 좌표가 등록돼 있나 / 어느 코스가 어느 지점 것인가** | [`BASE_LOCATIONS.md`](BASE_LOCATIONS.md) — 좌표 레지스트리 |
| **처음 세우는 자리의 좌표를 측량한다** | [`BASE_SURVEY.md`](BASE_SURVEY.md) |
| **이미 등록된 지점끼리 옮긴다** | [`BASE_MOVE.md`](BASE_MOVE.md) |
| 팀원 PC 를 베이스 PC 로 세팅한다 | 이 문서 아래 |
| 산업용 PC 세팅 | [`INDUSTRIAL_PC_SETUP.md`](INDUSTRIAL_PC_SETUP.md) |

이 문서는 **하드웨어 구성·환경 세팅·라디오 운용·트러블슈팅**을 담는다. 좌표를 다루는
절차는 위 세 문서에 있다.

> ⚠ **좌표는 플래시에 한 벌만 들어간다.** 덮기 전에 항상
> `python3 read_base_position.py` 로 현재 값을 확인하고, 레지스트리에 없는 값이면
> 먼저 표에 적을 것. 안 그러면 그 지점의 코스 CSV 가 통째로 못 쓰게 된다.

**담당: 김윤기** · stack_gps 산출물 "베이스 설치·RTK" (REQUIREMENTS.md)

EVK-F9P를 자체 RTK 베이스로 만들고, 차량의 FST-UEF9P 로버까지 RTCM3 보정을
배달하는 전 과정. 모든 스크립트는 ROS 무의존(표준 python3 + pyserial + pyubx2).

```
[베이스: EVK-F9P+안테나(고정)] --USB--> [베이스 PC] --WiFi/TCP--> [차량 PC] --RS232--> [FST-UEF9P]
     RTCM3 생성 (setup_base.py)      rtcm_server.py         rtcm_client_inject.py    RTK 연산은 내부에서
```

- 베이스는 **인터넷 불필요** (인터넷은 좌표 측량 때 NGII VRS 용으로 딱 한 번).
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

## 좌표 측량·설정 → 별도 문서

베이스 안테나 절대좌표 측량(NGII VRS 보정 + 10분 수집)과 `TMODE3=FIXED` 플래시 저장은
[`BASE_SURVEY.md`](BASE_SURVEY.md) 에 있다. 이미 등록된 지점으로 옮기는 것뿐이라면
[`BASE_MOVE.md`](BASE_MOVE.md) — **재측량하면 안 된다.**

## RTCM 배포 서버 (베이스 PC, 운용 중 상시 실행)

```bash
python3 rtcm_server.py          # /dev/ttyF9P_uart2 @38400 → tcp 0.0.0.0:2101
```

- 10초마다 통계 출력. 정상이면 **수백 B/s** 수준의 RTCM 유입이 보인다.
  `0 B/s ⚠` 이면 베이스 모드 설정 미완(`BASE_SURVEY.md` §6) 또는 포트 오류.
- 베이스 PC IP를 고정(공유기 DHCP 예약)해두면 차량 쪽 설정이 편하다.

## 차량(로버) 연결 검증

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
수신기 설정을 건드릴 필요가 전혀 없다.** `setup_base.py`·`measure_base_position.py`
실행 금지 — 잘못 실행하면 플래시의 확정 좌표가 날아간다. 할 일은 "USB에서 나오는 RTCM을 WiFi로
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

## 텔레메트리 라디오 운용 (Holybro SiK 915MHz — 무선, 인터넷·WiFi 불필요)

2026-07-31 실기기 검증 완료: EVK → 베이스 PC → 라디오 → **전파** → 수신 라디오,
RTCM 무손실 통과. 라디오 두 대 모두 시리얼 속도 **38,400으로 변경·저장됨**
(공장값 57,600 — 새 라디오 교체 시 AT 명령 `ATS1=38, AT&W, ATZ`로 재설정 필요).

**베이스 (PC 브릿지 방식 — 표준 운용):**

```bash
# EVK와 라디오를 모두 베이스 PC USB에 연결 (udev 규칙 설치 시 /dev/ttyRadio 자동 생성)
python3 rtcm_server.py --radio /dev/ttyRadio    # TCP(Tailscale)와 무선 동시 송출
```

**로버:**

```bash
# 라디오를 로버 PC USB에 연결
python3 rtcm_client_inject.py --from-serial /dev/ttyRadio --serial /dev/ttyRover --monitor
```

주의·기록:
- **라디오 JST 커넥터에 EVK를 직결(PC 제거)하는 방식은 보류.** 배선(TXD2→RX, GND)을
  전 구간 검증했음에도 통신 불가 — 라디오가 USB로 전원을 받는 동안 내부 USB 칩이
  JST RX 입력을 점유하는 회로 특성으로 결론. 직결하려면 USB 대신 **JST의 5V 핀으로
  급전**해야 할 것으로 추정 (추후 재시도, 납땜 전선은 보존).
- 보조배터리는 저부하(라디오+EVK ≈ 0.3A) 시 **자동 꺼짐**이 있는 모델이 많다 —
  현장 투입 전 10분 유지 확인 필수.
- 실내에서는 1005/1230만 송출된다(~35B/s). MSM 관측 메시지는 안테나가 하늘을 봐야
  나오며 그때 총 유량 ~560B/s (라디오 용량 대비 여유).

## 운용 절차 (현장 도착 시)

1. 베이스 안테나를 **그 지점의 등록된 자리·높이**에 설치, EVK 전원 인가
   (`BASE_LOCATIONS.md` 의 "안테나 설치" 칸)
2. 베이스 PC에서 `rtcm_server.py` 실행 (인터넷 불필요)
3. 차량 PC에서 `rtcm_client_inject.py` 실행 → RTK FIXED 확인
4. stack_gps 노드 기동 (FST NMEA → `/perception/gps_path`)

⚠ 저장된 FIXED 좌표가 실제 안테나 위치와 다르면 로버는 FIX 가 떠도 **위치 전체가 그
오차만큼 밀린다.** 웨이포인트도 같은 베이스 좌표 기준일 때만 유효하다.
지점을 옮길 때는 [`BASE_MOVE.md`](BASE_MOVE.md), 처음 세우는 자리면
[`BASE_SURVEY.md`](BASE_SURVEY.md).

## 트러블슈팅

| 증상 | 점검 |
|---|---|
| `/dev/ttyF9P` 없음 | udev 규칙 설치·재트리거, `lsusb`로 1546:0507/0508 확인 |
| 측량이 안 됨 (샘플 0, UBX 무수신 등) | [`BASE_SURVEY.md`](BASE_SURVEY.md) 트러블슈팅 표 |
| 보정 주입해도 carrSoln=NONE 고착 (diffSoln=1이어도) | diffSoln=1은 SBAS일 수 있음. RXM-RTCM으로 RTCM 유입 확인 — 0건이면 **UART2 baud 불일치** (실제 사례: 잔재 설정 115200 vs 주입기 38400). UART2를 38400으로 통일 |
| VALSET 응답 없음 | 포트에 다른 프로세스(ROS 노드 등)가 붙어 있는지 확인 |
| fixType=5 안 뜸 | 안테나 하늘 시야, 입력 좌표 오타(도 단위, 타원체고) 확인 |
| 서버 0 B/s | UART2 baud 불일치(기본 38400) — `setup_base.py` 재실행 |
| 로버 FLOAT 고착 | 기준선 거리, 로버 안테나 설치(차량 지붕 중앙, 금속 그라운드) |
