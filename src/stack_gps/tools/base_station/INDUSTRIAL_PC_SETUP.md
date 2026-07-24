# 산업용 PC — 임시 베이스 PC 세팅 가이드 (Tailscale 경유)

> 이 문서는 산업용 PC의 Claude Code가 그대로 실행할 수 있게 작성됨.
> 목표: **산업용 PC = 임시 베이스** (EVK-F9P 연결 + RTCM을 Tailscale VPN으로 배포).
> 로버는 김윤기 노트북(FST-UEF9P). 추후 산업용 PC는 차량 로버 PC로 역할이 바뀐다 —
> 그때도 Tailscale/저장소 세팅은 그대로 재사용되므로 지금 작업이 낭비가 아니다.

## ⚠ 먼저 읽을 것 (Claude Code 준수사항)

1. **EVK-F9P 수신기는 이미 베이스로 설정 완료**(TMODE FIXED + RTCM 출력, 플래시 저장).
   `setup_base.py` / `measure_base_position.py` / `ntrip_inject.py` **실행 금지** —
   실행하면 확정 좌표(README.md 맨 위)가 오염된다. 이 PC에서 할 일은 "중계"뿐이다.
2. 안테나는 **확정 좌표를 측량한 그 지점**에 고정되어 있어야 한다 (README.md 참조).
3. sudo가 필요한 명령은 사용자에게 비밀번호를 요청할 것.

## 0단계 — 저장소

```bash
git clone https://github.com/Kimyungi/HL-Global-Mobility-Team2.git ~/FMA_ws
# 이미 있으면: cd ~/FMA_ws && git pull
```

## 1단계 — Tailscale 설치 + 로그인

```bash
curl -fsSL https://tailscale.com/install.sh | sh     # sudo 포함
sudo tailscale up
# → 출력되는 https://login.tailscale.com/a/... 주소를 브라우저로 열어
#   ★ 팀 공용 계정(노트북과 동일 계정)으로 로그인 ★
```

로그인 완료 후 확인:

```bash
tailscale status          # 자기 기기 + 상대 기기(노트북)가 보여야 함
tailscale ip -4           # 이 PC의 고정 주소 (100.x.y.z) — 로버에게 전달할 값
```

**성공 판정**: `tailscale status`에 두 기기가 보이고, 노트북 쪽 100.x 주소로
`ping -c 3 <노트북_100.x.y.z>` 응답이 오면 VPN 완성.

## 2단계 — 베이스 의존성 (ROS·빌드 불필요)

```bash
pip3 install --user pyserial

cd ~/FMA_ws/src/stack_gps/tools/base_station
sudo cp 99-ublox-f9p.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

## 3단계 — EVK 연결 확인

EVK-F9P를 USB로 연결(전원 겸용, 별도 전원 불필요) 후:

```bash
ls -l /dev/ttyF9P /dev/ttyF9P_uart2    # 두 심볼릭 링크가 생겨야 정상
# 안 보이면: 케이블 재연결 → lsusb 에서 1546:0507 / 1546:0508 확인
```

## 4단계 — RTCM 서버 실행 (운용 내내 켜둠)

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station
python3 rtcm_server.py
```

**성공 판정**: 10초마다 나오는 통계에 `RTCM ~500 B/s` 수준이 찍히면 정상.
`0 B/s ⚠`가 계속되면 EVK 케이블/포트 문제 (수신기는 전원만 오면 송출한다).
Tailscale은 방화벽 없이 기기 간 직통이므로 포트 개방 작업은 필요 없다.

## 5단계 — 로버에게 알릴 것

1. 이 PC의 Tailscale IP (`tailscale ip -4` 결과)

로버(노트북) 쪽에서는 다음으로 접속한다 (참고용 — 노트북에서 실행):

```bash
python3 rtcm_client_inject.py --host <이PC의_100.x.y.z> --serial /dev/ttyRover --monitor
```

로버 터미널에 `RTK 상태: RTK FIXED`가 뜨면 전체 링크 완성.

## 트러블슈팅

- 서버/포트 문제: `README.md`의 트러블슈팅 표 참조 (같은 폴더).
- Tailscale 연결 안 됨: 양쪽 모두 인터넷 연결 확인 → `tailscale status`에서
  상대 기기 오프라인 여부 확인 → `sudo tailscale up` 재실행.
- 노트북 절전 금지 (서버가 아니라 로버라도, 끊기면 FIX 풀림).

## (참고) 추후 역할 전환 — 산업용 PC를 로버로

차량 탑재 시: 이 PC에 FST-UEF9P를 USB 연결(udev 규칙이 `/dev/ttyRover` 생성)
→ 위 5단계의 클라이언트 명령을 이 PC에서 실행하고, 그때의 베이스(노트북 또는
전용 베이스 PC)의 Tailscale IP를 `--host`에 넣으면 된다. Tailscale IP는 기기 고정이라
역할이 바뀌어도 주소는 그대로다.
