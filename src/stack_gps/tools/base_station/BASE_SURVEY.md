# 베이스 좌표 측량 — 새 지점을 세울 때

안테나를 **처음 세우는 자리**에서 그 자리의 절대좌표를 확정하는 절차. 인터넷이 필요하고
(NGII VRS 보정), 지점당 한 번만 한다.

> **이미 등록된 지점으로 옮기는 거면 이 문서가 아니다** → [`BASE_MOVE.md`](BASE_MOVE.md).
> 등록된 지점은 **재측량하면 안 된다** — 같은 자리라도 값이 cm 단위로 달라져 그
> 지점에서 기록한 코스가 그만큼 밀린다.

전제: 베이스 PC 세팅(udev 규칙·pyubx2)이 끝나 있을 것 → [`README.md`](README.md) 준비물.

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station
```

---

## 0. 덮기 전 백업 — 지금 들어 있는 좌표부터 확인

```bash
python3 read_base_position.py
```

수신기 플래시에는 좌표가 **한 벌만** 들어간다. 새 좌표를 쓰면 지금 값은 덮인다.
출력된 좌표를 [`BASE_LOCATIONS.md`](BASE_LOCATIONS.md) 표와 대조할 것.

- 표에 **있으면** 그냥 진행 (숫자가 남아 있으니 언제든 복원된다)
- 표에 **없으면** 출력 그대로 표에 행을 추가한 뒤 진행 — 안 그러면 그 좌표는 영영 사라지고,
  그 지점에서 기록한 코스 CSV 도 통째로 못 쓰게 된다

## 1. 안테나 고정

- 하늘 시야가 트인 곳. 벽·처마 밑 금지
- **최종 위치로 고정**한다. 측량이 끝난 뒤 옮기면 좌표가 무효다
- **삼각대 높이까지 그대로 유지.** 다음에 이 지점을 다시 쓸 사람이 재현해야 하므로
  사진을 찍어 두고 §5 표의 "안테나 설치" 칸에 적는다

## 2. 베이스 모드 해제 ← 빼먹으면 측량이 영원히 안 된다

```bash
python3 setup_base.py --disable
```

직전 현장에서 베이스로 쓰던 수신기라면 `TMODE FIXED` 상태다. 베이스 모드에서는
**측위를 하지 않아** 샘플이 한 개도 안 쌓인다(`fixType=5`). 10분 기다렸다 허탕 치기
딱 좋은 자리다 — 스크립트가 감지하면 경고 후 종료한다.

- `ublox_gps` ROS 노드(`start_rtk.sh`)가 떠 있으면 끌 것 — UART1 포트가 겹친다

## 3. [터미널 A] NGII VRS 보정 주입 (인터넷 필요)

```bash
export NGII_USER=kyg100800 NGII_PASS=ngii
python3 ntrip_inject.py --lat 37.3038 --lon 127.9073      # 한라대
```

`--lat/--lon` 은 **현장 개략 좌표**다 — VRS 가 이 위치 기준으로 가상 기준국 보정을
만든다. 현장이 바뀌면 반드시 갱신할 것.

| 현장 | 인자 |
|---|---|
| 한라대학교 | `--lat 37.3038 --lon 127.9073` |
| 원주 운전면허시험장 | `--lat 37.3003 --lon 127.9795` |

**NGII 계정 함정 (2026-08-18 정리):**

- 비밀번호 `ngii` 는 계정별 값이 아니라 **전 사용자 공통 고정값**이다(NGII 공식 FAQ).
  401 이 떠도 비번을 의심하지 말 것 — 재발급 대상이 아니다.
- **계정당 동시접속 1개.** 다른 PC 가 같은 ID 로 물고 있으면 실패한다. PC 마다 통합 ID 를
  따로 발급받는다: geodesy.ngii.go.kr → 마이페이지 → 통합회원 연계 → 등록
- 캐스터는 **RTS1** 이 기본이다. **RTS2 는 정상 계정도 401 로 거부한다** — 계정 문제가
  아니라 캐스터 측 문제(신·구 ID 모두 RTS2 401 / RTS1 200 확인)
- 접속만 먼저 보려면(F9P 불필요): `python3 ntrip_check.py kyg100800`

## 4. [터미널 B] 10분 측량

```bash
python3 measure_base_position.py --duration 600
```

- `carrSoln=FIXED` 가 떠야 샘플이 쌓인다. FLOAT 에 머물면 하늘 시야 → NGII 접속 순으로 점검
- 끝나면 **위도 / 경도 / 타원체고**와 각 표준편차가 나온다. 표준편차 경고가 뜨면 다시
- 출력 맨 아래에 다음 단계에 쓸 `setup_base.py` 커맨드가 그대로 찍힌다
- 높이는 해발고도가 아니라 **WGS84 타원체고**다

## 5. 좌표 등록 ← 여기서 안 적으면 다음에 못 쓴다

[`BASE_LOCATIONS.md`](BASE_LOCATIONS.md) 표에 행을 추가한다. **측량 직후 바로.**
`measure_base_position.py` 는 결과를 화면에만 찍고 파일로 남기지 않는다.

| 칸 | 예 |
|---|---|
| 위치 ID | `halla_20260819` |
| 장소 | 한라대학교 |
| 안테나 설치 | 어디에 어떻게 세웠는지 — **다음에 재현할 사람이 읽는다** |
| 위도 / 경도 / 타원체고 | §4 출력 그대로 |
| 측량일 / 상태 | 2026-08-19 / 현재 사용 |

기존 행은 **덮어쓰지 않는다.** 지점을 늘려 가며 쌓는 표다.

## 6. 베이스 모드 설정 (플래시 저장)

```bash
python3 setup_base.py --lat <§4 위도> --lon <§4 경도> --height <§4 타원체고>
```

`TMODE3=FIXED` + 항법 1Hz + UART2 를 RTCM3 전용 출력(1005/1074/1084/1094/1124/1230)으로
전환하고 **플래시에 저장**한다. 이후엔 전원만 넣으면 베이스로 동작한다.

- 검증: 스크립트가 `fixType=5 (TIME — 베이스 정상)` 을 확인해 준다
- 로버 실습으로 되돌리려면 `python3 setup_base.py --disable`
- ⚠ `--svin`(survey-in)은 임시 테스트 전용이다. 재부팅마다 좌표가 다시 잡혀
  **웨이포인트 재현성이 깨진다**

## 7. 송출 시작 → 로버 확인

```bash
# B1 [베이스 PC] — 운용 내내 켜 둠
python3 rtcm_server.py --radio /dev/ttyRadio

# V1 [차량 PC] — 라디오 → 로컬 TCP 중계, 운용 내내 켜 둠
python3 ~/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py \
    --port /dev/ttyRadio --tcp-port 2101

# 로버 RTK FIXED 확인
python3 ~/FMA_ws/src/stack_gps/tools/rtk_probe.py --seconds 120
```

정상 판정: B1 통계에 `RTCM ~500 B/s`. `0 B/s ⚠` 는 거의 항상 케이블·포트 문제다.

**C/N0 를 볼 것.** 위성 수·HDOP 는 정상인데 RTK 만 무너지면 OAK-D USB3 간섭이다
(39dB → 22dB). 카메라는 `usb_speed:=high camera_fps:=10` 으로 USB2 에 묶는다
(CLAUDE.md §6).

## 8. 코스 기록 — 새로 측량했으면 **반드시** 다시

웨이포인트 절대좌표 = 베이스 좌표 + RTK 기선이다. 베이스를 새로 측량했으면 그 지점의
**옛 코스는 그대로 못 쓴다.**

```bash
# stack_gps 노드는 꺼둘 것 — FST 포트를 한 프로세스만 쓸 수 있다
cd ~/FMA_ws/src/stack_gps/tools/waypoints
python3 record_waypoints.py --host 127.0.0.1 --name <코스이름> --spacing 0.3
python3 live_view.py --csv ../../waypoints/waypoints_<코스이름>_*.csv   # 품질 눈검사
```

요령: FIXED 확인 후 출발 / 시작점 3초 정지 / 주행보다 느리게, 조향 부드럽게 /
**급커브는 조향 70~80%만** (풀조향으로 기록하면 경로가 조향 한계 100%를 요구해
추종 보정 여유가 0 이 되고 커브 바깥으로 이탈한다 — 2026-08-06 S자 실측 87%가
바깥쪽, 최대 0.44m) / 폐곡선이면 시작·끝 3cm 이내면 합격 / "FIX 아님" 경고가 떴던
run 은 버리고 다시.

상세: `stack_gps/DRIVE_GUIDE.md` §A2.

마지막으로 [`BASE_LOCATIONS.md`](BASE_LOCATIONS.md) 의 **"위치 ↔ 코스 대응"** 표에
새 코스를 추가한다.

---

## 문제가 생기면

| 증상 | 점검 |
|---|---|
| 샘플이 하나도 안 쌓임 | §2 `--disable` 을 안 했다 (`fixType=5`) |
| measure 가 첫 줄에서 멈춤 (UBX 무수신) | UART1 출력 프로토콜 꺼짐 — 스크립트가 자동으로 켜지만 구버전이면 VALSET 으로 `CFG_UART1OUTPROT_UBX=1` |
| carrSoln=NONE 고착 | RXM-RTCM 으로 RTCM 유입 확인 → 0건이면 **UART2 baud 불일치**(38400 로 통일) |
| NTRIP 401 | 비번이 아니라 ① 동시접속 1개 ② RTS2 사용 을 의심 |
| VALSET 응답 없음 | 포트에 다른 프로세스(ROS 노드 등)가 붙어 있는지 |
| `fixType=5` 안 뜸 | 안테나 하늘 시야, 입력 좌표 오타(도 단위·타원체고) |

더 많은 증상은 [`README.md`](README.md) 트러블슈팅 표.
