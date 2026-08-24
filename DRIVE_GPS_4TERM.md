# GPS 단독 주행 — 터미널 4개 현장 카드 (2026-08-25, 한라대)

출처: `stack_gps/DRIVE_GUIDE.md` · `COMMANDS.md` · `HANDOVER.md` · `CLAUDE.md` §5.7
이 PC(Xanadu-book5) 실측 상태를 반영해 축약했다. 라이다·차선·신호등은 쓰지 않는다.

```
베이스(한라대 halla_20260819) --라디오--> [T1 중계] --TCP2101--> [T2 stack_gps] --> 로버
                                          [T1 브리지] --CAN--> dSPACE
```

---

## 0. 사전 점검 — 30초

```bash
ls -l /dev/ttyRover /dev/ttyRadio     # 둘 다 보여야 함
ip link show can0                     # state UP
```

베이스 PC에서 `rtcm_server.py --radio /dev/ttyRadio` 가 돌고 있어야 한다(현장 도착 시 1회).

---

## T1 — 상시 묶음 (RTCM 중계 + CAN 브리지 + MGM)

```bash
~/FMA_ws/gps_standing.sh
```

- 정상: 10초마다 `RTCM ~500 B/s`
- `0 B/s ⚠ RTCM 없음` → 베이스 가동 여부 → USB 허브 끊김 순으로 확인
- **Ctrl-C 하면 브리지를 먼저 죽이고 → `can_zero --once` 로 dSPACE 목표값 0 복귀 후 종료한다.**
  dSPACE watchdog 이 미구현이라(HANDOVER §3.6) 이 가드가 없으면 마지막 v_ref 를 무기한 유지한다.
  `bridge.launch.py` 를 직접 띄우면 이 가드가 **없다** — 반드시 이 스크립트를 쓸 것.

## T2 — stack_gps (★ 이게 RTK 시계를 시작한다)

```bash
ros2 run stack_gps stack_gps_node --ros-args \
    -p waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv \
    -p rtcm_host:=127.0.0.1 \
    -p imu_port:=/dev/ttyUSB4 \
    -p error_log_csv:=$HOME/FMA_ws/drive_logs/lateral_$(date +%m%d_%H%M).csv
```

- **CSV 는 반드시 한라대 것** — 베이스 플래시가 `halla_20260819` 다. 짝이 어긋나면 트랙이 통째로 평행이동한다.
- `imu_port` 를 손으로 지정하는 이유: 이 PC 에 `/dev/ttyUSB_IMU` udev 별칭이 없다.
  안 주면 `IMU:없음` 으로 뜨고 헤딩이 COG/접선 폴백으로 떨어진다. IMU 를 안 쓸 거면 `imu_port:=off`.
- 정상: 2초마다 `FIXED age 0.2s RTCM ...B/s idx N 횡오차 0.0Xm`
- T1 로그의 `클라이언트 0` → `1` 로 바뀌면 로버에 보정이 들어가기 시작한 것이다.
- **FIXED 까지 5~10분**(실측 7분). 이걸 켜기 전에는 시계가 돌지 않는다.

## T3 — 블랙박스 (출발 직전에 켠다)

```bash
cd ~/FMA_ws/src/stack_gps/tools/drive_log
./record_drive.sh run1 $HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv
```

`~/FMA_ws/drive_logs/run1_<시각>/` 에 bag + CAN 원시 + meta + 트랙 사본이 남는다.
사고·이상이 나도 **T3 는 끄지 말 것.**

## T4 — 출발 스위치 ★ 반드시 독립 창

```bash
python3 ~/FMA_ws/src/stack_gps/tools/drive_log/manual_go.py
```

**실행 = 출발 / Ctrl-C = 250ms 내 정지.** estop 해제 하트비트를 50ms 로 발행해
MGM 의 출발 조건(§5.7)을 채우는 소프트웨어 레버다.

> ⚠ **돌발 장애물 자동 정지가 없다.** 유일한 비상 수단은 물리 비상정지와 Ctrl-C 뿐.
> 저속·개활지·비상정지 담당 배치 필수. stack_estop 과 동시 실행 금지.

---

## 출발 전 확인 4줄

```bash
ros2 topic echo /perception/gps_path --once | grep fix_quality   # 4 = RTK FIXED
ros2 topic echo /perception/gps_path --once | head -20           # points[0].x > 0 (차 앞)
candump -n 3 can0                                                # 0x200/0x201/0x202 = dSPACE 회신
ros2 run adas_mgm state                                          # WAYPOINT(GPS)
```

첫 점이 차량 뒤(x<0)면 차가 트랙을 등지고 있는 것 — 돌려 세운다.
IMU 는 전원 직후 `미정렬(직진 주행 필요)` 이 정상이고, 첫 주행에서 몇 초 직진하면 COG 로 자동 정렬된다.

## 세우는 법 (급한 순서)

1. 물리 비상정지
2. **T4 Ctrl-C** (250ms 내 정지)
3. 코스 완주 자연 정지

## 철수 순서

T4 → T3 → T2 → **T1 마지막** (목표값 0 복귀 가드가 T1 에 있다)

---

## 이 PC 의 알려진 상태 (2026-08-25 실측)

| 항목 | 상태 |
|---|---|
| 워크스페이스 | main `c3f01bb` (CAN v2). v3 는 8/24 적용 후 롤백 — `ROLLBACK_v3_20260824.md` |
| dSPACE | **회신 없음** (can0 RX 0). 전원·모델 가동 확인 필요 |
| IMU 별칭 | 없음 → `imu_port:=/dev/ttyUSB4` 로 지정 |
| udev ③ (로버 USB 리셋) | **미설치** — 설치하면 NMEA 무수신 20초에 자동 복구 |
| 한라대 구간 파일 | 없음 (`zones_*.yaml`) — GPS 단독 주행에는 무관 |
| 라이다 | 4대 인식되나 GPS 단독에서는 미사용. 전방 T-mini 는 ttyUSB0/ttyUSB2 중 미확정 |

## 트러블슈팅 (DRIVE_GUIDE §7 발췌)

| 증상 | 점검 |
|---|---|
| FIXED 안 뜸 | 베이스 B/s → T1 B/s → 로버 하늘 시야 순으로 상류부터 |
| gps_path 빈 배열 | quality 0 이면 fix stale, 4 인데 비면 CSV 경로 오타 |
| target_ref 나오는데 바퀴 무반응 | `candump` 로 CAN TX 확인 → dSPACE 수신 확인(손상민) |
| 아예 안 움직임 | `/perception/estop` 발행 여부(T4) → gps_path fix_quality → candump 0x100 counter |
| 상태 로그 `IMU:없음` | `imu_port:=/dev/ttyUSB4` 를 안 준 것 |
| 곡선에서 크게 이탈 | 기록이 너무 빨랐음 — `--spacing 0.2` 로 재기록 |
