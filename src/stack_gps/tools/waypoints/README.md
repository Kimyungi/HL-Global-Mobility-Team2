# 웨이포인트 기록 도구

RTK FIXED 상태의 로버(FST-UEF9P)를 들고/싣고 경로를 이동하면 고정밀 좌표를
CSV로 기록한다. 기록된 웨이포인트는 stack_gps 노드가 읽어 `/perception/gps_path`
ref points의 원천이 된다.

## 사용법

```bash
# ⚠ rtcm_client_inject.py 가 돌고 있으면 먼저 종료 (Ctrl-C) — 이 도구가 주입까지 겸한다
cd ~/FMA_ws/src/stack_gps/tools/waypoints
python3 record_waypoints.py --host 172.20.10.2                   # 기본 (0.2m 간격)
python3 record_waypoints.py --host 172.20.10.2 --name track_A    # 트랙 이름 지정
```

- **RTK FIXED에서만 기록** — FLOAT로 떨어지면 자동 일시정지, 복귀하면 재개.
- 직전 점에서 `--spacing`(기본 0.2m) 이상 움직여야 새 점이 찍힌다
  (제자리에 서 있어도 점이 쌓이지 않음).
- Ctrl-C로 종료 → `src/stack_gps/waypoints/waypoints_*.csv` 저장.

## CSV 포맷

| 열 | 의미 |
|---|---|
| lat / lon | WGS84 십진도 (NMEA GGA 소스 — 아래 정밀도 참고) |
| height_m | 타원체고 (GGA MSL고도 + 지오이드 분리값) |
| east_m / north_m | 첫 점 기준 로컬 미터 좌표 (검토·시각화용) |
| quality | GGA quality (4=RTK FIXED) |

**정밀도**: FST-UEF9P의 USB는 출하 설정상 **NMEA 출력 + RTCM 입력만 허용**하고
UBX 설정 명령을 받지 않는다 (실측 확인 — 고정밀 NMEA 모드도 켤 수 없음).
GGA 좌표는 분 소수 5자리 = 약 1.85cm 양자화이므로, RTK 오차와 합산한
웨이포인트 유효 정밀도는 **2~3cm**. 주행 제어 요구 수준에는 충분하다.

## 기록 요령

- 안테나를 몸 앞이 아니라 **머리 위로** 들면 몸에 의한 신호 가림이 줄어든다.
- 차량 궤적을 딸 때는 차 지붕 중앙(최종 장착 위치)에 안테나를 두고 저속 주행.
- 건물·나무 옆에서 FIX가 풀리면 그 구간은 기록이 비니, 잠시 멈춰 FIXED 복귀 후 진행.
- 같은 경로를 왕복하지 말 것 — 한 방향 한 번이 한 트랙. 다른 경로는 `--name`을 바꿔 별도 파일로.

## 주의

- 웨이포인트의 절대좌표는 **베이스 확정 좌표에 묶여 있다** (base_station/README.md).
  베이스 좌표가 바뀌면 기존 웨이포인트 지도는 전부 무효.
- `waypoints/*.csv`는 커밋 대상 — 지도 데이터도 팀 자산이다.
