# 용인 운전면허시험장 베이스 GPS 최종 설정

용인 운전면허시험장에서 확정한 RTK 베이스 좌표와 현장 재가동 절차다.

## 확정 좌표

| 항목 | 값 |
|---|---:|
| 위치 ID | `yongin_license_20260905` |
| 위도 | `37.288898139` deg |
| 경도 | `127.107505461` deg |
| WGS84 타원체고 | `114.4403` m |
| 설정 정확도 | `0.0200` m |
| 측량일 | 2026-09-05 |

NGII VRS RTK FIXED 표본 1,200개를 20분간 수집했다. 위도·경도·타원체고의
표준편차는 각각 0.5cm, 0.4cm, 1.0cm였다. F9P 플래시 저장 후
`TMODE=FIXED`, `fixType=5 (TIME)` 및 좌표 일치를 확인했다.

> 안테나는 측량 당시의 같은 위치와 높이에 설치해야 한다. 다른 위치에 설치해도 RTK
> FIXED는 표시될 수 있지만 차량 좌표 전체가 설치 오차만큼 이동한다. 설치 사진, 바닥
> 기준점과 삼각대 높이는 철수 전에 기록을 보완한다.

## 다른 장소에서 용인 좌표로 복원

현재 플래시 좌표를 먼저 읽고 `BASE_LOCATIONS.md`에 등록된 값인지 확인한다.

```bash
cd "$HOME/FMA_ws/src/stack_gps/tools/base_station"
python3 read_base_position.py --port /dev/ttyF9P --baud 115200
```

그다음 용인 좌표를 저장한다.

```bash
python3 setup_base.py \
  --lat 37.288898139 \
  --lon 127.107505461 \
  --height 114.4403
```

`fixType=5 (TIME — 베이스 정상)`이 나온 뒤 좌표를 다시 읽어 위 표와 정확히
일치하는지 확인한다. 등록된 용인 지점을 재사용할 때는 재측량하지 않는다.

## RTCM 송출

```bash
cd "$HOME/FMA_ws/src/stack_gps/tools/base_station"
ls -l /dev/ttyF9P /dev/ttyF9P_uart2 /dev/ttyRadio
python3 rtcm_server.py --radio /dev/ttyRadio
```

야외에서 10초 통계가 RTCM 수백 B/s이면 정상이다. `0 B/s`가 지속되면 차량 운용을
시작하지 않는다.

## 코스 주의

용인에서 사용할 웨이포인트는 이 베이스 좌표로 RTK FIXED를 만든 뒤 새로 기록한다.
한라대나 원주에서 기록한 CSV를 사용하면 안 된다. 새 CSV 파일명에도
`yongin`을 포함한다. 예: `waypoints_yongin_license_course_a_YYYYMMDD_HHMMSS.csv`.
