# 최근 통합 주행 라이다·정지 진단

대상: `drive_logs/v2_20260914_045222_615965`, 2026-09-14 KST 04:52:22.766–05:01:14.766.
원본 dump v26의 ABI/크기를 검증해 53,201틱을 디코딩하고 현재 동일 형식 core_replay로 재생했다.
실제 transitions.csv의 23개 전환과 재생 결과의 주요 상태/경로 소스/v_ref가 일치한다.

## 결론

1. 이 주행에서는 유효한 라이다 입력이 처음부터 끝까지 없었다. 4개 드라이버가 약 0.6초 만에 종료됐으며, 각 드라이버 로그에는 초기화 시 `Unknown error`가 기록됐다. launch의 `finished cleanly`는 프로세스 exit code 0이며 센서 정상 동작을 뜻하지 않는다.
2. 장시간 v_ref=0은 SAFE_STOP이 아니라 T자 주차 미션의 벽 스캔 수집 대기였다. `parking_wall_acquisition_complete`가 전 구간 false여서 출발하지 못했다.
3. 현재 USB 위치가 기존 udev 규칙과 달라 `/dev/lidar_front/rear/left/right` 링크가 모두 없다. 이는 드라이버 초기화 실패의 강한 원인 후보다. 당시 오류 로그 자체는 `Unknown error`여서 당시 장치 상태까지 직접 확정하지는 않는다.

## 주요 시간 구간

| 시간(KST) | 상태 및 속도 | 사유 |
|---|---|---|
| 04:52:22.766–04:52:47.306 | SAFE_STOP, v_ref=0 | 초기 입력/경로 준비, 출발 인가 대기 |
| 04:52:47.306–04:53:02.806 | NORMAL, v_ref=1 | 일반 주행 |
| 04:53:02.806–04:53:04.416 | SAFE_STOP, v_ref=0 | ROUTE_SEQUENCE(512), CSV 전환 |
| 04:53:04.416–04:53:42.006 | NORMAL, v_ref=1 | 일반 주행 |
| 04:53:42.006–05:01:14.766 | safety=NORMAL, mission=ACTIVE/T_PARKING, speed_owner=MISSION, v_ref=0 | 벽 수집 완료 대기, 약 7분 33초 |

실제 SAFE_STOP은 약 26.15초였다. 회피 상태는 전체 53,201틱에서 INACTIVE였고, T자 주차 시작 뒤 종료까지 route 03에 머물렀다.

## 입력 기록

- lidar_valid=0: 53,201 / 53,201틱.
- avoid_obstacle_detected=0, avoid_avoidable=0, avoid_path.n=0: 전 구간.
- auto_estop=0: 전 구간. 장애물이 없었다는 증거가 아니라 유효 스캔이 없었던 상태다.
- parking_wall_acquisition_complete=0, parking_preparation_ready=0: 전 구간.
- traffic_fail_safe_stop=0: 전 구간. 이번 정지 원인은 이전 세션의 카메라 오류와 다르다.
- mission_events: zone_entry, search_start만 존재. ready/handoff/done 없음.

## 근거 로그와 코드

- `log_v2/ros/2026-09-14-04-52-22-600498-sangmin-17Z90TL-H-AUB9U1-24715/launch.log`: 라이다 4개 기동 직후 종료.
- `log_v2/ros/ydlidar_ros2_driver_node_24731_1789329142684.log` 및 나머지 3개 드라이버 로그: Unknown error.
- `log_v2/ros/python3_24739_1789329143188.log`: `active sensors 0; unified output paused`.
- `log_v2/ros/python3_24749_1789329143115.log`: `SCAN_TIMEOUT` 반복.
- `log_v2/ros/python3_24743_1789329143225.log`: 주차 요청 때 `stage=slam map_points=0`.
- `src/adas_mgm/core/manager_step.cpp:554`: 주차 GPS 탐색 중 요청과 일치하는 벽 수집 완료가 없으면 v_ref=0, speed_owner=MISSION.
- `src/adas_mgm/src/mgm_node.cpp`: auto_estop은 estop_real && scan_valid로 입력된다. 라이다 스캔 무효는 실제 장애물 감지로 취급되지 않는다.
- `base_stop_reasons`: 일반 주행에서 라이다 단독 미수신은 별도 필수 정지 사유가 아니다. 따라서 카메라/GPS로 인가·주행됐지만 회피와 라이다 장애물 대응은 동작하지 못했다.

## 현재 장치 경로 차이

기존 `/etc/udev/rules.d/99-fma-lidars.rules`는 `usb-0:2.1`, `2.2`, `2.3`, `2.4.1` 위치를 사용한다.
현재 CP2102 4개 장치는 `usb-0:4.1`, `4.2`, `4.3`, `4.4.1`에 있다.
번호만으로 전후좌우를 지정하면 안 되므로 GET_DEVICE_INFO의 하드웨어 serial로 재확인해야 한다.

사용자가 실행할 복구 명령 (이 분석에서는 실행하지 않음):

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
sudo /usr/bin/python3 scripts/v2_recover_lidars.py --apply
```

4개 identity/링크/권한 검증 후 통합 런처를 다시 실행해 4개 scan 수신을 확인해야 한다.
GPS/RTCM 서비스나 주행 코드는 이번 분석에서 변경하지 않았다.
