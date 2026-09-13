# 카메라–GPS 2 m/s 로깅 시험

준비한 실행 파일은 저장소 루트의 `scripts/run_lane_gps_2mps.sh`.
이 저장소의 한라대 코스 `waypoints_halla_univ_20260819_182657.csv`를 사용한다.
이 저장소 install을 source하며 별도 `~/FMA_ws` 체크아웃의 코드는 수정하지 않는다.

USB 없이 점검:
```bash
./scripts/run_lane_gps_2mps.sh --check
```

USB 연결 후 베이스 보정 송출을 켜고 차량 터미널에서 RTCM 중계:
```bash
python3 ~/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py --port /dev/ttyRadio --tcp-port 2101
```
다른 터미널에서 노드 기동(정지 대기, CAN 송신 시작):
```bash
./scripts/run_lane_gps_2mps.sh --start
```
출발은 별도 터미널에서 이 저장소의 환경을 source한 뒤:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run adas_mgm go
```
`go`의 RTK·차선·라이다·회피·CAN 확인을 통과해야 한다. 종료는 launch 터미널 Ctrl-C.
기존 dSPACE watchdog 미구현 기록 때문에 PC/USB 전체 고장은 종료 시 CAN zero로도
해결되지 않는다. 실제 비상정지 수단을 확보한 상태에서 시험한다.

시험 설정:
- 차선↔GPS 자동 전이, 일반/가속구간 목표 2.0 m/s(7.2 km/h).
- 회피 TTC 계산 속도도 2.0, 회피 속도 상한은 기존 0.6 m/s. 정지/감속 판단 우선.
- 후진 탈출 0, 주차 꺼짐. 신호등·정지선은 로깅용으로 켜고 정지 게이트는 0으로 둔다.
- USB2 HIGH / 카메라 2대 각각 10 fps. 성공한 모델 입력 프로필인 차선 640(XPU),
  신호등 800(CPU), 정지선 320(CPU)을 사용한다. 6.14 커널 재부팅 후
  `scripts/gpu/check_xpu.py` 검증 성공이 선행되어야 한다. CPU 폴백은
  `LANE_DEVICE=cpu ./scripts/run_lane_gps_2mps.sh --start`.
- 정적 정지 on/off 3.6/4.0 m, 감지 corridor 4.2 m.
  동적 정지 4.0 m, ROI 4.2 m, 추적 5.0 m, TTC 정지 2.0 s.
  기존 launch의 실측식 `0.303*v + 1.19*(0.13*v + v*v/(2*0.94))`를
  2 m/s로 외삽하면 약 3.45 m. 위 값은 여유를 더한 시험 초기값이며
  2 m/s 실제 제동 성능을 검증한 값은 아니다. 넓힌 ROI의 오정지도 확인해야 한다.
- 차선 수신 1초 중단 시 LANE에서 기존 watchdog 정지. 신뢰도 저하 시 GPS로
  전이하는 것과 USB/노드 수신 중단은 다르다. watchdog 완화나 자동 재시작은 추가하지 않았다.
  2 m/s에서 1초는 2 m이므로 연결 후 처리 주기/끊김을 확인하고 단계적으로 속도를 검증한다.

로그: `~/FMA_ws/drive_logs/run_<시각>/`의 rosbag, lane_frames.csv, lateral.csv,
vehicle_vector.csv, mgm_snapshots.bin, mgm_jitter.csv, transitions.csv.
CSV만 별도로 켜고 원시 디버그 영상은 꺼서 기록 부하를 줄인다.
rosbag의 차선 header stamp/수신 간격, /rosout의 파이프라인 지연·큐 폐기·watchdog,
transitions.csv를 대조하면 지연/신뢰도 저하/수신 중단을 구분할 수 있다.
영상도 필요한 별도 진단 run은 기존 launch에서 `lane_debug:=true`를 사용한다.
