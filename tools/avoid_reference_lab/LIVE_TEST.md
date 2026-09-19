# Zone 없이 회피 테스트

## 실제 이동: 01 → 03을 1 m/s로 테스트

01의 종점과 03의 시작점은 동일하다. 원본 CSV를 01 → 03 순서로 사용하고 03 종점에서 종료한다.

```bash
# 라이다 링크·접근 권한이 없을 때, 센서/차량 launch를 종료한 상태에서 1회 실행
sudo /usr/bin/python3 scripts/v2_recover_lidars.py --apply

# 첫 번째 터미널: 차량을 해당 출발 위치에 놓고 기동 (GO 전 대기)
scripts/avoid-drive-test

# 두 번째 터미널: RViz
scripts/v2 view

# 센서·GPS·경로 방향 및 주변 통제/물리 비상정지 담당 준비 후 출발
scripts/v2 go --skip-lane

# 운영 정지
scripts/v2 stop
```

03의 기존 T자 주차 미션은 테스트 전용 zone 파일에서 제외했다. 원본 zone 파일은 수정하지 않았다.

- `avoid_zone_only=false`: zone 밖에서도 장애물 감지로 MGM 회피에 진입한다. 회피 상태를 강제로 고정하는 설정은 아니다.
- `fixed_goals`, native 계산, 목표속도 1 m/s. 차선 카메라 대신 실제 GPS 웨이포인트를 사용한다.
- 기존 MGM의 GO 대기, GPS/라이다 신선도, 레퍼런스 유효성, 독립 E-stop 검사를 적용한다. 자동 후진은 끈다.
- 네 라이다/주차 인지 구성을 재사용한다. 선택한 테스트 zone 파일에는 주차 미션이 없다.
- rosbag 기록을 켠다. 실제 세션 폴더는 실행 로그에 표시된다.
- 기본 `scripts/v2 drive`의 사용자 작업 중인 04번 고정 경로 선택을 통하지 않는다.

## 정차 상태: 실제 라이다 + 가상 직선 + RViz

```bash
scripts/avoid-lidar-lab --port /dev/lidar_front
```

생성된 직선을 GPS 인터페이스로 공급하고 `/avoid_lab/mgm_state`에 AVOID_ACTIVE를 주기적으로 발행한다. 실제 라이다 외의 위치·속도 입력은 정차 테스트 가정이다. 모든 회피 출력은 `/avoid_lab` 아래이며 CAN/MGM 차량 명령 노드는 실행하지 않는다.

이미 전방 스캔이 발행 중이면 `scripts/avoid-lidar-lab`만 실행한다. 실제 GPS 경로를 사용하려면 `--gps-topic /perception/gps_path`를 추가한다. 이 옵션은 기존 GPS 노드의 유효한 위치·방향·웨이포인트 메시지가 필요하다.

장애물 배치를 바꿨으면 고정 목표를 초기화한다.

```bash
source /opt/ros/humble/setup.bash
source install_v2/local_setup.bash
ros2 service call /avoid_lab/reset std_srvs/srv/Trigger '{}'
```

초록 곡선은 생성 경로, 초록 구는 출력 레퍼런스, 주황 점은 실제 전방 스캔, 파란 직선은 가상 기준 경로다. 화면의 WAITING FOR LIVE LIDAR는 센서 입력 대기 상태다.
