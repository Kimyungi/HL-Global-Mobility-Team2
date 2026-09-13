# v2 정면 라이다 장애물 오인 수정 — 2026-09-13

현장 증상은 RViz의 전방이 비어 있는데 E-stop이 지속되고, E-stop 제외 시에도
회피 인지가 장애물을 보고하는 것이었다. 통합 런처의 좌표 불일치를 합성 스캔으로 재현했다.
해당 현장 rosbag 원본은 현재 워크스페이스에 없어 실제 센서 기록 재생은 수행하지 않았다.

`parking_enabled=true`에서 `/lidar/a1/scan`은 v2 드라이버의 `reversion=true` 원본이다.
`lidar_fusion_v2/config/fixed_geometry.yaml`의 a1 yaw는 -87도인데,
E-stop은 단일 라이다용 +90도, 회피는 원본 전방 270도를 적용하고 있었다.
RViz의 보정 점군과 두 인지 노드 사이에 177도 차이가 있었다.

| 입력 | 수정 전 | 수정 후 |
|---|---|---|
| a1 원본 -93도, 거리 0.6m의 5점 군집(차량 뒤쪽) | 전방 약 0.598m 장애물로 오인 | E-stop 전방 ROI·회피에서 제외 |
| a1 원본 +87도, 거리 0.6m의 5점 군집(실제 전방) | 전방 장애물을 놓침 | E-stop 정지·회피 장애물 검출 |
| 실제 전방 기준 좌/우 15도 | 전방 장애물을 놓침 | 검출 및 좌우 부호 보존 |

공유 통합 런처는 a1 yaw를 위 보정 YAML에서 읽어 E-stop에 전달한다.
회피의 전방 각도는 같은 yaw의 부호를 반전해 도 단위로 변환한다.
따라서 일반 v2와 no-estop 런처 모두 회피 좌표가 맞는다.
`laser_yaw_in_base_rad`를 명시하면 4-LiDAR 모드의 두 소비자에 함께 적용된다.
단일 라이다 `parking_enabled=false`는 기존 /scan 방향 설정을 사용한다.

거리 임계값은 앞 라이다 원점 기준으로 유지한다. 통합 스캔으로 입력을 바꿔
후축 원점 거리와 섞거나, E-stop·회피 기능을 끄는 수정은 아니다.
보정 YAML의 거리 오프셋은 종전처럼 fusion에만 적용되므로 원본 소비자의 거리와는 차이가 있다.

검증은 `scripts/test_v2_front_lidar.py`에서 실제 런처 파라미터를 평가하고
E-stop/회피의 실제 검출 함수를 호출한다. 하드웨어 노드나 CAN을 실행하지 않는다.
기존 E-stop·회피·라이다 테스트와 함께 실행할 수 있다.

로컬 검증 결과: 수정 전 신규 시험은 좌표/검출 관련 6개 실패·기존 동작 2개 통과였고,
수정 후 통합 런처·E-stop·회피·라이다 시험을 합쳐 **83개 통과**했다.
`scripts/v2 build`는 13개 패키지 성공, `scripts/v2 check`도 통과했다.
이번 결과는 전체 MGM CTest나 실차 주행 통과를 뜻하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source install_v2/local_setup.bash
export PYTHONPATH="$PWD/src/stack_estop:$PWD/src/stack_avoid:$PWD/src/lidar_fusion_v2:$PYTHONPATH"
python3 -m pytest -q scripts/test_v2_front_lidar.py scripts/test_v2_launch.py src/stack_estop/test src/stack_avoid/test src/lidar_fusion_v2/test
```

실차 적용에는 `scripts/v2 build` 후 런처 재시작이 필요하다.
4-LiDAR 구성의 시작값은 E-stop yaw 약 `-1.518436449` rad,
회피 `lidar_mount.forward_angle_deg=87.0`이다. 과거에 명시한 +90도 yaw 인자는 제거해야 한다.
실제 차량에서 빈 전방의 오인 해제와 전방 장애물 정지 확인은 별도 현장 검증으로 남는다.
