# stack_avoid — 측정·설정값 정리 (M3: 정적 장애물 회피)

> 실측값의 단일 소스는 `config/params.yaml`. 이 문서는 각 값의 **측정 방법**을 설명한다.
> 결정 사항: 제원=직접 실측 / 원점=후축 중심(파라미터로 변경 가능) / LiDAR=수평·정면 정렬 / 시나리오=정적 장애물.

## A. LiDAR 장착 (원점→LiDAR, 수평·정면 정렬 → x·y·z만 측정)

수평·정면 정렬이므로 yaw/roll/pitch = 0 고정. launch TF도 이에 맞춰 갱신 필요
(`~/ydlidar_ros2_ws/.../launch/ydlidar_launch.py:49` 의 `base_link→laser_frame`).

| # | 값 | 단위 | 측정 방법 |
|---|---|---|---|
| A1 | lidar_mount.x_m | m | 후축 중심(좌우 뒷바퀴 축의 중앙)에서 LiDAR 원점까지 **전방 수평거리**를 줄자로 측정 |
| A2 | lidar_mount.y_m | m | 차량 중심선에서 LiDAR의 **좌우 편차**. 중앙 정렬이면 0 |
| A3 | lidar_mount.z_m | m | 지면에서 LiDAR **스캔 평면 높이**를 줄자로 측정 |

## B. 차량 제원 (직접 실측)

| # | 값 | 단위 | 측정 방법 |
|---|---|---|---|
| B1 | vehicle.width_m | m | 줄자로 **타이어 외측 폭**(가장 넓은 부분). 사이드미러 포함 시 `mirror_included: true` |
| B2 | vehicle.length_m | m | 앞범퍼~뒷범퍼 전장 |
| B3 | vehicle.wheelbase_m | m | **앞바퀴 중심~뒷바퀴 중심**(같은 쪽) 거리 |
| B4 | vehicle.front_overhang_m | m | 앞범퍼~앞축 중심 |
| B5 | vehicle.max_steer_deg | deg | full lock 시 **앞바퀴 각도**(각도앱/각도기) 또는 조향 서보 스펙 |
| B6 | vehicle.min_turn_radius_m | m | full lock 원 주행 후 반경 측정, **또는** `wheelbase / tan(max_steer)` 로 계산 |

## C. 회피 판단 파라미터 (정적 장애물 — 초기값 설정 후 주행 튜닝)

물리 측정이 아닌 설계·튜닝값. B1과 시연 속도가 정해지면 파생 계산 가능.

| # | 값 | 단위 | 산정 방법 |
|---|---|---|---|
| C1 | avoid.roi_angle_deg | deg | 전방 관심영역 ±각도. 기본 60° 제안 |
| C2 | avoid.max_range_m | m | 기본 12.0 (센서 한계) |
| C3 | avoid.ttc_stop_s | s | 즉시정지 임계. 제동거리+반응 고려, 초기 제안값 논의 |
| C4 | avoid.lateral_margin_m | m | 측방 안전여유. 측방여유판정 = width/2 + margin, 통과최소폭 = width + 2·margin |
| C5 | avoid.target_speed_mps | m/s | **M3 시연 주행 속도** (TTC 계산 기준) — 실측/목표값 |

## D. LiDAR 스캔 특성 (✅ 실측 완료 — YDLIDAR T-mini Pro)

| 항목 | 값 |
|---|---|
| 스캔 주파수 | 10.07 Hz (지터 std ~0.4ms) |
| 각도 해상도 | 0.839° |
| 각도 범위 | ±180° |
| 거리 범위 | 0.03 ~ 12.0 m |
| 포인트/스캔 | 약 429점 |

## E. 실행·검증 커맨드

```bash
source /opt/ros/humble/setup.bash
source ~/ydlidar_ros2_ws/install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
  params_file:=$HOME/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
# 검증: ros2 topic hz /scan  /  ros2 topic echo /scan --once  /  rviz2
```
