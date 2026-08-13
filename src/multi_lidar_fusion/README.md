# multi_lidar_fusion

2D LiDAR **4대를 하나의 가상 360° LiDAR로 추상화**하는 ROS 2 Humble 패키지.

| 슬롯 | 위치 | 모델 |
|---|---|---|
| `a1` | 전방 | YDLiDAR T-mini Plus |
| `a2` | 후방 | YDLiDAR T-mini Plus |
| `b1` | 좌측 | SLAMTEC RPLiDAR C1M1 |
| `b2` | 우측 | SLAMTEC RPLiDAR C1M1 |

회피 로직(`stack_avoid`)은 라이다가 4대라는 사실을 알 필요가 없다. **`/lidar/merged_scan` 하나만 구독**하면 된다.

---

## 1. 아키텍처

```
[드라이버]              [코어 라이브러리 — ROS 의존 없음]                  [출력]

a1 ─scan─┐
a2 ─scan─┤   ① LidarConverter      LaserScan/PointCloud2 → CloudFrame
b1 ─scan─┤      + 센서별 FOV/blind, min/max_range, NaN·Inf 제거
b2 ─scan─┘            │
                      ├──────────────────────────► /lidar/{a1,a2,b1,b2}/cloud
                      ▼
             ② CloudTransformer     P_base = T_base_lidar · P_lidar  (tf2)
                      ▼
             ③ CloudSynchronizer    t_ref 선정 → 센서별 최근접 프레임
                                    age·tolerance 판정, 빠진 센서는 건너뜀
                      ▼
             ④ MotionCompensator    점별 dt 까지 t_ref 로 되돌림 (ON/OFF)
                      ▼
             ⑤ CloudMerger          concat + PointCloud2 직렬화(1회)
                      ▼
             ⑥ CloudFilter          range → ROI → self → voxel
                      ├──────────────────────────► /lidar/merged_cloud
                      ▼                            /lidar/merged_cloud_debug
             ⑦ VirtualLaserScan     bin 당 최소 거리 (§16), 미관측 = +inf
                      ├──────────────────────────► /lidar/merged_scan  ★
                      ▼
             ⑧ Diagnostics          FPS·dt·실패 카운터 → /diagnostics
```

설계상 지키는 것:

- **파이프라인 전체가 `CloudFrame` 하나만 주고받는다.** 콜백에서 한 번 정규화하고 발행 직전에 한 번만 `PointCloud2`로 되돌린다. `PointCloud2 → PCL → PointCloud2` 왕복 없음.
- **센서 모델 의존은 ① 단계에만.** 모델이 바뀌면 YAML(과 필요시 드라이버 launch)만 고친다. 융합 코어는 손대지 않는다.
- **라이다 하나가 죽어도 노드는 죽지 않는다.** 그 주기에서만 빠지고, 진단이 사유를 남긴다.
- **extrinsic·FOV 하드코딩 없음.** 전부 `config/lidar_extrinsics.yaml`.

### 파일 구성

| 파일 | 역할 |
|---|---|
| `include/.../types.hpp` | `CloudFrame`, `FusionPoint`, `SensorConfig`, `AngularSector` — 전 단계 공용 계약 |
| `lidar_converter.{hpp,cpp}` | ① 메시지 정규화 + 센서별 FOV/거리/유효성 |
| `cloud_transformer.{hpp,cpp}` | ② tf2 조회 → base_link |
| `cloud_synchronizer.{hpp,cpp}` | ③ stamp 기반 프레임 선택 |
| `motion_compensator.{hpp,cpp}` | ④ 자차 운동 보상 |
| `cloud_merger.{hpp,cpp}` | ⑤ 병합 + `PointCloud2` 직렬화 |
| `cloud_filter.{hpp,cpp}` | ⑥ range/ROI/self/voxel |
| `virtual_laserscan.{hpp,cpp}` | ⑦ 가상 `LaserScan` |
| `diagnostics.{hpp,cpp}` | ⑧ 진단 집계 |
| `multi_lidar_fusion_node.cpp` | ROS wrapper — 배선만, 알고리즘 없음 |
| `test_scan_publisher.cpp` | 합성 라이다 4대 시뮬레이터 |

---

## 2. TF 구조

4대 모두 `base_link` 직속이다. 라이다끼리 연쇄로 매달지 않는다.

```
base_link
 ├── lidar_a1_link
 ├── lidar_a2_link
 ├── lidar_b1_link
 └── lidar_b2_link
```

기본적으로 이 노드가 `publish_static_tf: true` 로 static TF를 직접 낸다. URDF/`robot_state_publisher` 로 이미 내보내고 있다면 `false` 로 끈다.

좌표 규약(REP-103): x = 전방, y = 좌측, z = 위, yaw 반시계 양수.

---

## 3. 토픽

**입력**

| 토픽 | 타입 | 비고 |
|---|---|---|
| `/lidar/{a1,a2,b1,b2}/scan` | `sensor_msgs/LaserScan` | `sensors.<id>.input_type: "cloud"` 로 바꾸면 `PointCloud2` 입력도 그대로 처리 |
| `/odom` 또는 `/vehicle/twist` | `nav_msgs/Odometry`, `geometry_msgs/TwistStamped` | motion compensation용 (선택) |

**출력**

| 토픽 | 타입 | frame |
|---|---|---|
| `/lidar/{a1,a2,b1,b2}/cloud` | `PointCloud2` | `base_link` (`sensor_cloud_frame` 로 전환) |
| `/lidar/merged_cloud` | `PointCloud2` | `base_link` |
| `/lidar/merged_cloud_debug` | `PointCloud2` (+`sensor_id`) | `base_link` |
| **`/lidar/merged_scan`** | **`LaserScan`** | **`base_link`** ← 회피 로직이 볼 유일한 토픽 |
| `/diagnostics` | `DiagnosticArray` | — |

**QoS**: 입출력 모두 기본 `best_effort` depth 5 (`SensorDataQoS` 호환). `stack_avoid` 가 `qos_profile_sensor_data` 를 쓰므로 그대로 맞는다.

> `ros2 topic hz /lidar/merged_scan` 이 아무것도 못 받으면 QoS mismatch다. `ros2 topic hz /lidar/merged_scan --qos-reliability best_effort` 로 볼 것.

---

## 4. 빌드

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select multi_lidar_fusion
source install/setup.bash
```

의존: `rclcpp`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `diagnostic_msgs`, `tf2`, `tf2_ros`. **PCL 의존 없음** (voxel 필터는 직접 구현 — `pcl_ros` 미설치 환경에서도 빌드된다).

단위 테스트:

```bash
colcon test --packages-select multi_lidar_fusion
colcon test-result --all --verbose
```

---

## 5. 실행

### 실 센서 없이 (권장 첫 단계)

```bash
ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py sim:=true rviz:=true
```

합성 라이다 4대(A=10Hz, B=20Hz)가 방 + 장애물을 스캔한다. 파이프라인 전 단계가 그대로 돈다.

### 실 센서

```bash
# 터미널 1 — 드라이버 4대
ros2 launch multi_lidar_fusion multi_lidar_drivers.launch.py

# 터미널 2 — 융합
ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py rviz:=true
```

드라이버와 융합을 분리한 이유: 드라이버가 죽어도 융합 노드는 살아 있어야 하고, rosbag 재생 때는 융합만 돌려야 한다.

### 주요 인자

```bash
ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py \
    extrinsics_file:=/path/to/my_extrinsics.yaml \
    params_file:=/path/to/my_params.yaml \
    sim:=true rviz:=true log_level:=debug
```

---

## 6. ⚠ 시리얼 포트 — 4대 구성의 첫 번째 함정

YDLiDAR T-mini Plus와 RPLiDAR C1M1은 **둘 다 Silicon Labs CP210x** 를 쓴다. 이 PC의 `/etc/udev/rules.d/99-ydlidar.rules` 는 `10c4:ea60` 전체를 `/dev/ydlidar` 로 묶어버린다 — IMU까지 같은 칩이라, 2026-08-13 확인 시점에 `/dev/ydlidar` 는 **실제로 IMU를 가리키고 있었다**.

`/dev/ttyUSB*` 도 재연결마다 번호가 바뀐다. **반드시 `/dev/serial/by-id/` 를 쓴다** (칩 시리얼이 들어가 개체마다 유일).

```bash
ros2 run multi_lidar_fusion identify_lidars.sh          # 후보 목록
ros2 run multi_lidar_fusion identify_lidars.sh probe    # 한 대씩 띄워 확인하는 절차
```

확인한 값을 `launch/multi_lidar_drivers.launch.py` 의 `DEFAULT_PORTS` 에 적어두거나 인자로 넘긴다:

```bash
ros2 launch multi_lidar_fusion multi_lidar_drivers.launch.py \
    a1_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_..._0001-if00-port0
```

---

## 7. 설정

### 7.1 장착 위치와 보는 범위 — `config/lidar_extrinsics.yaml`

가장 자주 만지는 파일. 코드는 건드리지 않는다.

```yaml
extrinsics:
  a1: {x: 0.30, y: 0.00, z: 0.20, roll: 0.0, pitch: 0.0, yaw: 0.0}
```

**보는 범위·방향**은 센서 좌표계 기준으로 자른다:

```yaml
sensors:
  a1:
    fov_enabled: true
    fov_min_deg: -100.0     # min < max → 보통 구간 (앞 200도)
    fov_max_deg:  100.0
```

`min > max` 로 두면 ±180°를 가로지르는 구간이 된다. 예를 들어 `150 ~ -150` 은 뒤쪽 60°만 남긴다. 브래킷에 가린 각도는 `blind_sectors_deg: [min1, max1, min2, max2, ...]` 로 따로 지운다.

> **`blind_sectors_deg: []` 로 두지 말 것.** ROS 2 에서 빈 리스트는 타입이 없어, 그 YAML을 읽는 **모든 노드가 생성자에서 즉시 죽는다** (`InvalidParameterValueException: No parameter value set`). 안 쓸 때는 키를 통째로 지우거나 주석 처리한다. rclcpp가 파라미터 override를 변환하는 시점이라 코드로 막을 수 없다.

### 7.2 융합 동작 — `config/fusion_params.yaml`

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `fusion_rate_hz` | 20.0 | 융합 주기. 가장 빠른 센서 이상으로 |
| `sync.sync_tolerance_ms` | 60 | `\|t_i − t_ref\|` 허용치. 느린 센서는 원리적으로 자기 주기의 절반(10Hz→50ms)만큼 뒤처지므로 그보다 커야 한다 |
| `sync.max_cloud_age_ms` | 100 | 이보다 오래되면 그 주기에서 제외 |
| `sync.strict` | false | true면 허용치 초과 센서를 버림. false면 쓰되 경고 |
| `motion.enable_motion_compensation` | false | 이동 중 벽이 두 겹으로 보이면 true |
| `filter.roi_*` | −5..10, −5..5 | 관심 상자 |
| `filter.vehicle_length/width` | 0.6 / 0.4 | 자기반사 제거 상자 — **실측값으로 교체** |
| `filter.enable_voxel_filter` | false | 겹침이 심할 때 켬 |
| `scan.angle_increment` | 0.0174533 (1.0°) | 아래 참조 |

**`scan.angle_increment` 를 원본 센서보다 촘촘하게 잡지 말 것.** T-mini Plus ~0.9°, RPLiDAR C1 ~0.72° 인데 0.5°(720 bin)로 두면 점이 닿지 않는 bin이 생겨 `inf` 로 남는다. 회피 로직은 그것을 "비어 있음"으로 읽으므로 위험하다.

시뮬 실측 (2026-08-13):

| `angle_increment` | bin 수 | 미관측 bin |
|---|---|---|
| 0.5° | 720 | 23 |
| 1.0° | 360 | **0** |

---

## 8. RViz 검증

Fixed Frame = `base_link`. `rviz/multi_lidar.rviz` 에 아래가 이미 들어 있다.

| 표시 | 색 |
|---|---|
| `/lidar/a1/cloud` | 빨강 (전방) |
| `/lidar/a2/cloud` | 노랑 (후방) |
| `/lidar/b1/cloud` | 청록 (좌) |
| `/lidar/b2/cloud` | 초록 (우) |
| `/lidar/merged_cloud` | 흰색 |
| `/lidar/merged_scan` | 자홍 |

### 검증 ① 정지 상태 — extrinsic

차량 주변에 평평한 벽을 하나 두고, 두 대 이상이 같은 벽을 볼 수 있게 세운다.

- **정상**: 색이 다른 점들이 같은 선 위에 겹친다.
- **벽이 여러 겹으로 보인다** → extrinsic 오차. 어긋난 방향으로 해당 센서의 `x/y/yaw` 를 고친다. 평행 이동이면 `x/y`, 회전하며 벌어지면 `yaw`.

### 검증 ② 이동 상태 — 시간 정렬

차량을 밀면서 같은 물체를 본다.

- **물체가 길게 늘어나거나 두 겹으로 보인다** → 먼저 `/diagnostics` 의 `max_stamp_spread_ms` 를 본다.
  - spread가 크면 `sync.sync_tolerance_ms` 를 줄이거나 `fusion_rate_hz` 를 올린다.
  - spread가 작은데도 늘어나면 motion compensation을 켠다:
    ```yaml
    motion:
      enable_motion_compensation: true
      twist_source: "odom"     # 또는 "twist"
    ```

시뮬로 재현하려면 `config/sim_lidars.yaml` 의 `vehicle.vx` 를 올린다 (시뮬이 `/odom` 과 `odom→base_link` TF를 함께 낸다).

---

## 9. 진단 읽는 법

2초마다 한 줄 (콜백마다 찍지 않는다):

```
active=4/4 cycles=41 pub=41 merged=1724pt out=1724pt spread=50.7ms cover=100%
 | a1 10.2Hz dt=-0.2ms used   | a2 10.2Hz dt=-0.1ms used
 | b1 20.0Hz dt=-0.1ms used   | b2 20.0Hz dt=+0.0ms used
```

| 항목 | 의미 |
|---|---|
| `active=n/N` | 이번 주기에 기여한 센서 수 |
| `spread` | 기여 센서 stamp의 최대−최소. 이게 크면 시간 정렬을 의심 |
| `cover` | `merged_scan` 에서 값이 들어간 bin 비율 |
| `dt` | 그 센서 프레임이 `t_ref` 대비 얼마나 앞/뒤인가 |
| 상태 | `used` / `reused`(느린 센서가 같은 프레임 재사용) / `too_old` / `out_of_sync` / `tf_failed` / `never_received` / `empty` / `disabled` |

로그 레벨은 자동으로 올라간다: 활성 센서 < `min_active_sensors` → WARN, 0 → **ERROR**.

`/diagnostics` 에는 센서별 누적 카운터까지 들어간다:

```bash
ros2 topic echo /diagnostics
```

---

## 10. 실패 모드 (요구 §20)

| 상황 | 동작 |
|---|---|
| 라이다 1대 연결 끊김 | 나머지로 계속 융합, 해당 센서 `too_old`, WARN |
| TF 없음 | 그 cloud만 제외, throttled WARN, 다음 주기 재시도 |
| 오래된 데이터 | `max_cloud_age_ms` 초과분 제외 |
| NaN / Inf | 정규화 단계에서 점 단위 제거 |
| 빈 cloud | 조용히 무시 (`empty`) |
| **전 센서 두절** | merged 발행 중단, `active=0` + diagnostic **ERROR**. 노드는 죽지 않는다 |

검증됨 (2026-08-13, 시뮬):

```
[ERROR] active=0/4 cycles=40 pub=0 ... | a1 0.0Hz too_old | ... | EMPTY_CYCLES=40
[WARN]  기여 센서 0 — merged 출력 없음. 드라이버/TF/max_cloud_age 확인 필요.
```

---

## 11. 단계별 진행 (요구 §26)

| Phase | 내용 | 성공 조건 | 상태 |
|---|---|---|---|
| 1 | 라이다 1대 → PointCloud2 | RViz에 점군 | ✅ 코어 완료 |
| 2 | 2대 → base_link | 같은 벽이 겹침 | ✅ 코드 완료 / **실차 캘리브레이션 필요** |
| 3 | 4대 concat | `/lidar/merged_cloud` | ✅ 시뮬 검증 |
| 4 | timestamp 동기화 | 다른 주기 안정 병합 | ✅ 시뮬 검증 (10Hz+20Hz) |
| 5 | ROI/range/invalid 필터 | — | ✅ 시뮬 검증 |
| 6 | 가상 LaserScan | `/lidar/merged_scan` | ✅ 시뮬 검증 |
| 7 | 이동 테스트 + motion comp | — | ⬜ **실차** |
| 8 | disconnect/TF error | crash 없음 | ✅ 시뮬 검증 (전 센서 두절) |

**남은 실차 작업**: ① 4대 시리얼 포트 확정, ② extrinsic 실측, ③ 각 라이다 FOV 확정, ④ `vehicle_length/width` 실측, ⑤ 이동 중 motion compensation 판정.

---

## 12. rosbag 테스트

```bash
ros2 bag record /lidar/a1/scan /lidar/a2/scan /lidar/b1/scan /lidar/b2/scan \
                /tf /tf_static /odom

# 재생하며 파라미터를 바꿔가며 튜닝 (드라이버 없이 융합만)
ros2 bag play <bag>
ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py
```

`publish_static_tf` 는 bag에 `/tf_static` 이 들어 있으면 `false` 로 두는 편이 낫다 (중복 발행 방지).

---

## 13. 회피 로직 쪽 인터페이스

```python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

self.create_subscription(LaserScan, '/lidar/merged_scan',
                         self.on_scan, qos_profile_sensor_data)
```

360° 한 대짜리 라이다와 완전히 동일하게 다루면 된다. `ranges[i]` 의 각도는 `angle_min + i * angle_increment`, 미관측은 `+inf`.

미관측(`inf`)과 "최대거리 밖"을 구분해야 하는 시점이 오면 coverage mask를 별도 토픽으로 뺄 수 있게 `VirtualLaserScan::observed()` 에 bin별 관측 여부가 이미 계산돼 있다.
