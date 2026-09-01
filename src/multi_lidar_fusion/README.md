# multi_lidar_fusion

2D LiDAR **4대를 하나의 가상 360° LiDAR로 추상화**하는 ROS 2 Humble 패키지.

| 슬롯 | 위치 | 모델 | 포트 (launch 기본값) | 규칙이 무엇으로 가르나 | 실측 |
|---|---|---|---|---|---|
| `a1` | 전방 | YDLiDAR T-mini Plus | `/dev/lidar_front` | 허브 구멍 `ID_PATH …3.4` | 10.2 Hz, 0.839°, 12 m |
| `a2` | 후방 | YDLiDAR T-mini Plus | `/dev/lidar_rear` | 허브 구멍 `ID_PATH …3.3` | 10.1 Hz, 0.839°, 12 m |
| `b1` | 좌측 | SLAMTEC RPLiDAR C1M1 | `/dev/lidar_left` | 시리얼 `f2ee467b…` | 10 Hz, 0.499°, 16 m |
| `b2` | 우측 | SLAMTEC RPLiDAR C1M1 | `/dev/lidar_right` | 시리얼 `76d341fd…` | 10 Hz, 0.499°, 16 m |

심링크는 `tools/99-fma-lidars.rules` 가 만든다. **설치돼 있어야 launch 가 돈다:**

```bash
ls -l /dev/lidar_front /dev/lidar_rear /dev/lidar_left /dev/lidar_right
# 없으면
sudo cp tools/99-fma-lidars.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> 2026-08-27 에 기본값을 by-path 에서 심링크로 바꿨다. by-path 는 2026-08-25 udev
> 규칙을 넣은 시점에 이미 낡아 있었고(`…1.2.4/1.2.3` → 실제 `…3.4/3.3`),
> 값이 두 곳에 갈라져 있어서 생긴 일이다. 이제 슬롯의 단일 원천은 udev 규칙이다.
>
> ⚠ RPLiDAR 는 시리얼로 가르니 구멍을 옮겨도 따라가지만, **YDLiDAR 두 대는
> 시리얼이 둘 다 `0001` 이라 허브 구멍으로만 갈린다** — 옮겨 꽂으면 앞/뒤가
> 뒤바뀌므로 규칙의 `ID_PATH` 를 고쳐야 한다.

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

### 명령 하나로 (권장)

```bash
~/FMA_ws/src/multi_lidar_fusion/tools/run_4lidar.sh
```

터미널 하나면 되고 `source` 도 필요 없다. 드라이버 4대 → 스캔 수신 확인(4/4) → 융합 + RViz 순으로 띄우고, `Ctrl-C` 하나로 전부 내린다.

이 스크립트가 대신 처리해 주는 것 두 가지:

- **YD 포트 자동 탐지.** YD 2대는 by-id 가 겹쳐 by-path 를 쓰는데, 그 주소는 "허브의 그 구멍"이라 USB 를 옮기거나 허브 전원을 껐다 켜면 조용히 바뀐다(2026-08-14: `0:1.2.x` → `0:3.x` 로 바뀌어 YD 2대가 무발행이었다). 매번 CP2102 장치를 찾아 배정하고 무엇을 골랐는지 찍는다.
- **안전한 종료.** `SIGINT` 를 먼저 보내고 4초 기다린 뒤에야 강제 종료한다. `SIGKILL` 로 죽이면 드라이버가 라이다에 정지 명령을 못 보내, RPLiDAR 가 다음 기동에서 `SL_RESULT_OPERATION_TIMEOUT` / `Can not start scan` 으로 실패한다(실측).

```bash
run_4lidar.sh --no-rviz            # RViz 없이 (rosbag 기록·원격 접속)
run_4lidar.sh --build              # 빌드 후 실행
run_4lidar.sh --a1 /dev/... --a2 /dev/...   # 자동 탐지가 틀렸을 때
```

> 앞/뒤가 바뀐 것 같으면 `ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=yd0` 로 한 대씩 확인한다.

### 실 센서 없이 (시뮬)

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

YDLiDAR T-mini Plus와 RPLiDAR C1M1은 **둘 다 Silicon Labs CP210x** 를 쓴다. IMU까지 같은 칩이라 `/etc/udev/rules.d/99-ydlidar.rules` 가 `10c4:ea60` 전체를 `/dev/ydlidar` 로 묶어버린다. 2026-08-13 확인 시점에 `/dev/ydlidar` 와 `/dev/rplidar` 가 **둘 다 같은 장치**를 가리키고 있었다. 이 심볼릭 링크는 쓰지 않는다.

### YDLiDAR 2대는 by-id 로도 구분되지 않는다 (실측 확인)

T-mini Plus 유닛의 CP210x 시리얼이 **둘 다 `0001`** 이라 by-id 이름이 완전히 같아진다. 4대를 동시에 붙여 확인한 결과:

```
by-path 항목 6개   vs   by-id 항목 5개      ← YDLiDAR 하나가 by-id 주소를 잃음
usb-…CP2102_…_0001-if00-port0 -> ttyUSB5    ← 나중에 붙은 쪽이 먼저 것을 덮어씀
```

따라서:

| | 고정 방식 | 이유 |
|---|---|---|
| `a1`, `a2` (YDLiDAR) | **by-path** | 시리얼이 겹쳐 by-id 불가 |
| `b1`, `b2` (RPLiDAR) | **by-id** | 시리얼이 고유 |

> **by-path 는 "허브의 그 구멍"이 주소다.** 케이블을 다른 포트로 옮겨 꽂으면 앞/뒤가 조용히 뒤바뀐다. 옮겼으면 반드시 다시 확인할 것.

### 어느 유닛이 어디에 달렸는지 확인하는 법

```bash
# 한 대만 띄워 RViz 로 본다. unit = yd0 | yd1 | rp0 | rp1
ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=yd0
```

화면 정중앙이 그 라이다, 빨간 축(X)이 그 유닛의 0도 방향이다. 차량 앞쪽에서 손을 넣어 점이 반응하면 그게 앞 라이다다. 확인한 값을 `launch/multi_lidar_drivers.launch.py` 의 `DEFAULT_PORTS` 에 적는다.

포트 목록만 보려면 `ros2 run multi_lidar_fusion identify_lidars.sh`.

손 가림으로 자동 판정하는 도구도 있다(좌/우가 31cm 밖에 안 떨어져 있어 실패할 수 있음):

```bash
ros2 launch multi_lidar_fusion identify_positions.launch.py
```

### ⚠ 두 번째 함정 — RPLiDAR 가 `health OK` 인데 `/scan` 이 0 Hz (2026-08-30)

**드라이버로는 절대 안 풀린다. `RESET(0xA5 0x40)` 을 사람이 보내야 한다.**

`rplidar_node` 는 `STOP → GET_INFO → GET_HEALTH → SCAN` 만 보내고 **RESET 을 보내지
않는다.** 그래서 라이다가 모터 latch-off 상태로 굳으면 재기동·재연결·`/start_motor`
서비스 호출 중 무엇으로도 못 빠져나온다. 로그가 전부 정상으로 보이는 게 악질이다:

```
[rplidar_b1] RPLidar health status : OK.                      ← 정상
[rplidar_b1] current scan mode: Standard, 5 KHz, 16.0m, 10Hz  ← 정상
ros2 topic hz /lidar/b1/scan  → 발행 없음                      ← ★ 여기만 이상
```

시리얼로 직접 찔러 본 결과 (2026-08-30):

| 보낸 것 | 응답 |
|---|---|
| `GET_DEVICE_INFO` | 27B 정상 (모델 `0x41`, FW 1.02, HW 18) |
| `GET_HEALTH` | `Good(0)` |
| `SCAN` / `EXPRESS_SCAN` | **디스크립터 7바이트만, 측정 데이터 0** |
| DTR/RTS 6개 조합 · `SET_MOTOR_SPEED` | 변화 없음 |
| **`RESET`** | **부트 배너 + 1초에 4095B — 즉시 회복** |

```bash
python3 - <<'PY'
import serial, time
for d in ['/dev/lidar_left', '/dev/lidar_right']:
    s = serial.Serial(d, 460800, timeout=1)
    s.write(b'\xA5\x25'); time.sleep(0.3); s.reset_input_buffer()
    s.write(b'\xA5\x40'); time.sleep(2.5)          # RESET
    raw = s.read(s.in_waiting or 1)
    print(f"  {'OK' if b'LIDAR System' in raw else 'FAIL'} {d}"); s.close()
PY
```

한 번 풀리면 드라이버를 SIGTERM/SIGKILL 어느 쪽으로 죽여도 재발하지 않는다
(재기동 4회 연속 확인). 들어가는 원인은 전원으로 보인다 — 스캔 도중 전압이 끊기면
컨트롤러는 살아남고 모터만 latch-off 된다. **HANDOVER §3.7 "전류를 갈라라"와 같은
고장의 앞뒤다: 전류 분리는 재발을 막고, 이 RESET 은 이미 걸린 것을 푼다.**
4대 기동 절차에는 리셋을 먼저 넣어 두는 것이 안전하다.

> **위 스크립트의 `FAIL` 은 그 자체로 고장이 아니다.** 배너는 읽기 창을 놓치면
> 안 잡히고(이미 정상인 장치도 그렇다), 무엇보다 **다른 프로세스가 포트를 잡고 있으면
> 반드시 실패한다.** 판정의 최종 근거는 배너가 아니라 **`/scan` 이 도느냐**다:
>
> ```bash
> # 먼저 포트를 잡은 놈이 없는지 — 이게 FAIL 의 최빈 원인이다
> for d in /dev/lidar_left /dev/lidar_right; do
>   echo "$d: $(fuser $(readlink -f $d) 2>&1 >/dev/null || echo 비어있음)"; done
> # 그다음 실제 판정
> for t in a1 a2 b1 b2; do echo -n "$t: "; \
>   timeout 7 ros2 topic hz /lidar/$t/scan 2>&1 | grep -oE 'average rate: [0-9.]+' | head -1; done
> ```
>
> **launch 를 두 번 띄우면 포트마다 프로세스가 둘씩 붙어 서로를 죽인다.** 실제로
> 2026-08-30 검증 중에 이걸로 3대가 0Hz 가 됐다 — 리셋 문제로 오진하기 딱 좋다.
> 새로 띄우기 전에 위 `fuser` 로 항상 비었는지 확인할 것.

### ⚠ 부트 배너의 `RP S2` 는 모델명이 아니다

RESET 하면 `RP S2 LIDAR System.` 이 찍히는데 **펌웨어 플랫폼 배너일 뿐이고 C1M1 이 맞다.**
실측 제원이 전부 C1 이다:

```
모델 바이트 0x41 · FW 1.02 · HW 18 · 보드 460800
샘플레이트 5 kHz · 10 Hz · 물리 분해능 0.72° (511점/회전)
지원 모드  Standard(16.0 m) / DenseBoost(40.0 m)
```

S2 라면 `0.12° · 32 kHz · 1 Mbps` 여야 하므로 하나도 맞지 않는다.
**배너를 보고 모델을 바꿔 잡거나 보드레이트를 1 Mbps 로 올리지 말 것.**

> 표의 `0.499°` 와 여기 `0.72°` 가 다른 것은 정상이다. launch 가 `angle_compensate=True`
> 로 띄우므로 드라이버가 720 bin(0.499°)으로 보간해 내보내고, 0.72° 는 그 아래의
> 물리 샘플 간격이다. 융합 `scan.angle_increment` 하한(1.0°)의 근거는 **성긴 쪽인
> T-mini 0.839°** 이므로 어느 쪽으로도 바뀌지 않는다.

---

## 7. 설정

### 7.1 장착값의 단일 원천 — `stack_parking/config/lidar_mounts.yaml`

**장착 좌표와 시야각을 고칠 곳은 여기 한 곳뿐이다.** `multi_lidar_fusion.launch.py` 가 이 파일을 읽어 노드 파라미터로 주입하고, `config/lidar_extrinsics.yaml` 의 사본과 다르면 경고를 찍는다.

```
[multi_lidar_fusion] 장착값 원천: …/stack_parking/config/lidar_mounts.yaml (fov_status=geometric_upper_bound)
[multi_lidar_fusion] ! lidar_extrinsics.yaml 이 원천과 다르다 — 원천 값으로 덮어쓴다: a1.x: 0.76 -> 0.9
```

현재 값 (좌표 2026-08-11, 시야·yaw 2026-08-14):

| 슬롯 | mounts 키 | x | y | z | 시야 중심 | 시야 폭 | 사용 raw 구간 |
|---|---|---|---|---|---|---|---|
| `a1` | `front` | 0.760 | 0 | 0.065 | 0° | 180° | 357~177° |
| `a2` | `rear` | −0.055 | 0 | 0.065 | 180° | 140° | 20~160° |
| `b1` | `left` | 0.310 | +0.155 | 0.065 | +95° | 130° | 120~250° |
| `b2` | `right` | 0.310 | −0.155 | 0.065 | −100° | 120° | 113~233° |

> "사용 raw 구간"은 `fov_center_deg − yaw_deg` 로 launch 가 계산한 값이다. 손으로 적지 말고 launch 로그로 확인할 것.

슬롯↔키 대응은 `multi_lidar_fusion.launch.py` 의 `MOUNT_OF_SLOT` 한 곳에만 있다. `stack_parking` 이 없는 환경이면 조용히 `lidar_extrinsics.yaml` 사본만 쓴다. 원천을 무시하려면 `use_mounts:=false`.

**장착 yaw · 거리 보정 (2026-08-14 실차 확정)** — yaw 는 "그 라이다의 스캔 0도가 차량 기준 어느 쪽을 보는가"이지 센서를 설치한 방향이 아니다. 모델마다, 그리고 **드라이버 각도 옵션마다** 달라진다.

| 슬롯 | 모델 | `yaw_deg` | `range_offset_m` | 상태 |
|---|---|---|---|---|
| `a1` front | T-mini Plus | **−87.0°** | **+0.069** | 겹침 역산 확정 |
| `a2` rear | T-mini Plus | +90.0° | +0.069 | ⚠ yaw 미검증 (거리는 같은 모델이라 공유) |
| `b1` left | RPLiDAR C1M1 | **−89.8°** | **−0.016** | 겹침 역산 확정 |
| `b2` right | RPLiDAR C1M1 | **+87.4°** | **−0.016** | 겹침 역산 확정 |

**전제 — 드라이버 각도 옵션과 한 세트다. 하나라도 바뀌면 위 값 전부 무효:**

| 모델 | 설정 |
|---|---|
| YD T-mini Plus | `lidar_type=1`, `sample_rate=9`, `reversion=true`, `inverted=false` |
| RPLiDAR C1M1 | `inverted=true` |

**확정 방법** — `tools/pair_calibrate.py` 로 두 겹침 쌍(앞↔우, 앞↔좌)을 **동시에** 풀었다. 좌·우가 같은 모델이라 거리 바이어스가 같아야 한다는 구속이 절대 yaw 를 정한다. **한 쌍만으로는 불가능하다** — yaw 를 5° 돌리면 거리 오프셋이 4cm 움직여 서로 상쇄되기 때문이다(실측 확인).

검증: 캡처 7개에서 두 센서의 같은 평면 인식이 **거리 0.70cm / 각도 0.70°** 이내로 일치 (보정 전 7.66cm / 1.53°).

**하루를 태운 함정 3가지 — 같은 실수를 반복하지 않기 위해 남긴다:**

1. **`lidar_type=0`(TOF)로 띄우면 거리가 정확히 4배**로 나온다. T-mini Plus 는 삼각측량이라 `1` 이어야 한다. 판을 30cm 에 뒀는데 145cm 로 읽혀 "판이 안 보인다"로 나타났다.
2. **`inverted` 는 각도 부호 반전(거울상)이다.** 정면에 판을 두고 yaw 를 맞추면 정면에서는 상쇄되고 **좌우만 뒤집혀** 보인다 (`a_rep = Y − β`, `yaw_cfg = −Y` → `β_pipe = −β`). 센서끼리는 서로 일치해서 개별 검증으로는 안 잡히고, 병합 화면 전체를 봐야 드러난다. 좌우 교환 증상이 보이면 **포트보다 각도 부호를 먼저** 의심할 것.
3. **그 반전을 되돌릴 때 yaw 를 네 대 모두 뒤집으면 안 된다.** 부호 반전의 고정점이 raw 0 / raw 180 이므로, YD(정면 raw 270)만 부호 반전이 필요하고 RP(정면 raw 180)는 그대로 둬야 한다. 일괄 적용했다가 좌·우 점이 2개/6개만 남았다.

**검산 규칙**: 같은 모델끼리 **센서기준 시야중심이 같아야 한다.**

> ⚠ **뒤(a2) yaw 는 아직 검증 전이다.** 같은 모델이라 앞에서 유도하면 +93° 지만, 앞이 규약값(−90)과 3° 달랐던 만큼 유닛별 편차가 있다. 뒤↔좌 또는 뒤↔우 쌍으로 같은 절차를 밟을 것.
>
> ⚠ **거리 보정의 절대 배분은 근거가 약하다.** 상대차 8.44cm(YD − RP)만 확정값이고, 절대값은 판 실측 평균에 맞춘 것이다. 더 나은 기준이 생기면 **두 값을 같은 양만큼 함께** 옮길 것 — 상대차만 유지되면 4대 정합은 그대로다.

### 7.2 센서 정의 — `config/lidar_extrinsics.yaml`

토픽·메시지 타입·frame·신뢰 거리는 여기가 원천이다. 코드는 건드리지 않는다.

**보는 범위·방향**은 센서 좌표계 기준으로 자른다 (원천을 쓰면 launch 가 덮어쓴다):

```yaml
sensors:
  a1:
    fov_enabled: true
    fov_min_deg: -100.0     # min < max → 보통 구간 (앞 200도)
    fov_max_deg:  100.0
```

`min > max` 로 두면 ±180°를 가로지르는 구간이 된다. 예를 들어 `150 ~ -150` 은 뒤쪽 60°만 남긴다. 브래킷에 가린 각도는 `blind_sectors_deg: [min1, max1, min2, max2, ...]` 로 따로 지운다.

> **`blind_sectors_deg: []` 로 두지 말 것.** ROS 2 에서 빈 리스트는 타입이 없어, 그 YAML을 읽는 **모든 노드가 생성자에서 즉시 죽는다** (`InvalidParameterValueException: No parameter value set`). 안 쓸 때는 키를 통째로 지우거나 주석 처리한다. rclcpp가 파라미터 override를 변환하는 시점이라 코드로 막을 수 없다.

### 7.3 융합 동작 — `config/fusion_params.yaml`

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `fusion_rate_hz` | 20.0 | 융합 주기. 가장 빠른 센서 이상으로 |
| `sync.sync_tolerance_ms` | 60 | `\|t_i − t_ref\|` 허용치. 느린 센서는 원리적으로 자기 주기의 절반(10Hz→50ms)만큼 뒤처지므로 그보다 커야 한다 |
| `sync.max_cloud_age_ms` | 100 | 이보다 오래되면 그 주기에서 제외 |
| `sync.strict` | false | true면 허용치 초과 센서를 버림. false면 쓰되 경고 |
| `motion.enable_motion_compensation` | false | 이동 중 벽이 두 겹으로 보이면 true |
| `filter.roi_*` | −5..10, −5..5 | 관심 상자 |
| `filter.vehicle_length/width` | 0.85 / 0.62 | 자기반사 제거 상자. 원본은 `stack_avoid/config/params.yaml` (차체 x = −0.090~0.760 → 중심 0.335) |
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
