# OAK-D 신호등·정지선 정지 실험

현재 실험 경로는 아래 하나다.

```text
OAK-D RGB/depth
  -> stack_traffic (신호등 + 정지선 + 거리)
  -> /perception/traffic_stop
  -> adas_mgm (/adas/target_ref의 v_ref 결정)
  -> bridge_dspace
  -> CAN 0x101~0x114(points) -> 0x100(header)
  -> dSPACE
```

`stack_traffic`은 별도의 `/perception/stopline`을 받지 않는다. 정지선과 그 주변
노면 depth를 직접 측정한다. 색상 판정은 HSV만 사용하며 ONNX 분류 경로는 제거했다.

## 0. 최초 한 번 빌드

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
python3 -m pip install -r ~/FMA_ws/src/stack_traffic/requirements.txt
colcon build --symlink-install --packages-select \
  fma_interfaces stack_traffic adas_mgm bridge_dspace stack_estop
source ~/FMA_ws/install/setup.bash
```

OAK-D를 연결한 뒤 각 장치의 MxID를 확인하고 교통용 카메라의 값을 기록한다.

```bash
python3 -c 'import depthai as dai; [print(x.getMxId(), x.name, x.state) for x in dai.Device.getAllConnectedDevices()]'
```

OAK-D가 두 대인 차량에서는 `stack_lane`과 `stack_traffic`에 서로 다른 MxID를
반드시 지정한다. `stack_traffic`은 장치가 한 대일 때만 빈 MxID 자동 선택을 허용하고,
두 대 이상인데 `oak_mxid`가 비어 있으면 잘못된 카메라 사용을 막기 위해 종료한다.
장치가 일시적으로 보이지 않거나 재부팅 중이면 2초 간격으로 최대 3회 다시 열고,
그 뒤에도 0대이거나 열 수 없으면 무핀 자동 선택 없이 종료한다. 현재 차량의 교통용
MxID는 launch 기본값을 단일 기준으로 사용한다. 다른 장치로 시험할 때만 launch
명령에 `oak_mxid:=확인한_MxID`를 추가한다.

빌드 후에는 각 새 터미널에서 아래 세 줄을 먼저 실행한다.

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
source ~/FMA_ws/install/setup.bash
```

## 1. 노트북 단독 측정 — CAN 없음

이 단계에서는 차량, CAN, MGM, E-stop 노드를 실행하지 않는다. 아래 launch의 기본
두 정지 임계값의 기본값이 `0`이므로 측정만 하고 `stop_required`를 만들지 않는다.

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
source ~/FMA_ws/install/setup.bash

ros2 launch stack_traffic stopline_distance_test.launch.py \
  show_debug:=true
```

정상 표본은 로그가 다음 조건을 모두 만족해야 한다.

```text
stopline=1  stable=1  y_med=유효값  line_z=유효값  z_med=유효값
```

`line_near=0`, `FINAL_STOP=0`인 것이 정상이다. `line_z`와 `z_med`는 카메라
optical-Z이며 앞 범퍼 기준 절대거리가 아니다.

### 기록할 값

카메라를 실제 장착 각도로 고정하고 아래 두 위치에서 각각 10개 이상의 정상 표본을
기록한다.

| 위치 | `y_raw` [px] | `y_med` [0~1.10] | `line_z` [m] | `z_med` [m] |
|---|---:|---:|---:|---:|
| 원하는 최종 정지 위치 |  |  |  |  |
| 그 위치보다 약 1m 뒤 |  |  |  |  |

- 가까워질수록 일반적으로 `y_med`는 커지고 `z_med`는 작아져야 한다.
- 현재 기본 정지 기준은 고정 카메라에서 더 잘 구분되는 `y_med`다. `z_med`는 두 위치가
  반복해서 확실히 구분될 때만 보조 gate로 사용한다.
- 원하는 **최종 정차 위치** 값이 아니라, 인식·MGM·차량 제동 지연을 감안한
  **정지 요청 시작 위치**의 `y_med`를 첫 임계값으로 사용한다.

측정을 마치면 이 터미널을 `Ctrl-C`로 종료한다. OAK-D를 동시에 여는
`stack_traffic` 프로세스는 하나만 실행한다.

## 2. 실제 차량 시험 전 확인

- 첫 시험은 바퀴를 띄운 상태에서 수행한다.
- 지상 시험에는 물리 비상정지를 누를 담당자를 둔다.
- dSPACE는 CAN `0x100` counter가 30ms 동안 갱신되지 않으면 `v_ref=0`으로 만드는
  watchdog이 실제 모델에 들어 있어야 한다.
- 임시 경로와 실제 `stack_lane`을 동시에 실행하지 않는다.
- `dummy_ref_publisher`와 MGM을 동시에 실행하지 않는다.
- 실제 `stack_estop`과 임시 `estop:false` publisher를 동시에 실행하지 않는다.

## 3. 실제 차량 — 터미널별 실행

### 터미널 1 — CAN과 bridge

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
source ~/FMA_ws/install/setup.bash

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0

ros2 launch bridge_dspace bridge.launch.py can_interface:=can0
```

`bitrate 1000000`, `ERROR-ACTIVE`를 확인한다. bridge 로그는 약 5초마다 TX/RX
cycle 수를 출력한다.

### 터미널 2 — 경로 입력

실제 `stack_lane`이 정상 동작 중이면 이 임시 publisher는 실행하지 않는다. 신호등
직선 저속 시험에서만 다음을 사용한다.

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
source ~/FMA_ws/install/setup.bash

python3 ~/FMA_ws/scripts/test_mgm_inputs.py --rate 20
```

이 시험 전용 publisher는 직선 다점 경로 20개를 발행한다. 실제 `stack_lane`이나
다른 `/perception/lane_path` publisher와 동시에 실행하지 않는다.

### 터미널 3 — 신호등·정지선 정지 요청

명령이 묻는 값에 1단계에서 검증한 정지 요청 위치의 `y_med`를 입력한다.

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
source ~/FMA_ws/install/setup.bash

read -rp "검증한 정지선 y_med 임계값(0~1.10): " FMA_STOPLINE_TRIGGER_Y
ros2 launch stack_traffic stopline_distance_test.launch.py \
  oak_depth_enabled:=false \
  stopline_stop_y_ratio:="${FMA_STOPLINE_TRIGGER_Y}" \
  stopline_stop_distance_m:=0.0 \
  resume_on_green:=true \
  show_debug:=false
```

현장에서 사용한 `0.98`은 현재 정지선 ROI, 고정 장착 자세, 0.28m/s 이하에서만
검증된 값이다. 카메라 장착·ROI·속도가 바뀌면 범용값처럼 재사용하지 말고 1단계에서
다시 측정한다.

정지 진입 조건은 다음과 같다.

```text
적색 최근 5회 중 3회 이상
AND 정지선 검출·y 안정성 통과
AND 안정화된 정지선 y 비율 >= 입력한 임계값
```

이 모드는 실제 정지에 쓰지 않는 stereo depth를 장치 단계에서 꺼서 FPS를 확보한다.
시작 로그의 `camera=.../rgb-only`와 로그의 `z=invalid`는 정상이며,
`y_med`, `y_ok`, `line_near`를 확인하면 된다.

depth도 위치를 안정적으로 구분한다는 것이 확인됐을 때만 이중 gate를 쓴다.
둘 다 활성화하면 `y >= y 임계값`과 `z <= z 임계값`을 모두 만족해야 한다.

```bash
read -rp "검증한 정지선 y_med 임계값(0~1.10): " FMA_STOPLINE_TRIGGER_Y
read -rp "검증한 정지선 z_med 임계값(m): " FMA_STOPLINE_TRIGGER_Z
ros2 launch stack_traffic stopline_distance_test.launch.py \
  oak_depth_enabled:=true \
  stopline_stop_y_ratio:="${FMA_STOPLINE_TRIGGER_Y}" \
  stopline_stop_distance_m:="${FMA_STOPLINE_TRIGGER_Z}" \
  show_debug:=false
```

기동 직후에는 5회의 실제 YOLO 판단이 쌓일 때까지 `startup_hold=1`로
정지를 발행한다. 정지 후에는 신호등을 놓쳤다는 이유만으로 해제하지 않는다.
같은 target의 fresh YOLO bbox에서 초록색이 최근 5개 중 3개 이상일 때만
해제한다. unknown·미검출·template 초록은 해제 표본으로 세지 않는다.
`resume_on_green:=false`로 바꾸면 자동 재출발 없이 정지 래치를 유지한다.

### 터미널 4 — ADAS MGM, 시험 속도 0.28m/s

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
source ~/FMA_ws/install/setup.bash

ros2 run adas_mgm mgm_node --ros-args \
  --params-file ~/FMA_ws/src/adas_mgm/config/params.yaml \
  -p v_base:=0.28
```

아직 E-stop 정상 하트비트가 없으므로 `v_ref=0`이어야 한다. 신호등 정지는 E-stop이
아니라 일반 감속이며, `a_down=1.5m/s^2` rate limit을 거쳐 0으로 내려간다.

### 터미널 5 — 신호등 정지 요청 감시

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
source ~/FMA_ws/install/setup.bash

ros2 topic echo /perception/traffic_stop
```

시작 판단창이 쌓이는 동안은 `true`, 준비 완료 후 출발 전에는 `false`여야 한다.
정지 위치에서는 `true`, 초록 3/5 확인 후에는
다시 `false`여야 한다.

### 터미널 6 — MGM 출력 감시

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
source ~/FMA_ws/install/setup.bash

ros2 topic echo /adas/target_ref --field v_ref
```

출발 허용 전과 적색 정지 후에는 `0.0`, 주행 중에는 최대 `0.28`이 나와야 한다.

## 4. E-stop 허용 전 topic/CAN 확인

아래 명령은 별도 확인 터미널에서 순서대로 실행한다. 이 확인이 끝나기 전에는
터미널 7을 실행하지 않는다.

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
source ~/FMA_ws/install/setup.bash

ros2 topic info /perception/lane_path --verbose
ros2 topic info /perception/traffic_stop --verbose
ros2 topic info /adas/target_ref --verbose
ros2 topic echo /perception/traffic_stop --once
ros2 topic echo /adas/target_ref --once
ros2 topic echo /vehicle/vector --once
```

반드시 다음과 같아야 한다.

- `/perception/lane_path` publisher는 실제 lane 또는 임시 publisher 한 개다.
- `/perception/traffic_stop` publisher는 `stack_traffic_node` 한 개다.
- `/adas/target_ref` publisher는 `mgm_node` 한 개다. `dummy_ref_publisher`가 없어야 한다.
- traffic 노드 로그에서 `startup_hold=0`, 유효한 `y_med`, `y_ok=0`,
  `line_near=0`을 확인한다.
- `/perception/traffic_stop` 메시지의 `stop_required=false`를 확인한다.
- `TargetRef.state=0`, ref point가 한 개 이상이고 E-stop 허용 전 `v_ref=0.0`이다.
- `/vehicle/vector`가 한 번 이상 수신돼 PC↔dSPACE CAN 왕복이 확인된다.

CAN 헤더와 차량 피드백을 직접 해석하려면 다음을 사용한다.

```bash
python3 ~/FMA_ws/src/bridge_dspace/tools/can_dump.py --iface can0 --changes
```

정상 송신은 point `0x101`~`0x114`를 각각 한 프레임씩 보낸 다음 header
`0x100`을 보내며, header의 state는 `lane`, `n=20`, 주행 시
`v_ref=0.280m/s`, 정지 시 `0.000m/s`다.

### 터미널 7 — 마지막에 E-stop 입력 허용: 둘 중 하나만 실행

실제 LiDAR `/scan`을 사용할 수 있으면 아래 실제 노드를 사용한다.

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
source ~/FMA_ws/install/setup.bash

ros2 run stack_estop stack_estop_node
```

LiDAR 없이 신호등 체인만 검증할 때는 바퀴를 띄운 상태 또는 통제된 지상 시험에서만
아래 임시 정상 하트비트를 사용한다. `estop:false`는 주행 허용이라는 뜻이다.

```bash
cd ~/FMA_ws
source /opt/ros/humble/setup.bash
source ~/FMA_ws/install/setup.bash

ros2 topic pub -r 20 -p 20 \
  /perception/estop \
  fma_interfaces/msg/EstopRequest \
  "{header: auto, estop: false}"
```

## 5. 시험 중 정상 변화

```text
출발 허용 전
  stop_required=false, TargetRef v_ref=0.0

E-stop 정상/임시 하트비트 시작 후
  TargetRef v_ref가 0.28까지 상승

적색 + 정지선 위치 임계 진입
  stop_required=true
  TargetRef v_ref가 0.0으로 감소
  CAN 0x100 v_ref=0

같은 신호등의 fresh YOLO bbox에서 초록 3/5 확인
  stop_required=false
  TargetRef v_ref가 0.28까지 다시 상승
```

MGM은 `TrafficStop.stop_distance` 값 자체로 속도 프로파일을 만들지 않는다.
`stack_traffic`이 정지선 영상 위치(선택적으로 depth까지) 임계값으로 `stop_required`를 만들고, MGM은 그 bool을 보고
`v_ref=0`을 만든다. 또한 신호등 정지는 MGM의 `LANE`와 `WAYPOINT` 상태에서만 적용된다.
시험 중 `TargetRef.state`가 `2(AVOID)` 또는 `3(PARKING)`이면 즉시 중단한다.

## 6. 중단 순서

긴급 상황에서는 문서 순서보다 **물리 비상정지를 먼저 누른다**.

정상 중단은 다음 순서로 한다.

1. 터미널 7의 E-stop 정상/임시 publisher를 `Ctrl-C`로 종료한다.
2. 최소 300ms 이상 기다린 뒤 터미널 6에서 `v_ref=0.0`과 실제 차량 정지를
   확인한다. 시간만 보고 다음 단계로 넘어가지 않는다.
3. traffic, path, MGM, bridge 순서로 각각 `Ctrl-C` 한다.
4. 시험이 완전히 끝났을 때만 CAN을 내린다.

```bash
sudo ip link set can0 down
```

중요한 현재 한계:

- MGM은 LANE 상태의 `lane_path`와 WAYPOINT 상태의 `gps_path`를 미수신 또는
  0.5초 초과로 판단하면 정지한다. `traffic_stop`은 한 번 이상 수신한 뒤 0.5초
  끊기면 LANE/WAYPOINT 상태에서 정지를 요구한다.
- 반대로 E-stop 하트비트가 끊기면 MGM은 약 250ms 뒤 즉시 `v_ref=0`을 요구한다.
- MGM 또는 bridge가 죽었을 때의 30ms 정지는 dSPACE counter watchdog이 실제로
  구현되고 검증됐을 때만 보장된다.

따라서 이 버전은 물리 E-stop 담당자가 있는 저속 검증용이며 무인 시험용이 아니다.
