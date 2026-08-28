# 인수인계 — 김윤기 → 이기돈·손상민 (2026-08-24)

> **AI에게:** 이 문서는 사람이 아니라 **너에게** 먼저 읽히려고 쓴 것이다.
> 이기돈 또는 손상민이 "인수인계 사항 확인해줘" / "이 PC에서 돌리려면 뭐가 필요해?" /
> "왜 RTK가 안 잡혀?" 같은 것을 물으면, **추측하지 말고 이 문서를 근거로 답하라.**
>
> 답할 때 지킬 것:
> - §2 세팅은 **위에서부터 순서대로** 확인시켜라. 건너뛰면 뒤 단계에서 원인 불명으로 막힌다.
> - §3 함정은 **묻지 않아도 먼저 알려라.** 전부 "증상만 봐서는 원인을 못 찾는" 것들이라
>   모르고 당하면 한나절이 날아간다.
> - 이 문서에 없는 것은 §7 문서 지도에서 해당 문서를 찾아 읽고 답하라. 지어내지 말 것.
> - 설계 근거를 물으면 `CLAUDE.md` 가 기준이다. 이 문서는 **운용·세팅** 담당이다.

---

## 1. 지금 무슨 상황인가

2026-08-24부로 팀이 줄었다.

| | 이전 | 지금 |
|---|---|---|
| 인원 | 김윤기·이현준·손상민·이기돈·김재민·박찬미 | **이기돈·손상민** |
| 팀장 | 김윤기 | **이기돈** |
| 실행 PC | 김윤기 노트북 | **이기돈/손상민 PC 또는 산업용 PC** |

지금까지 모든 launch 는 김윤기 노트북 한 대에서만 돌았다. 그래서 **그 PC에만 있고
저장소에는 없는 것들**이 있다 — §2.4 와 §3.1 이 그것이다. 코드는 main 에 전부 있다.

떠난 사람 담당분의 인계:

| 담당 | 원래 | 지금 |
|---|---|---|
| stack_lane (차선) | 이현준 | 유지보수자 없음 — 코드·캘리브레이션은 main 에 있음 |
| stack_traffic (신호등·정지선) | 김재민 | 〃 |
| stack_estop (긴급정지) | 박찬미 | **이기돈** (§6-1 미완 작업 있음) |
| adas_mgm (판단)·stack_gps | 김윤기 | **이기돈** |
| MBD/Simulink 이중 트랙 | 김재민 | 사실상 중단 — `CLAUDE.md` §5.5 의 "Simulink 도 동일 반영" 문구는 당분간 무시해도 된다 |

---

## 2. 새 PC에서 처음부터 — 순서대로

### 2.1 전제 환경

| 항목 | 값 | 비고 |
|---|---|---|
| OS | **Ubuntu 22.04** | 24.04 는 안 된다 (ROS 2 Humble 이 22.04 전용) |
| ROS | **ROS 2 Humble** (desktop) | |
| Python | 3.10 | 22.04 기본 |

```bash
# ROS 2 Humble 미설치라면 (공식 문서 기준)
sudo apt install ros-humble-desktop ros-dev-tools
```

### 2.2 저장소

```bash
git clone https://github.com/Kimyungi/HL-Global-Mobility-Team2.git ~/FMA_ws
cd ~/FMA_ws
```

**경로는 반드시 `~/FMA_ws`.** launch 파일 여러 곳이 `~/FMA_ws/src/...` 절대경로를
기본값으로 갖는다 (모델 가중치, 호모그래피, 웨이포인트 CSV). 다른 이름으로 클론하면
launch 인자를 매번 손으로 넘겨야 한다.

### 2.3 의존 패키지

```bash
# ROS 의존성 (rosdep 이 package.xml 을 읽어 채운다)
sudo rosdep init 2>/dev/null; rosdep update
cd ~/FMA_ws && rosdep install --from-paths src --ignore-src -r -y

# 파이썬 — rosdep 이 안 잡는 것들
pip3 install --user pyserial depthai opencv-python
```

**차선(stack_lane)을 쓸 때만** PyTorch 가 추가로 필요하다. 김윤기 노트북·산업용 PC는
NVIDIA 가 없고 인텔 Arc iGPU 라 XPU 빌드를 썼다.

| PC | 설치할 것 |
|---|---|
| 인텔 GPU (산업용 PC 등) | XPU 빌드 PyTorch → launch 인자 `lane_device:=xpu` (기본값) |
| NVIDIA GPU | 일반 CUDA 빌드 → `lane_device:=0` |
| GPU 없음 | CPU 빌드 → `lane_device:=cpu` (추론 390ms/frame ≈ 2.5Hz, 느리지만 동작함) |

김윤기 노트북 기준 조합: `torch 2.12.1+xpu`, `torchvision 0.27.1`, `depthai 3.6.1`,
`numpy 1.26.4`, `scipy 1.8.0`. **차선(stack_lane)은 이 조합에서 정상 동작한다.**

> 차선을 안 쓸 거면 PyTorch 없이도 된다 — launch 에 `lane_enabled:=false`,
> 출발 인가는 `ros2 run adas_mgm go --skip-lane`.

**⚠ 신호등(stack_traffic)은 이 PC 조합에서 기동 불가 상태다 (2026-08-24 실측).**
`ultralytics` → `torchvision` 을 끌어오는데 `import torchvision` 자체가 실패한다:

```
RuntimeError: operator torchvision::nms does not exist
```

torchvision 이 지금 깔린 torch 와 다른 버전에 맞춰 빌드된 것이다(XPU 빌드 torch 와
일반 torchvision 을 섞은 결과로 보인다). stack_lane 은 torchvision 을 안 쓰므로 멀쩡하고,
**stack_traffic 만 막힌다.** 신호등을 쓰려면 torch/torchvision 을 **짝이 맞는 조합으로**
다시 설치해야 한다 — 그때 stack_lane 의 XPU 동작이 유지되는지 함께 확인할 것.
담당자(김재민)가 떠났으므로 이 문제를 아는 사람이 없다.

### 2.4 저장소에 **없는** 파일 — 수동으로 채워야 한다

```bash
ls -l ~/FMA_ws/src/stack_lane/models/yolopv2.pt
```

**`yolopv2.pt` (156MB) 는 gitignore 대상이라 clone 에 안 딸려온다.** 없으면 stack_lane 이
기동에 실패한다. YOLOPv2 공식 릴리스에서 받아 저 경로에 두거나, 김윤기 노트북에서
복사한다. 차선을 안 쓸 거면 필요 없다.

호모그래피(`src/stack_lane/config/homography.json`, 2026-08-11 실측 RMS 4cm)와
웨이포인트 CSV·구간 YAML(`src/stack_gps/waypoints/`)은 **저장소에 있다.** 다시 만들 필요 없다.

### 2.5 udev 규칙 3종 — 최초 1회, sudo 필요

이걸 안 하면 `/dev/ttyRadio`, `/dev/ttyRover` 같은 이름이 안 생기고 `can0` 도 안 올라온다.
문서·launch 의 모든 명령이 그 이름을 전제하므로 **여기서 막히면 그 뒤가 전부 막힌다.**

```bash
cd ~/FMA_ws

# ① CAN 자동 셋업 — Kvaser Leaf v3 꽂으면 can0 이 CAN FD(1M/2M) 로 자동 up
sudo src/bridge_dspace/tools/can_setup/install.sh          # 개발 PC는 --vcan 추가 (루프백용)

# ② GPS 수신기 명명 (/dev/ttyRover, /dev/ttyRadio, /dev/ttyF9P_uart2)
sudo cp src/stack_gps/tools/base_station/99-ublox-f9p.rules /etc/udev/rules.d/

# ③ 로버 수신기 USB 리셋 권한 (아래 설명)
sudo cp src/stack_gps/tools/99-ublox-f9p-usbreset.rules /etc/udev/rules.d/

sudo udevadm control --reload && sudo udevadm trigger
```

③은 로버 F9P 가 "USB에는 열거된 채 NMEA 만 안 나오는" 고장을 자동 복구하기 위한 것이다
(`GgaLink` 가 20초 무수신 시 USB 재열거). 권한이 없으면 그 복구가 조용히 실패한다.

확인:

```bash
ip link show can0                    # state UP
ls -l /dev/ttyRover /dev/ttyRadio    # 심볼릭 링크 존재
```

### 2.6 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/FMA_ws
colcon build          # ⚠ --symlink-install 금지 (§3.2)
source install/setup.bash
```

`echo "source ~/FMA_ws/install/setup.bash" >> ~/.bashrc` 해두면 편하다.

### 2.7 하드웨어 없이 되는 데까지 확인

```bash
# CAN 루프백 (vcan) — dSPACE 없이 왕복 성립 확인
ros2 launch bridge_dspace loopback_test.launch.py
ros2 topic hz /vehicle/vector        # ≈100Hz 면 OK

# MGM 단독 — 10ms 루프와 지터 로깅
ros2 launch adas_mgm mgm.launch.py

# 단위시험 전부
colcon test && colcon test-result --verbose
```

---

## 3. 함정 — 증상만 봐서는 원인을 못 찾는 것들

### 3.1 카메라 USB3 가 RTK 를 죽인다 ★ 가장 악질

OAK-D 가 USB3(SuperSpeed)로 열거되면 그 방사 잡음이 GNSS L1(1575MHz)을 덮어 로버
C/N0 를 **최대 16.5dB** 떨어뜨린다. 같은 안테나 위치에서 USB3 22dB(DGPS 고착) ↔ USB2 39dB(FIXED).

**악질인 이유:** 위성 수도, HDOP 도, RTCM 유입량도 **전부 정상으로 보인다.** 평소 보는
상태줄로는 아무 이상이 없는데 FIXED 만 안 잡힌다. C/N0(GSV)를 따로 봐야 보인다:

```bash
python3 ~/FMA_ws/src/stack_gps/tools/rtk_probe.py
```

**2026-08-24부터 기본값이 안전한 쪽(`usb_speed:=high`, `camera_fps:=10`)으로 뒤집혀 있다.**
그전에는 매 launch 마다 손으로 붙여야 했고, 한 번 잊으면 위 증상이 났다. 이제는 그냥 두면 된다.

`camera_fps` 를 같이 내리는 건 선택이 아니다 — USB2 대역폭이 ~40MB/s 라 720p 30fps(83MB/s)가
안 들어간다. 추론이 5.8Hz 라 10fps 로도 손실은 없다. USB3 가 정말 필요하면:
`usb_speed:=super camera_fps:=30`.

적용 확인: stack_lane 콘솔에 `USB 링크 속도 제한: HIGH`.

### 3.2 `colcon build --symlink-install` 금지

이 워크스페이스는 **일반 `colcon build`** 로 통일한다. 두 방식을 섞으면 `stack_gps` 가
`PackageNotFoundError` 로 즉사하는데, **이미 열려 있던 터미널에서만** 터진다 — 새 터미널은
멀쩡해서 재현이 안 되고 원인을 엉뚱한 데서 찾게 된다. 섞였다 싶으면:

```bash
rm -rf build install log && colcon build
```

### 3.3 베이스 좌표와 웨이포인트 CSV 는 **한 세트**다

베이스 수신기는 한 번에 한 위치의 좌표만 플래시에 갖는다. 안테나를 옮기면 그 위치 좌표로
다시 설정해야 하고, **그 위치에서 기록한 CSV 만** 써야 한다. 짝이 어긋나면 트랙 전체가
평행이동한 채로 주행한다 — 코스는 그럴듯하게 따라가는데 실제 위치가 어긋난다.

등록된 위치와 코스 대응: `src/stack_gps/tools/base_station/BASE_LOCATIONS.md`
현재 플래시에 들어 있는 것: **한라대 `halla_20260819`**. 원주 운전면허시험장
`outdoor_20260818` 은 보관 상태(코스가 살아 있어 되돌릴 수 있음).

베이스 이동 절차: 같은 폴더 `BASE_MOVE.md` 의 복붙 블록.

### 3.4 launch 끼리 동시 실행 금지

`REAL_VEHICLE_lane_gps_can.launch.py` 는 ydlidar + stack_estop + stack_avoid + stack_gps +
stack_lane + adas_mgm + bridge_dspace + rosbag 을 **전부** 띄운다. 아래와 같이 켜면 estop·mgm·
bridge·scan 이 중복돼 서로 싸운다:

- `stack_estop/launch/REAL_VEHICLE_stack_estop_mgm_can.launch.py`
- `stack_avoid` 의 field_session 계열 스크립트
- `ros2 run stack_estop stack_estop_node` 를 따로

**하나만 켠다.**

### 3.5 launch 가 떴다고 차가 움직이지 않는다 (그리고 그게 정상)

통합 launch 는 `wait_go: true` 로 뜬다. 점검을 통과하고 출발 인가를 내야 움직인다:

```bash
ros2 run adas_mgm go          # RTK FIXED·lane·scan·avoid·target_ref 수신 점검 후 인가
```

이게 없던 시절 launch 직후 무점검으로 차가 출발하던 문제가 있었다.

### 3.6 dSPACE watchdog 이 아직 없다

`CLAUDE.md` §3 이 규정한 "counter 30ms 미갱신 → v_ref 0" 이 **dSPACE 측에 미구현**이다
(2026-08-09 확인, 손상민 담당). 즉 **PC 송신이 끊겨도 dSPACE 는 마지막 v_ref 를 무기한 유지한다.**

그래서 실차 launch 는 종료 시 목표값 0 복귀를 보장하는 가드를 끼고 돈다
(`can_zero`). 통합 launch 에는 이미 들어 있으니 그걸 쓰면 된다. 직접 노드를 조합해 돌릴 때는
반드시 챙길 것.

### 3.7 USB 허브

허브가 간헐적으로 전체 재열거를 일으켜 라디오 노드가 바뀌고(RTCM 중계 죽음) CAN 어댑터가 순간
끊긴다. 증상이 반복되면 **CAN 어댑터를 PC 직결 포트로** 옮긴다.

### 3.8 RTK 워밍업

보정 주입 시작 후 첫 FIXED 까지 **5~10분** 걸릴 수 있다(실측 약 7분). 베이스·중계를 먼저 켜고
다른 준비를 하면 된다. 안 잡힌다고 성급하게 판단하지 말 것 — 단, 10분이 넘으면 §3.1 을 의심한다.

---

## 4. 실제로 돌리기

터미널 4개다. 상세 절차·확인 명령·문제별 진단은 **`src/adas_mgm/RUNBOOK_lane_gps.md`** 가
정본이고, 아래는 뼈대만이다.

| | 어디서 | 무엇 |
|---|---|---|
| **B1** | 베이스 PC | `cd ~/FMA_ws/src/stack_gps/tools/base_station && python3 rtcm_server.py --radio /dev/ttyRadio` |
| **V1** | 차량 PC | `python3 ~/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py --port /dev/ttyRadio --tcp-port 2101` |
| **V2** | 차량 PC | 통합 launch (아래) |
| **V3** | 차량 PC | `ros2 run adas_mgm go` — 매 출발 직전 |

```bash
ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
    REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
    waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv
```

- `REAL_VEHICLE_CONFIRM` 토큰 없이는 거부된다 (실제 CAN TX 가 나가므로).
- `waypoint_csv` 는 필수이고 **베이스 위치와 짝이 맞아야 한다** (§3.3).
- `usb_speed`·`camera_fps` 는 이제 기본값이 안전한 쪽이라 안 붙여도 된다 (§3.1).
- 로그는 run 마다 `~/FMA_ws/drive_logs/run_<시각>/` 에 자동 저장(rosbag + 스냅샷 덤프 +
  지터 CSV + lateral CSV). 사후 분석의 재료이므로 지우지 말 것.

상태 관찰: `ros2 run adas_mgm state` (전이할 때마다 한 줄) / `--all` (매 초).

**GPS 단독 주행**만 할 거면 `src/stack_gps/DRIVE_GUIDE.md` 가 별도 절차를 갖고 있다.

---

## 5. 산업용 PC 를 쓸 때 추가로 알 것

산업용 PC 는 **베이스 역할로 세팅된 이력**이 있다. 문서: `src/stack_gps/tools/base_station/INDUSTRIAL_PC_SETUP.md`

- 그 문서의 **"실행 금지" 목록을 반드시 지킬 것** — `setup_base.py`,
  `measure_base_position.py`, `ntrip_inject.py` 를 베이스 PC 에서 실행하면 확정 좌표가
  오염되고, 그러면 §3.3 의 좌표↔코스 짝이 통째로 깨진다.
- 산업용 PC 를 **차량 로버**로 쓸 경우: FST-UEF9P 를 USB 연결하면 udev 가 `/dev/ttyRover` 를
  만든다. 그 뒤로는 §2·§4 를 그대로 따르면 된다.
- 이 PC 는 NVIDIA 가 없다. 차선 추론은 인텔 Arc iGPU(XPU) 로 172ms/frame ≈ 5.8Hz.
  XPU 초기화가 실패하면 `lane_device:=cpu` 로 폴백(390ms/frame).
- RTCM 배포 경로는 **라디오(현장 표준, 인터넷 불필요)** 와 Tailscale 두 가지다. 현장은 라디오.

---

## 6. 남긴 미완 작업

### 6-1. `EstopRequest.rear_clear` 구현 — **이기돈**

2026-08-24 에 MGM 에 **후진 탈출**을 넣었다. 회피 불가 장애물 앞에서 estop 이 걸리면
차가 영영 못 빠져나오는 교착(장애물이 치워질 일이 없는 경우)을, estop 이 충분히 오래
유지되면 곧게 조금 물러나 회피가 성립하는 거리를 만드는 것으로 푼다.

**MGM 쪽은 끝났고, 후방 여유를 판정하는 쪽이 비어 있다.**

- 계약: `fma_interfaces/EstopRequest.rear_clear` (bool) — "지금 뒤로 빼도 되는가"만 답한다.
- estop 과 같은 **레벨 신호**다: 매 스캔 재평가, 래치 금지.
- **모르면 false.** 지금은 아무도 안 채우므로 항상 false 이고, 그래서 후진 기능은
  **잠긴 채 안전하게** 있다. 켜려면 이걸 먼저 구현해야 한다.
- 어느 라이다를 쓸지(후방 단독 / `multi_lidar_fusion` 4개 통합)는 **이기돈 판단**이다.
  MGM 은 bool 하나만 본다.
- 참고 코드: 닫힌 PR #44 의 브랜치 `chanmi-followup-20260819` 에 박찬미가 만든
  후방 ROI 클러스터 분석(`analyze_rear_scan`)이 있다. 브랜치는 안 지웠다.
- 켜는 법: `adas_mgm/config/params.yaml` 의 `escape_after_cycles` 를 1000(=10초)으로.
  되돌리기: `ros2 param set /adas_mgm_node escape_after_cycles 0`.
- 상세 설계·안전 불변식 6개: `CLAUDE.md` §4 "avoid 안의 후진 탈출 페이즈".

### 6-2. 음수 v_ref 실차 확인 — **손상민**

후진 탈출은 PC 가 **음수 v_ref** 를 보내는 첫 사례다. 그전까지 PC 는 0 이상만 보냈고,
`PROTOCOL.md` 도 "정지 = 0" 까지만 규정했었다. MPC·하위 PI 가 음수를 그대로 후진으로
처리한다는 확인은 받았지만 **실차에서 바퀴 방향을 눈으로 한 번 보는 것**이 남았다.

### 6-3. dSPACE watchdog — **손상민**

§3.6. `CLAUDE.md` §3 이 규정한 대로 구현되면 §3.6 의 `can_zero` 가드 의존을 덜 수 있다.

### 6-4. 실차 미검증인 채로 main 에 있는 것들

`CLAUDE.md` 가 "실차 미검증"이라고 표시해 둔 항목들이다. 되돌리는 법이 각 항목에 적혀 있다.

| 항목 | 되돌리기 |
|---|---|
| GPS 재합류 ref[0] 하한 상향 (`rejoin_target_min_m` 1.8) | `CLAUDE.md` §3 ⓒ |
| 곡선 선행 보상 2종 (`rejoin_curve_ff`, `rejoin_curve_margin`) | `ros2 param set /stack_gps_node rejoin_curve_ff 0.0` |
| 후진 탈출 (§6-1) | `escape_after_cycles 0` (지금 기본값) |

### 6-5. 닫힌 PR — 브랜치는 남아 있다

| PR | 왜 닫았나 | 브랜치 |
|---|---|---|
| #37 (박찬미) | #44 와 중복 | `chanmi-reverse-recovery` |
| #44 (박찬미) | 후진 로직을 MGM 으로 재구현. **후진 외의 부분은 살릴 가치가 있다** — LiDAR scan gap 모니터, static extent 필터, 오프라인 검증 도구 | `chanmi-followup-20260819` |
| #29 (김재민) | MxID 핀닝·USB2 파라미터가 이미 main 에 반영됨 | `agent/stack-traffic-oak-followup` |

**#45 (손상민, 4-LiDAR ICP 주차)는 draft 로 열려 있다.** 본인 작업이라 그대로 뒀다.

---

## 7. 문서 지도 — 무엇을 어디서 보나

| 알고 싶은 것 | 문서 |
|---|---|
| **설계가 왜 이런가** (스테이트 머신·우선권·계층) | `CLAUDE.md` ← **모든 설계 판단의 기준** |
| 통합 주행 절차 (터미널 4개, 문제별 진단) | `src/adas_mgm/RUNBOOK_lane_gps.md` |
| GPS 단독 주행 | `src/stack_gps/DRIVE_GUIDE.md` |
| 회피 현장 시험 | `src/adas_mgm/RUNBOOK_avoid_field_test.md` |
| CAN 배선·bringup·watchdog 검증 | `src/bridge_dspace/CAN_BRINGUP.md` |
| CAN 프레임 레이아웃·양자화 (dSPACE 와의 합의 원본) | `src/bridge_dspace/PROTOCOL.md` |
| dSPACE 로그 정합·측정 변수 | `src/bridge_dspace/DSPACE_LOGGING.md` |
| 베이스 좌표 목록·코스 대응 | `src/stack_gps/tools/base_station/BASE_LOCATIONS.md` |
| 베이스 이동 절차 | `src/stack_gps/tools/base_station/BASE_MOVE.md` |
| 산업용 PC 세팅 | `src/stack_gps/tools/base_station/INDUSTRIAL_PC_SETUP.md` |
| 카메라 캘리브레이션(호모그래피) | `src/stack_lane/CALIBRATION_GUIDE.md` |
| 각 스택이 무엇을 책임지나 | `src/<stack>/REQUIREMENTS.md` |
| git 사용법 (초심자용) | `docs/GIT_GUIDE.md` |

---

## 8. 빠른 진단표

| 증상 | 먼저 볼 곳 |
|---|---|
| RTK 가 FIXED 로 안 감 (위성·HDOP·RTCM 은 정상) | **§3.1** — C/N0 를 `rtk_probe.py` 로 확인 |
| RTK 가 아예 안 잡힘, NMEA 한 줄도 없음 | 로버 수신기 출력 사망 — USB 재삽입. udev ③(§2.5) 설치하면 자동 복구 |
| `PackageNotFoundError: stack_gps` | **§3.2** — 빌드 방식 혼용. `rm -rf build install log` 후 재빌드 |
| launch 는 떴는데 차가 안 움직임 | **§3.5** — 정상이다. `ros2 run adas_mgm go` |
| 차가 트랙과 나란히 어긋난 채로 감 | **§3.3** — 베이스 좌표 ↔ 웨이포인트 CSV 짝 |
| CAN 안 올라옴 (`can0` 없음) | udev ①(§2.5) 설치 여부 → 어댑터 재삽입 → **§3.7** 허브. dmesg 에 `kvaser_usb` 가 없으면 커널이 Leaf v3 를 모르는 것 → CAN_BRINGUP.md §1 |
| estop/조향이 이상하게 겹침 | **§3.4** — launch 중복 실행 |
| PC 껐는데 차가 계속 굴러감 | **§3.6** — dSPACE watchdog 미구현. `can_zero` 가드 포함된 launch 를 쓸 것 |
| stack_lane 기동 실패 | **§2.4** — `yolopv2.pt` 없음 |
| stack_traffic 기동 실패 (`torchvision::nms does not exist`) | **§2.3** — torch/torchvision 짝 불일치. 이 PC 의 기존 문제이며 신호등만 영향 |
