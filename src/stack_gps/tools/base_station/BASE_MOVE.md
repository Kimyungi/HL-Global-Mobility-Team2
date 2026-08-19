# 베이스 위치 옮기기 — 이미 등록된 지점끼리

[`BASE_LOCATIONS.md`](BASE_LOCATIONS.md) 표에 **이미 있는** 지점 사이를 오갈 때의 절차.

> 처음 세우는 자리라면 이 문서가 아니다 → [`BASE_SURVEY.md`](BASE_SURVEY.md).

---

## 먼저 — 좌표는 지워지지 않는다

| | 어디에 | 개수 |
|---|---|---|
| **좌표 숫자** | [`BASE_LOCATIONS.md`](BASE_LOCATIONS.md) | 지점을 늘려 가며 **쌓인다** |
| **지금 로드된 좌표** | F9P 플래시 (`setup_base.py` 가 RAM+BBR+FLASH 에 씀) | **한 벌만** |

표가 **서가**고 플래시가 **지금 펴 놓은 책 한 권**이다. 원주에서 한라대로 옮긴다고
원주 좌표가 사라지는 게 아니라, 나중에 원주로 돌아가면 표의 그 줄을 다시 넣으면 된다.

**표에 없는 값만 진짜로 사라진다** — 그래서 아래 2단계가 있다.

## 재측량하지 말 것

등록된 지점은 표의 숫자를 **그대로 다시 넣는 것이 정답**이다. 같은 자리에서 재측량해도
값이 cm 단위로 달라지고, 그 지점에서 기록한 코스가 그만큼 밀린다.

코스 재사용에 필요한 건 절대 정확도가 아니라 **일관성**이다. 그때 측량이 진짜 위치에서
20cm 틀렸더라도, 같은 좌표를 선언하고 같은 자리에 세우면 그때와 **같은 프레임**이
재현돼 코스 CSV 가 맞는다.

---

## 절차 5단계

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station
```

### 1. 안테나·삼각대를 그 지점의 등록된 자리·높이로 설치

표의 **"안테나 설치"** 칸을 보고 재현한다. 베이스는 자기가 어디 있는지 모르고 우리가
말해 준 좌표를 믿으므로, 안테나가 다른 자리면 **로버 위치가 통째로 그만큼 밀린다.**
**삼각대 높이도 포함**이다(타원체고가 그만큼 틀어진다).

### 2. 지금 들어 있는 값이 표에 있는지 확인

```bash
python3 read_base_position.py
```

표에 없는 값이 나오면 **덮기 전에** 출력 그대로 표에 행을 추가한다.

### 3. 그 지점 좌표를 플래시에 쓴다 ← 설정은 사실상 이것 하나

```bash
python3 setup_base.py --lat <위도> --lon <경도> --height <타원체고>
```

`fixType=5 (TIME — 베이스 정상)` 이 뜨면 성공.

### 4. 송출 시작

```bash
# B1 [베이스 PC]
python3 rtcm_server.py --radio /dev/ttyRadio

# V1 [차량 PC]
python3 ~/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py \
    --port /dev/ttyRadio --tcp-port 2101
```

### 5. 로버 launch 의 코스를 그 지점 것으로

`waypoint_csv:=` 에 **그 지점에서 기록한 코스**를 지정한다 (표의 "위치 ↔ 코스 대응").

⚠ **틀린 짝을 쓰면 조용히 실패한다** — RTK FIXED 는 멀쩡히 뜨는데 위치만 통째로 밀린다.
코스 CSV 첫 줄의 lat/lon 으로 장소를 구분할 수 있다.

---

## 안 해도 되는 것

- **재측량** (오히려 하면 안 된다 — 위 참조)
- **NGII 접속·인터넷** — 측량 때만 필요하다. 운용은 인터넷 없이 돈다
- **로버 설정 변경** — 로버는 받은 RTCM 을 그대로 쓴다
- **dSPACE·CAN 쪽 아무것도**

---

## 지점별 커맨드 (복붙용)

### 한라대학교 — 8/1 지점

```bash
python3 setup_base.py --lat 37.303841799 --lon 127.907284433 --height 183.9014
```
코스: `waypoints_straight_1_20260811_193556.csv` 등 (표 참조)

### 원주 운전면허시험장 — 8/18 지점

```bash
python3 setup_base.py --lat 37.300314764 --lon 127.979451327 --height 224.2647
```
코스: `waypoints_straight_1_20260818_160511.csv` (지정 구간 3종 포함)

> 새 지점을 측량하면 [`BASE_LOCATIONS.md`](BASE_LOCATIONS.md) 표와 **이 목록에도**
> 커맨드를 추가할 것.

---

## 옛 좌표를 잃어버렸을 때

1. 그 지점을 설정했던 PC 의 셸 히스토리 — `grep setup_base ~/.bash_history`
   (실행 커맨드에 좌표가 그대로 들어 있다)
2. 그 EVK 를 가져와 `python3 read_base_position.py`

둘 다 안 되면 그 좌표는 복구 불가다. 그 지점의 코스 CSV 는 버리고
[`BASE_SURVEY.md`](BASE_SURVEY.md) 로 재측량 + 코스 재기록.
