# 최근 주행 2회: 속도·조향·위치·state 추출

2026-09-22 기준 로컬 `drive_logs`의 가장 최근 **주행 시작 시각** 두 개를 추출했다.
주행 제어 코드·런처·설정은 변경하지 않는다. 각 CSV는 CAN 피드백 원본 한 행당 한 행이며,
시작 준비·정차·주행·종료 구간을 모두 보존했다. 미수신 구간은 보간하지 않는다.

| CSV | 기록 구간(KST) | 행 수 | 덤프 |
|---|---|---:|---|
| [16:05 주행](v2_20260920_160548_762545.csv) | 09-20 16:05:49.062 ~ 16:26:59.600 | 57,700 | v46 |
| [13:20 주행](v2_20260920_132033_650355.csv) | 09-20 13:20:33.909 ~ 13:30:35.557 | 60,165 | v45 |

## 요청 항목

| 열 | 단위 | 출처·의미 |
|---|---|---|
| `v_ref` | m/s | 기록 당시 코어·파라미터로 MGM snapshot을 재생한 최종 목표 속도 |
| `v_act` | m/s | `vehicle_vector.csv`의 `v`, dSPACE 실측 속도 |
| `str_ref` | rad | `vehicle_vector.csv`의 `str_ref`, dSPACE에서 돌려준 MPC 목표 조향각 |
| `str_act` | rad | `vehicle_vector.csv`의 `str`, dSPACE 실측 조향각 |
| `x`, `y` | m | 대응 MGM snapshot의 `gps_x`, `gps_y`; 세션 시작 CSV 첫 좌표 기준 로컬 위치 |
| `state` | 정수 | MGM 재생 출력의 `TargetRef.state`; CSV 미션 마커의 state가 아님 |

속도·조향 부호는 원본 그대로 보존했으며 조향각을 degree로 변환하지 않았다.
`x/y`는 차량 좌표계의 목표점이나 위·경도가 아니다. GPS 위치가 무효하면 빈칸이다.
dSPACE 원본 `x/y`는 `feedback_x/feedback_y`로 함께 보존한다.

`state`: 0 LANE, 1 WAYPOINT, 2 AVOID, 3 PARKING, 4 TRAFFIC, 5 ESTOP.
FINISH·SAFE_STOP은 이 단일 state와 별도의 상위 상태이므로 이 열만으로 구분하지 않는다.

## 시간 정렬 및 보조 열

- 기준은 CAN CSV의 `stamp_s`(Unix 초, 원본 정밀도 1ms)이다. `time_kst`와 `elapsed_s`도 제공한다.
- `v_act/str_ref/str_act/feedback_x/feedback_y`는 동일한 CAN 행에서 그대로 가져온다.
- MGM 값은 해당 CAN 시각 **이전 또는 같은 시각**의 가장 최근 snapshot을 사용한다.
- `mgm_age_ms`가 100ms를 초과하거나 이전 snapshot이 없으면 `matched=0`으로 표시하고
  `v_ref/state/x/y/gps_position_valid/mgm_tick`을 빈칸으로 둔다. 미래 표본이나 보간값을 넣지 않는다.
- `mgm_tick`은 원본 snapshot 인덱스, `gps_position_valid`는 기록된 GPS 위치 유효성이다.
- `feedback_gap_s`는 직전 CAN 행과의 시간 간격이다. 첫 행은 빈칸이다.
- 이 정렬은 기록된 시간의 비교이며 서로 다른 노드의 callback 및 CAN 전송 지연을 보정한 동시 측정은 아니다.

## 기록 공백과 검증 결과

- 16:05 세션: **16:15:25.165 → 16:26:58.750, 693.585초** CAN 공백이 있다.
  MGM dump에는 09-21 10:16:01까지 데이터가 있으며 최대 시간 공백은 64,086.790초이다.
  파일 수정 시각만 보고 이를 별도 신규 주행으로 취급하지 않았다. CAN이 없는 MGM 단독 후반 구간은 CSV에 추가하지 않았다.
- 시간 대응 없는 행: 16:05 세션 5행, 13:20 세션 14행.
- 시간 대응은 있지만 GPS 위치 무효인 행: 각각 63행, 58행.
- 두 덤프의 헤더 버전·구조체 크기·레코드 완전성을 검사했다(각 63,710 / 60,151 snapshot).
- 기록 버전에 맞는 코어를 사용해 원본 `transitions.csv`의 **35 + 58 = 93행**과
  속도·상위 상태·내비게이션·회피·신호·안전·미션·참조 소스·마지막 미션 필드를 대조했다. 모두 일치했다.
- CAN 원본의 시각·실속도·명령/실제 조향·피드백 좌표는 **117,865행 전체에서 원본 문자열과 동일**함을 검사했다.
- 원본 SHA256, 출력 SHA256, 재생 커밋은 [manifest.json](manifest.json)에 기록했다.

## 재현

원본 로그는 차량 PC에 보존되어 있으며 수백 MB의 원본 덤프는 이 PR에 포함하지 않는다.
Git 히스토리 전체와 g++/tar/Python 3가 필요하다. ROS 노드나 차량을 실행하지 않는다.

```bash
python3 analysis/latest_two_drives_20260922/extract.py \
  /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main/drive_logs
```

[extract.py](extract.py)가 기록 버전의 소스를 임시 폴더에 추출하고
[replay_signals.cpp](replay_signals.cpp)를 컴파일해 CSV 및 manifest를 생성한다.
지원하는 두 버전 외의 덤프·시간 역행·전이 불일치는 실패 처리한다.
