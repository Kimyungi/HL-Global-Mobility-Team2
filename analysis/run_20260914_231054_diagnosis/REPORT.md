# RViz 차량 모형 점멸 분석 — 2026-09-14 23:10 실행

대상: `drive_logs/v2_20260914_231054_732541`.
사용자 확인: 실제 차량의 반복 출발·정지가 아니라 RViz 차량 모형이 사라졌다 나타나는 현상.
이번 작업에서는 주행/표시 코드를 수정하거나 하드웨어를 기동하지 않았다.

## 결론과 확실성

추가 사용자 관찰: 차량 모형뿐 아니라 회피 레퍼런스 패스도 점멸했다.
회피 로그에는 경로 생성 성공 뒤 재검사 실패로 전환하는 기록이 있다. 실제 실행된
fixed_goals 코드는 이때 경로를 비우므로, 경로 소멸은 표시 문제만으로 설명할 수 없다.
차량 모형 점멸에는 map 기준 RViz와 차량 마커 사이의 TF 시각 불일치, 매 렌더의
DELETEALL이 후보로 남는다. GPS/CAN 데이터가 끊긴 기록은 보이지 않는다.
동일한 TF 시간 관계에서 변환 실패를 실제 tf2 Buffer로 재현했지만 실제 점멸 원인을
확정한 재현은 아니다.

다만 이 세션은 /tf, /integration_v2/view/markers, 화면 영상을 기록하지 않았으며
RViz 로그에도 프레임별 변환 오류는 없다. 따라서 점멸 프레임 각각의 원인을 로그로
확정한 것은 아니다. 아래 구조와 수신 통계에 근거한 우선 진단이다.

## 확인된 수신 상태

- GPS lateral.csv: 1,298행, 모든 행 RTK FIXED(quality=4).
- GPS 발행 간격 중앙값 약 100ms, 최대 109ms; 400ms 초과 공백 없음.
- GPS fix_age 최대 129ms.
- 차량 CAN 피드백: 13,063행, 간격 중앙값 10ms, 최대 14ms.
- CAN 브리지 TX/RX 카운터는 실행 동안 약 100Hz로 계속 증가하며 연결 오류 로그 없음.
- RViz/표시 노드는 중간 재시작 없이 23:13:05의 Ctrl-C에서 정상 종료.

## 표시 코드의 시간 관계

`src/adas_mgm/config/integration_v2.rviz`:

- Fixed Frame = map
- 카메라 Target Frame = v2_vehicle_view

`src/adas_mgm/tools/integration_view.py`:

- v2_vehicle_view는 base_link에 고정된 차량 프레임.
- render()는 100ms 주기로 실행하며 마커 stamp를 렌더 현재 시각으로 설정.
- 차량 사각형/헤딩 마커는 v2_vehicle_view 좌표로 발행.
- 매 프레임 DELETEALL 뒤 차량 등 ADD 마커를 발행.
- 마커 lifetime은 400ms.

`src/stack_gps/stack_gps/node.py`:

- map→base_link 동적 TF를 GPS 노드 tick의 메시지 시각으로 발행.
- 표시 노드의 렌더 타이머와 GPS 타이머는 독립적으로 실행.

GPS TF 최신 시각보다 차량 마커의 요청 시각이 앞서면 map으로 변환할 때
미래 외삽이 필요해진다. DELETEALL로 직전 차량을 지운 뒤 새 차량 ADD를 변환할 수
없는 순간이 있으면 점멸이 발생할 수 있다. 마커 유효시간을 늘리는 것만으로는
매 프레임 삭제/시각 불일치 문제를 해결하지 못한다.

GPS 웨이포인트 경로 마커는 map 좌표라 차량 프레임→map 변환이 필요하지 않다.
그러나 이번 fixed_goals 회피 경로는 path_io.py에서 base_link로 발행된다.
표시 노드는 해당 프레임을 유지하므로 회피 경로도 차량 모형처럼 동적 TF에 의존한다.
새 main_gap_path의 map 발행 방식과 이번 실행을 혼동해서는 안 된다.
직전 작업에서 Fixed Frame을 차량 프레임에서 map으로 바꾸면서 이 의존성이 생겼다.

## 재현

`reproduce_tf_timing.py`는 네트워크·하드웨어 없이 tf2 Buffer만 사용한다.
GPS TF가 1.000초와 1.100초에 있고 차량 마커가 1.150초를 요청하는 대표 상황이다.
실제 세션의 TF 재생은 아니다.

- 1.150초 요청: ExtrapolationException, latest data 1.100초.
- GPS TF와 같은 1.100초 요청: 성공.
- 최신 TF 요청(time=0): 성공.

## 별도로 확인된 실행 모드 불일치

세션의 avoid_planner_mode.txt = `fixed_goals`, backend = `native`다.
회피 로그에도 `fixed_goals`, `no collision-free cubic obstacle chain`,
`fixed obstacle chain blocked`가 나오며 신규 `pose=gps_map_enu; path_frame=map`
시작 문구는 없다. 새 main_gap_path 모드가 실제로 실행된 기록이 아니다.

런처 기본값이 fixed_goals로 남아 있고 런북에 이전 시험 명령도 공존한다.
정확히 어떤 명령을 복사했는지는 기록만으로 단정할 수 없지만, 명시적인
`avoid_planner_mode:=main_gap_path` 선택이 필요했던 실행 구조가 혼동을 만들 수 있다.
이 모드 불일치는 경로 유지 요청이 이번 시험에 반영되지 않은 이유다.

## 회피 경로의 실제 소멸 전환

`log_v2/ros/python3_104611_1789395055389.log`에 다음 전환이 기록됐다(KST).

| 생성 성공(target=True) | 재검사 실패(target=False) | 두 로그 간격 |
| --- | --- | --- |
| 23:12:06.627 | 23:12:06.703 | 약 76ms |
| 23:12:13.413 | 23:12:13.624 | 약 211ms |
| 23:12:14.028 | 23:12:14.122 | 약 94ms |

실패 이유는 `fixed obstacle chain blocked; goals retained`다.
fixed_obstacle_path.py의 step()은 시작 시 self.path를 None으로 초기화한다.
재검사에 실패하면 목표점은 남지만 경로는 비어 있고, path_io.py는 빈 Path를 발행한다.
위 간격은 로그 전환 간격이며 실제 화면 표시 지속시간을 측정한 값은 아니다.
좌표계가 두 위치 사이를 번갈아 이동했다는 직접 증거는 확보되지 않았다.

## 다음 수정 방향

1. 차량 표시를 최신 유효 GPS 자세로 map 좌표에 직접 변환해 GPS 경로와 같은 프레임에
   그리거나, 차량 마커의 시각을 사용한 TF와 일치시킨다. 경로 절대좌표는 유지한다.
2. DELETEALL을 매번 보내지 않고 동일 namespace/id 마커를 갱신하며,
   사라져야 하는 마커에만 삭제를 적용한다.
3. 런북의 현재 시험 명령과 런처 기본 모드를 일치시킨다.
4. 필요하면 /tf 및 표시 마커를 함께 기록해 실제 점멸 프레임을 검증한다.

GPS 헤딩은 후반에 융합 정렬이 -127°에서 +54°로 바뀌는 별도 기록도 있다.
이는 차량 표시의 방향 전환/회전 문제에는 영향을 줄 수 있으나 모형의 소멸·재등장과
동일한 현상이라고 단정하지 않는다.
