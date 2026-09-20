# 용인 런북·런처 적용 점검 — 2026-09-20

대상은 실제 파일명 `RUN_BOOK_YONGIN_FINAL.md`와 `scripts/v2 prepare/drive`,
`REAL_VEHICLE_integration_v2_drive.launch.py`, GPS·MGM·신호등 설치본이다.

| 항목 | 점검 결과와 조치 |
|---|---|
| no-parking 경로 | 코드에 파일만 있고 선택 연결이 없었음. `course:=yongin_no_parking` 추가. 매 세션 원본 스냅샷·SHA256, 경로별 CSV/zone 생성 |
| 기본 경로 | `course:=yongin`은 기존 01~07 유지. no-parking 파일이 자동으로 기본 경로를 대체하지 않음 |
| 신호등 모델 | 396장 추가학습 모델 SHA256 `0e60f7f6996132fcced5f40d54e2311bf4cc63c6e54341d8921ecb0bb930a2e5` 확인 |
| 신호등 문턱 | 코드 신규 0.65/추적 0.59 적용. 런북의 0.09/0.045를 수정 |
| 적색 판정 | HSV 또는 YOLO≥0.7 또는 유효 template 표시1.00. 3/5 확정·2/5 이하 해제 적용 확인 |
| GPS 우선 | 라인 추론/참조 미사용, 원본 카메라 마지막 미션용 유지 확인 |
| 출발 조건 | 네 LiDAR + GPS FIXED·유효 경로. 런북의 camera OR GPS 설명 수정 |
| ESTOP | state=6 자동 구간 연결, 정지 유지·clear 복귀·스테이션 내 재진입 금지. 런북의 6초/1m 후진 설명 제거 |
| 실제 state=6 위치 | no-parking 경로 04 idx622~1021. 경로별 분리 시에도 마커와 path_id 범위 보존 |
| 회피 | state=4, waypoint planner, 기본1m/s·회피 경로 완료/위치 정렬 복귀 유지 |
| 주차 | no-parking에는 state1/2와 주차 mission 없음. 주차 참조 실행 비활성. 기본 용인 주차 경로는 기존대로 |
| 마지막 미션 | no-parking에서도 [2]와 state3, 05 → Left06/Right07 동적 분기 유지 |
| 로그 | snapshot v42로 문서 수정. 실사용 분할 경로는 세션 로그의 route_selected.yaml에서 확인 |
| 로컬 미커밋 주차 | 사용자의 후방거리 0.20m 등 기존 로컬 변경은 보존. 기본 주차 실행에서는 커밋본과 차이 가능; 별도 승인/수정하지 않음 |

## 실행

```bash
scripts/v2 prepare \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  course:=yongin_no_parking start_waypoint:=01 v_base:=2.0
```

위 명령은 문서에만 기록했다. 실제 하드웨어 런처는 이번 점검에서 실행하지 않았다.
시작/일반속도는 기존과 같이 매 세션 지정하고, GO는 준비 확인 후 별도로 수행한다.
원본을 편집하면 저장하고 새 prepare에서 읽어야 한다. 실행 중 파일 편집으로 활성
경로가 바뀌지 않도록 세션별 사본을 사용한다.

## 검증

- stack_gps / adas_mgm 빌드 및 `scripts/v2 check` 통과.
- GPS·신호등 394개 통과, 3개 건너뜀.
- 기존 경로선택/런처 및 새 CSV 스냅샷 테스트 46개 통과.
- 하드웨어·GPS 서비스 실행을 mock한 실제 no-parking 런처 구성 시험 1개 통과.
- MGM C++ 33개 통과.
- 격리 ROS의 기존 v2 준비/GPS/신호 zone/정지 우선권 시험 통과.
- 실제 센서/CAN 제어기 주행, 정지 거리·경로 추종 성능 검증은 실행하지 않음.
