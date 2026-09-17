# 21:24 주행: estop 후 후진 종료 뒤 재출발 실패

대상: drive_logs/v2_20260914_212415_018504. 시각 KST.

직접 원인은 후진 종료 후 필요한 유효한 AVOID reference가 끝까지 생성되지 않은 것이다. estop은 해제됐고 GPS 위치와 heading도 유효했다.

| 시각 | 사건 |
| --- | --- |
| 21:24:41.081 | AVOID_ACTIVE 진입. 이미 avoid_path_n=0, v_ref=0 |
| 21:24:42.551 | auto_estop=1 |
| 21:24:52.541 | 약 10초 대기 후 후진 명령 -0.8m/s |
| 21:24:53.951 | auto_estop=0, DANGER_CLEARED로 후진 종료. 후진 명령 1.410초, 해당 기간 측정속도 적분 거리 약 0.517m |
| 이후 ~21:25:11.881 | safety=REVERSE_RECOVERY 유지, AVOID reference 무효, v_ref=0 |

후진 종료 후 1,794 snapshot 전부 auto_estop=0, gps_valid=1, gps_heading_valid=1, external_stop=0, avoid_path_n=0이었다. 따라서 앞선 21:03 주행의 IMU heading 소실과 원인을 구분해야 한다.

## 코드 조건

src/adas_mgm/core/manager_step.cpp:516에서 후진 종료 시 recovery_waiting_reference=true로 설정한다. 524행에서 AVOID_ACTIVE인 경우 AVOID provider의 유효 reference가 있어야 대기가 풀린다. GPS reference가 있어도 이 조건을 충족하지 않는다. 531행에서는 이 대기 상태도 REVERSE_RECOVERY로 표시하므로, safety=2가 계속 보인다고 실제 후진 명령이 계속 나가는 것은 아니다.

## 경로가 없는 이유와 확인 한계

log_v2/ros/python3_87272_1789388655530.log의 planner 진단:
`no collision-free cubic path with waypoint return at +2.7m; target=False`

진단은 21:24:43.409에 기록됐고 이후 사유 변경 로그가 없다. path_io.py는 진단 내용이 바뀔 때만 출력하므로 updates=1만 보이는 것으로 planner가 한 번만 실행됐다고 판단하면 안 된다.

gps_cubic_path.py는 최소 회전반경, 차량 차체와 장애물의 여유 간격, waypoint 복귀점 등 조건을 만족하지 못하면 경로를 비운다. 이 메시지는 후보 없음/복귀점 없음/곡률 또는 충돌 검사 실패 등을 묶은 최종 진단이라 정확히 어느 조건이 모든 후보를 거부했는지 현재 기록만으로 구분할 수 없다. raw LiDAR와 후보별 거부 사유는 이 분석에 확보되지 않았다.

또한 planner는 initial_surfaces와 last_surfaces를 유지해 장애물 검출이 사라진 뒤에도 충돌 검사에 사용한다. MGM 상태 콜백은 avoidance 활성 여부 변경 때 planner를 리셋하므로 AVOID_ACTIVE를 유지하는 후진 진입/종료 자체로는 이 기억이 초기화되지 않는다. 이 기억 때문에 막혔는지는 후보와 장애물 기하가 있어야 검증 가능하다.

후속 수정 검토는 후진 종료 위치에서 재계획한 후보가 거부되는 세부 사유를 기록하는 것, 보존 장애물 기하와 복귀점/회전반경 제약을 재현하는 것에 초점을 맞춰야 한다. estop 해제만으로 무효 경로를 무시하고 전진시키는 것은 이 원인을 해결하지 않는다.

raw dump ABI 확인, 5,673 snapshot 디코딩 및 오프라인 replay 수행. 운영 코드는 수정하지 않았다. replay와 원본 전이의 상태 및 경로 유효성을 대조했다.
