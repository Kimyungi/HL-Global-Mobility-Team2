# 장애물을 보면서 접근한 원인 및 좌우 부호 점검

대상: 2026-09-14 21:24:15 실행, Path 04. inputs.csv/replay.csv/원본 vehicle_vector.csv를 시각으로 대조. 아래 시각은 KST. 운영 코드 수정 없음.

확인 결과 이번 주행에서는 유효한 회피점이 한 번도 발행되지 않았다. 좌우가 반대인 회피점으로 장애물을 향했다고 볼 근거가 없다. 접근 중에는 GPS/차선 reference를 따랐고, 회피 진입 후에는 경로 무효로 정지 명령이 나왔지만 실제 감속에는 시간이 걸렸다.

| 시각 | 로그 의미 |
| --- | --- |
| 21:24:36.012 | TTC 유한값 등장. 스캔에서 전방 물체를 관측해도 이때 회피 obstacle_detected는 false. 같은 물체인지 원시 scan 부재로 확인 불가 |
| 21:24:40.182 | GPS에서 차선 reference로 전환, v_ref=2m/s |
| 21:24:40.962 | obstacle_detected=true 최초. TTC 1.738초, 속도 약 1.923m/s. TTC×속도로 추정한 범퍼 앞 거리 약 3.34m |
| 21:24:41.082 | AVOID_ACTIVE 진입. 약 0.12초 전진(속도 적분 약 0.23m) 뒤 전이. avoid_path_n=0, v_ref=0 |
| 21:24:42.552 | 별도 auto_estop=true. 실측 속도 약 0.769m/s |
| 21:24:43.292 | 실측 절대속도 0.05m/s 미만. 정지 명령부터 약 2.21초, 전진 속도 적분 약 2.456m |

거리 추정은 snapshot의 실측 속도를 적분한 값이며 장애물까지의 실제 잔여 여유나 접촉 여부를 뜻하지 않는다. 원시 scan/영상이 없어 충돌 여부는 확정할 수 없다.

## 감지와 회피 개시 조건

node.py의 검출은 범퍼 앞 3.5m 이내, 검출 반폭 0.5m 조건이다. RViz 점이 보이는 것과 obstacle_detected=true는 다르다. Zone 밖에서도 검출은 수행하지만 path_io.py:164는 MGM AVOID_ACTIVE 전까지 경로 계산을 막는다. manager_step.cpp:72는 독립 GPS 5샘플을 확인한 뒤 Zone 소유권을 전환한다. 이번 로그에서 obstacle_detected부터 상태 전환까지 지연은 0.12초였다. 5초 일찍 물체를 봤더라도 동일 물체 여부, 검출 거리/폭 충족 여부와 Zone 조건을 구분해야 한다.

## 회피 계산 시작 후 관측된 발행 지연

Zone 진입 전 avoid_updated 간격은 약 0.1초. 21:24:41.062 → 43.412에 2.35초 공백 발생. 이후 간격은 2.27–3.55초로 반복된다. 첫 planner 메시지는 21:24:43.409의 `no collision-free cubic path with waypoint return at +2.7m`이다.

node.py:on_scan은 감지 뒤 _station_reference와 모든 곡선 후보 검사를 동기적으로 완료해야 AvoidStatus를 publish한다. main은 rclpy.spin(node) 기본 실행이다. path_io.py:_path_goals는 장애물 단면/측면 후보를 순회하고 gps_cubic_path.py:_curve는 각 곡선 4개 scale을 검사한다. 많은 후보 또는 충돌 검사로 계산이 오래 걸리면 감지 결과 발행도 함께 늦어진다. 첫 긴 공백과 planner 첫 결과 시점이 일치하므로 이 경로의 계산 지연이 유력하다. 정확한 계산 구간별 시간은 기록되지 않아 CPU 스케줄링 등 다른 지연 기여도와 분리해 확정할 수 없다.

wrapper의 avoid stale 기준은 0.5초이고, 초과하면 obstacle_detected=false와 TTC=1e9로 보정한다(mgm_node.cpp:835). 따라서 snapshot의 장애물 검출이 중간에 false가 된 것은 장애물이 실제로 사라졌다는 증거가 아니다. 이번 후진 분석에서도 이 점을 함께 해석해야 한다. 독립 estop 입력은 별도로 동작했다.

## 정지 명령과 실제 차의 응답

21:24:41.082부터 v_ref=0이지만 차량은 약 2.46m 추가 전진했다. CAN bridge는 이 구간 100Hz 수준 TX/RX 누계가 지속됐고 오류 기록은 없다. 다만 TX 누계만으로 각 명령의 수신/적용을 입증할 수는 없다. CAN TargetHeader는 속도 목표를 보내며 별도 immediate_stop/브레이크 필드는 이 송신 코드에 없다. 따라서 코어의 immediate_stop이 실제 즉각적인 제동을 뜻하지 않는다. 차량 측 감속/제동 제어 및 명령 적용 시각을 대조해야 한다.

경로가 비면 MGM은 직전 reference 기하를 hold한다. 이번 hold 값은 x≈2.558m, y≈+0.021m로 거의 정면이었다. 이것은 새 회피점이 아니다. 실제 str_ref는 정지 구간에 0 근처였다.

## +/- 점검

- LiDAR: rel=scan_angle−front_center, x=lidar_x+r*cos(rel), y=lidar_y+r*sin(rel). 차체 좌표에서 y+는 왼쪽이다. 현재 front_center=87도. 실제 센서 장착과 스캔 좌우 일치는 raw scan/현장 기준점이 없어서 미확인.
- ENU ↔ 차체: station_path.py의 정회전/역회전식이 일치. 임의 yaw=-2.35rad에서 ±y 왕복 변환 확인.
- 대칭 후보 (2.66,+0.76), (2.66,-0.76)를 순수 planner에 입력해 target y가 각각 +0.76,-0.76으로 유지됨을 확인.
- 왼쪽 회피 S곡선은 진입 곡률이 +이고 측면 목표점에 도달할 때 곡률은 −가 될 수 있다. 이번 대칭 예제는 진입 +0.3813, 목표점 −0.3813(오른쪽은 반대). 목표점 y와 curvature 부호가 다르다는 이유만으로 부호 오류라고 볼 수 없다.
- MGM v2 및 CAN bridge는 provider의 x/y/yaw/curvature를 복사하며 이 경로에 좌우 반전 게인이 없다. ref_points.py의 예전 ray/lat_sign 코드를 현재 MGM 회피 출력으로 혼동하면 안 된다.
- 실제 GPS 주행 일부에서 target y<0에 str_ref>0이 관측됐다. str_ref의 좌우 물리 정의 및 dSPACE 내부 변환은 이 로그로 확정할 수 없다. 유효한 AVOID reference가 0건이므로 회피 실차 부호를 검증한 것은 아니다.

우선 점검할 항목은 ① 경로 계산 시간 계측과 감지 발행 분리, ② 후보별 실패 사유/장애물 기하 기록, ③ 현재 2m/s에서 정지 응답과 검출·Zone 시작 위치의 여유, ④ 유효한 회피 출력 확보 후 좌우 명령과 실제 차체 방향 대조다. TTC 1.5초는 현재 독립 estop 발동 자체를 지시하지 않으며, avoidable 표시는 속도/정지 요구와 동일하지 않다.
