# CLAUDE.md — 자율주행 시스템 프로젝트 컨텍스트

> 구형·미사용 상태와 별도 MISSION_PREPARE를 현재 정의에서 제외했다. raw dump는 v35다.
> 현재 확정 상태 머신: **스테이트 v09.16**. [명칭·빈 상태 점검](docs/STATE_V09_16.md)을 현재 명세로 사용한다.

> 2026-09-16 PR #108 후속: 상위 ESTOP에 실제 정차 10초 → 실측 후진 1m → 정차 완료 실행기 연결.
> 독립 EstopRequest/장애물 소실 조기 해제 제외. CAN state=5, raw dump v34.
> [현재 ESTOP 계약](docs/V2_PR108_INTEGRATION.md)이 아래 과거 설명보다 우선한다.

> 2026-09-16 후속 회피 교체: `prepare/drive`는 PR #103의 `waypoint_avoid_node`를 사용한다.
> `waypoint_avoid=true`, `avoid_v2_enabled=false`만 허용한다. 구형 선택 분기·v2 런처·회피 시험 진입점은 제외했다.
> 이전 New_Avoid_v2/legacy 소스는 기록 비교용이며 현재 v2 실행 그래프에는 포함하지 않는다.
> PR #105 분리 CSV와 공통 GPS 원점을 사용하고, 09.16 상위 AVOID 전이 조건은 유지한다.
> 무효 곡률/경로는 points를 비워 정차하며 1m preview를 변경 없이 전달한다.
> [회피 교체 보고서](docs/V2_PR103_INTEGRATION.md)가 아래 과거 회피 설명보다 우선한다.

> 2026-09-16 T 주차 참조경로 통합: 일반 drive 런처는 좌측 LiDAR 판별 후 03 끝까지 전진,
> 선택 경로 후진, 후방 벽 0.50m 정차/10초 대기, 같은 경로 전진 복귀 후에만 done을 보낸다.
> T Mission은 03 종점에서 자동 취소하지 않으며 주차 모듈이 복귀 정차를 확인한 뒤 MGM ACK로 04에 전환한다.
> T 속도는 provider 권장값을 v_base 이하로 보존한다. 평행 주차 기존 정책은 유지한다.
> raw dump v33. [연결·검증 범위](src/stack_parking/docs/T_REFERENCE_SEQUENCE.md).
> 오프라인 검증이며 실차·ROS graph·MPC 폐루프 검증을 뜻하지 않는다.

> 2026-09-15: v2 기본 회피 provider는 전방·좌우 3개 LiDAR를 사용하는 `stack_avoid_v2`다. 기존 zone 진입·GPS_RETURN·완료 표식 조건은 유지한다. 운영 연결은 [벽 중앙 회피 MGM 연결](docs/AVOID_V2_MGM_INTEGRATION.md)을 따른다. 아래 기존 `stack_avoid` 경로 생성/그림자 전용 설명보다 이 연결 계약이 우선한다.

> 2026-09-14 최종 v2 정책: MGM 자체 회피 TTC/경로 실패 AUTO_ESTOP은 사용하지 않는다.
> 독립 LiDAR 요청만 AUTO_ESTOP의 입력이며, CAN/운전자/신호/미션 정지는 별도다.
> 장애물이 없고 회피 제어점이 비거나 오래됐으면 유효 GPS로 즉시 복귀한다.
> 신호 해제는 `!red`이며 GPS 주행 상태로 전환한다. 아래 과거 정책보다 우선한다.
> HTML 실험실 및 관련 도구는 로컬 전용으로 이번 PR에서 제외한다.

> 2026-09-14 검출 거리: 앞범퍼 전방 3.5m 미만, 좌우 ±0.5m. 검출 유지 +0.4m(3.9m).
> 속도 2m/s, 회피점 station+2.7m 복귀, 기존 TTC 기준 유지. 상세 RUN_BOOK_FINAL.md.

> 2026-09-14 회피 목표점: 최근접 거리는 검출·TTC 전용. 관련 연결 면의 앞뒤 범위와
> 원래 끝점·극점에서 후보 단면을 잡고, 선분+여유 원의 합집합으로 실제 점유 구간을
> 계산한다. 면 기준 생성과 GPS station+1 → 회피점 → station+2.7m 연결 계약은
> [현재 경로 문서](docs/AVOID_STATION_PATH.md)가 아래 과거 설명보다 우선한다.

> 2026-09-14 사용자 지정 SAFE_STOP: v2 `safe_stop_all_sensors_only=true`.
> 차선/신호 카메라 2대, raw LiDAR 4대, GPS가 전부 무효일 때만 사유 2로 SAFE_STOP.
> 센서별 freshness를 독립 판정하며, `MgmState.sensor_alive_mask`로 보고한다.
> 운영자/CAN 정지, AUTO_ESTOP 및 제어점 부재 출력 대기(`reference_motion_blocked`)는 별도.
> dump v29. 아래 과거 개별 입력 상실 SAFE_STOP 규칙보다 이 설정이 우선한다.
> 기존 출발 인가 조건은 유지. 상세 `RUN_BOOK_FINAL.md`.

> 2026-09-14 주행 준비 변경: `scripts/v2 prepare`는 전체 스택을 준비하고,
> GPS/RTCM은 별도 상주 서비스로 유지한다. `go`는 카메라 프레임 **또는**
> GPS FIXED(4)로 인가하고, `stop → go`는 GPS를 재시작하지 않는다.
> 일반 CSV 구간은 GPS 상실 시 유효한 카메라 경로로 주행한다. raw dump v26.
> 상세: [주행 준비](docs/DRIVE_PREPARATION.md). 아래 과거 전체 센서 필수 출발 조건보다 우선한다.

> **2026-09-14 회피 station 경로:** 회피는 내부 다점 경로를 유지하고 매 새 스캔에서
> 이전 station ±`(abs(v_ref) * sample_time + 0.5m)` 안으로 투영한다. 출력은 경로
> 누적거리 station+1m의 점 1개이며 MGM은 xy/yaw/curvature를 축소 없이 전달한다.
> 장애물 뒤쪽이 불명확하면 회피 후 직선 꼬리를 유지하고, 신선한 GPS 기준점까지
> 차량 폭·길이를 고려한 연결 경로가 관측상 비어 있을 때만 복귀 경로를 만든다.
> 경로는 신선한 GPS 위치/헤딩 또는 VehicleVector pose에 고정하며, 동일 episode
> 중 좌표 소스를 바꾸지 않는다. 자세 상실 시 경로를 전진시키거나 완료하지 않는다.
> 상세: `docs/AVOID_STATION_PATH.md`. 아래 과거 회피점 1/20 축소 규칙보다 우선한다.

> **2026-09-13 무인 RC 시험 후진:** [현재 설정](docs/RC_REVERSE_RECOVERY.md)이 아래 과거 Recovery OFF/필수 CLEAR/고정속도 설명보다 우선한다.
> 일반 v2 설정은 `escape_after_cycles=1000`, `v_escape=-0.8`, `escape_max_cycles=162`,
> `escape_require_rear_clear=false`. 주차·외부·CAN 정지는 유지하며 no-estop/MBD/bench는 복구 OFF다. raw dump v22.

> **2026-09-13 회피 장애물 표면 모델:** `stack_avoid`는 매 스캔의 인접 반사점을
> 연결된 2D 윤곽(선분)으로 구성한다. 감지·TTC는 윤곽과 전방 통로의 교차로,
> 회피 공간은 깊이 밴드에 걸친 **윤곽 전체**의 측방 점유 구간에 차폭/여유를
> 반영해 계산한다. 깊이·측방 컷으로 벽의 가짜 끝을 만들지 않는다.
> 무효 빔/거리 불연속은 연결하지 않고 고립점도 보존한다. 목표와 rate limit
> 중간점은 연속 선분의 거리·가림을 검사한다. 3D 면 복원이나 가려진 물체의
> 체적 추정은 아니며, 실제 제어 궤적 전체의 충돌 검증은 별도다.
> 단일 목표점/MGM/CAN/완료 계약은 유지한다. 상세: `docs/AVOID_SURFACES.md`.

> **2026-09-13 GPS-only Zone 선행 진입:** 일반 `GPS_ONLY_ZONE` membership은
> 현재 station 최근접 웨이포인트와 station+2.5m preview에 가장 가까운 CSV
> 웨이포인트 중 하나라도 Zone 범위 안이면 true다. preview의 주행 기준점 보간은
> 유지하되 Zone 판정에는 보간 비율이 아니라 가장 가까운 한쪽 웨이포인트 index를 쓴다.
> `MISSION_ZONE`과 주차·정지·회피·가속처럼 동작/state를 바꾸는 나머지 판정은
> preview를 사용하지 않고 현재 station index만 사용한다.

> **2026-09-13 주차 맵 재관측 반경:** 기존 맵 점의 재관측 일치 반경은
> `0.02m`다. 같은 voxel이 아닌 점은 2cm 이내에서만 기존 셀의 hit로 인정한다.
> 4m freespace 삭제 범위와 tentative/confirmed miss 횟수, 빈 bin을 unknown으로
> 취급하는 정책은 변경하지 않는다.

> **2026-09-13 주차 측면 탐색 거리:** 사용자 지정으로 주차공간 검출의 측면 경계
> 최대거리는 T자·평행 모두 `3.0m`다. 통합 `SpaceDetector`뿐 아니라 별도
> `parallel_parking_node`가 상속하는 `WallGapDetector`의 초기 벽 탐색에도 같은
> 기본값을 적용한다. T자 주차의 후면 벽 판정은 이 상한과 분리해
> `perpendicular_min_depth_m` 이후의 지지점을 사용한다. 측면 탐색 상한을 늘려도
> 후면 벽이 반드시 3m보다 멀어야 하는 조건으로 바뀌지 않는다.

> **2026-09-13 회피 수정:** [main 동작 복원 메모](docs/AVOIDANCE_MAIN_RESTORE.md)가 아래 과거 회피 고정속도/소실 200틱 종료 설명보다 우선한다.
> AVOID 첫 CAN 기준점·yaw, .6/.2 m/s 상한, 가감속, 완료/최대 시간 종료 및 종료 후 GPS hold를 복원했다.
> v2 단일점 계약과 병행 Manager는 유지한다. CTest 21/21 통과; 실차 확인은 남아 있다.


> **2026-09-13 현재 통합 설정:** 일반 주행은 회피와 LiDAR E-stop 모두 ON이다.
> Mission ACTIVE에서는 준비·실행 전체에서 둘 다 제외하며 종료 후 다시 적용한다.
> 신호등 OAK 자동 노출 보정 기본값은 `-2`다(차선 카메라와 별도).
> 아래 과거 OFF/노출 설정 기록보다 이 기준을 우선한다.

> 2026-09-13 정면 라이다 좌표 수정: `parking_enabled=true`인 공유 통합 launch는
> E-stop yaw를 `lidar_fusion_v2/config/fixed_geometry.yaml`의 a1 -87도에서 읽고,
> 회피 전방 각도를 동일 yaw에서 +87도로 계산한다. 기존 +90도/270도 해석으로
> 뒤쪽 반사점을 전방 장애물로 오인하던 문제를 합성 스캔으로 재현·수정했다.
> 일반/no-estop에 공통 적용하며 단일 /scan 방향과 거리 문턱은 유지한다.
> 현장 검증은 남아 있다. [재현·검증 기록](docs/FRONT_LIDAR_ALIGNMENT.md).

> **2026-09-13 PR 검토 상태:** [현재 범위·검증 결과](docs/INTEGRATION_V2_PR_STATUS_20260913.md). 이번 MGM 격리 빌드 성공, Python 432 통과/3 skip.
> 전체 CTest는 11/20 통과이며 회귀 정리가 남아 있다. 아래 과거 미빌드/전체 통과 표기보다 이 결과를 우선한다.

> 2026-09-13 주차 제어권 변경: v2 `parking_zone_entry_active=true`는 stable Mission Zone
> 진입(기존 5회 확인) 즉시 MISSION_ACTIVE/PARKING으로 전환한다. 탐색 중에는 현재 CSV의
> GPS 목표점 1개를 v_base로 추종하며 PREPARE 명령으로 SLAM/검출/계획을 준비한다.
> 현재 요청의 fresh ready 틱에 Parking 제어로 인계하고 ACTIVATE를 보낸다.
> 이때부터 실행 ack/유효 Parking reference까지 정지한다. mission_start/handoff도 ready에 기록한다.
> Zone 이탈/시간/탐색 거리로 복귀하지 않는다. 정상 복귀는 현재 요청의 실행 ack 이후 done,
> 또는 유효한 현재 CSV 종점 도달이다. 종점에서 미완료 요청은 ROUTE_END=10으로 CANCEL,
> 실패 기억을 기록하고 기존 실제 정지/새 CSV ack 인계 절차를 따른다. 03 종점까지 미준비이면
> 04 요청·적용·재출발이 자동으로 이어지며 추가 go가 필요 없다. done과 종점 동시면 성공 우선.
> 탐색 중 GPS 상실은 정지이며 높은 LINE 신뢰도로 대체하지 않는다. 일반 신호/외부/CAN
> 정지는 탐색에도 적용한다. 일반 LiDAR E-stop과 회피는 ACTIVE 준비·실행 전체에서 제외한다.
> Parking status 부재 자체는 GPS 탐색을 막지 않는다.
> ready 이후 모듈 응답/경로 상실은 Parking 제어를 유지한 정지다. 명시 취소/새 session/최종 FINISH,
> 운전자·CAN 정지는 유지한다. 이 설정은 과거 parking_search_zone_only/수명 제한보다 우선한다.
> 현행 기본은 즉시 진입 true / 과거 Zone 탐색 false. 과거 시험은 즉시 진입 false를 명시한다.
> 탐색의 TargetRef delta는 GPS, 기동의 delta는 Parking SLAM이다. state byte만으로 고르지 않는다.
> 신호 정지 초기 거리 1.5m는 그대로다. raw dump v20이며 v18/v19 run은 당시 빌드로 재생한다.
> 세부 기준: [MGM_PARKING_ENTRY.md](docs/MGM_PARKING_ENTRY.md).

> 2026-09-13 GPS station 변경: 최초 유효 fix에서만 전역 최근접 index를 찾는다. 이후에는 저장한
> 연속 station ± `(abs(v_ref) * sample_time * 1.5 + 0.5m)` 안의 경로 선분에서만 다음 station/index를 찾는다.
> 거리 기준은 경로 누적 길이다. 연속 station을 함께 저장해 CSV 간격보다 작은 이동도 누적한다.
> sample_time은 GPS `publish_period`(현재 0.1s), v_ref는 `/adas/target_ref`의 최종 명령이다.
> 같은/역행 fix에는 갱신하지 않는다. 명령 미수신·비유한·기존 GPS stale_timeout 초과는 v_ref=0으로
> 탐색 반경을 0.5m로 한다. 정지 중에도 새 fix로 이 범위의 station/index를 갱신한다.
> GPS 공백으로 window를 늘리거나 전역 재탐색하지 않는다.
> CSV 전환/명시적 새 session에서만 station을 초기화한다. GPS-only Zone은 현재 위치의
> 저장 index와 preview 최근접 index의 OR를 사용하고, 그 밖의 Zone은 현재 위치 index만 사용한다.
> preview는 station +2.5m(종점 클램프). 두 점 중 한쪽 가중치가 90% 이상이면 해당 점을 사용하고,
> 나머지는 xy/곡률을 선형 보간하고 yaw는 짧은 각도 방향으로 보간한다. 반환/발행은 1점이다.
> endpoint snap일 때는 +2.5m에서 최대 선분 길이의 10%만큼 달라질 수 있다.
> 기존 GPS 재합류 합성·1.8m 거리·25도 제한은 v2 station 경로에서 사용하지 않는다.
> 과거 PathEngine snapshot은 legacy 비교용이며 ROS 운용은 station 경로를 고정 사용한다.

> 2026-09-12 카메라 목표점: 차량 원점의 차로 중심선 최근접 투영을 현재 station으로 삼고,
> 중심선을 따라 +2.5m 지점의 xy/yaw/curvature **1개만 반환/발행**한다.
> 중심선 피팅·내부 20점 타당성 검사·계수 smoothing은 유지한다. x 고정/원점 직선거리 고정이 아니다.
> 신뢰도·곡률에 따라 preview 거리를 바꾸는 기존 REF_POINT_00 치환은 제거한다.
> 검출 실패/HELD/SEARCH의 기존 유효성·generation 정책은 유지한다. GPS 기하는 이번 변경 범위 밖이다.
> 가시 범위 밖 station은 기존 피팅 곡선의 외삽이며 새로운 실측으로 간주하지 않는다.

> **아래 전체 provider 1점 계약은 진행 중인 설계다.** MGM 소스만 일부 반영된 미빌드 상태이며,
> GPS/LINE 생산부는 1점을 반환하도록 수정·오프라인 검증했다. 나머지 생산부 및 MGM 설치본의 통합은 미완료다.

> 2026-09-12 사용자 요청 단일 목표점: v2 GPS/LINE/Avoid/Parking→MGM→TargetRef→CAN은
> 유효한 제어 목표점 1개를 전달하는 것이 목표다. GPS n_points=1이며 LINE n_points는 내부 검사 표본 수다.
> CSV 원본/Zone/주차 계획은 다점 경로 그대로 보존한다. MGM은 1→20 보간을 하지 않고
> 선택점의 xy/yaw/curvature를 보존한다(기존 소스 전환 블렌드는 1점에 적용).
> 비어 있거나 1개 초과/비유한/기본 원점인 입력은 기존 Reference 정지로 처리한다.
> legacy/generated 재생용 배열 용량 20은 보존하되 병행 Manager 유효 n은 1이다.
> 승인된 Recovery는 기존 1.5m 직선 목표 1개, 활성 설정은 계속 OFF다. dump v18.
> 기존 1→20 기하 보상 지침은 legacy 전용이다. 실제 주행 없이 CSV 전환까지 mock으로 확인한다.

> 2026-09-12 사용자 요청 임시 회피 OFF: v2 YAML/통합 launch의 `avoidance_enabled=false`.
> 병행 Manager는 일반 AVOID 진입·CLEAR_CONFIRM·LiDAR-only fallback과 회피 전용 TTC 정지를 사용하지 않는다.
> LINE/GPS reference가 모두 없으면 정지한다. 인지 노드는 관측/로그용으로 유지하며 별도 LiDAR E-stop,
> 외부/CAN/Reference/Signal/Mission 정지는 각각 기존 설정을 따른다. no_estop 런처의 E-stop 제외는 그대로다.
> 파라미터는 startup-only, 재활성화는 `avoidance_enabled:=true`. CoreParams 추가로 dump는 v17이다.

> 2026-09-12 사용자 요청: v2 병행 Manager의 비정지 목표속도 크기는 `v_base` 하나로 고정한다.
> 현재 YAML은 1.0m/s이며 주차/승인된 Recovery 후진은 -v_base다. provider의 0 정지 요구는 보존한다.
> 가속 Zone, 회피 권장속도 크기/v_avoid/v_narrow 및 일반 주행 가감속 ramp는 적용하지 않는다.
> Signal 정지 profile/정지 감속, TTC·외부·CAN·Reference 정지 및 제어권 인계의 0은 유지한다.
> legacy/generated 경로는 기존 속도 정책이다. Bus layout은 같지만 재생 속도 계약 구분을 위해 dump는 v16이다.
> 차량 실행 없이 core 회귀로 확인한다. 과거 run은 해당 빌드로 재생한다.

> 2026-09-12 사용자 요청 시험 구성: `REAL_VEHICLE_integration_v2_no_estop.launch.py`는
> `stack_estop` 노드와 해당 LiDAR E-stop 입력/미수신 보정만 제외한다.
> MGM의 startup-only `lidar_estop_enabled=false`는 병행 core, wait_go=true,
> escape_after_cycles=0에서만 허용한다. 일반 런처와 MGM 기본값은 true다.
> 운전자 정지, CAN fault/latch, Reference final gate, Signal, Mission, 회피 TTC 정지는 유지한다.
> 이는 전방 좌표 불일치 수정이나 실차 안전 검증을 뜻하지 않는다. 생성 backend/core bus를 바꾸지 않으며,
> dump에는 선택한 시험 모드로 conditioning된 입력을 기록한다. 작성·검증 중 실제 차량 런처/go는 실행하지 않는다.
> 실행과 제외 범위: [docs/INTEGRATION_V2_NO_ESTOP.md](docs/INTEGRATION_V2_NO_ESTOP.md).

> 2026-09-12 현장 라이다 복구: v2의 통합 Parking launch는 `lidar_fusion_v2/drivers.launch.py`를 사용한다.
> 기존 multi_lidar 드라이버 런처를 경유하면 a1에 9K/16bit가 전달된다. v2에 이미 검증된
> 네 대의 공통 4K/8bit 프로필을 단일 기준으로 연결한다. 장착 변환·FOV·차량 제어는 변경하지 않는다.

> 2026-09-12 주차 탐색 정책 변경: v2 `parking_search_zone_only=true`.
> PREPARE는 요청을 시작한 stable Mission Zone 안에서만 ready→ACTIVE를 허가한다.
> 확정 이탈(현재 5회)은 ZONE_EXIT=9로 실패 기록·CANCEL하며 현재 CSV의 다음 점부터 일반 주행을 계속한다.
> 실패는 성공 완료와 별도 Mission ID 기억이다. 같은 session에 재시도하지 않으며 CSV 종점에서는
> 해당 실패를 종료된 Mission으로 취급한다. 실패 순간 다음 CSV로 건너뛰지 않는다.
> 시간/거리 제한은 사용하지 않는다. elapsed/travel은 관측 기록만 유지한다. GPS/Zone 불명은 PREPARE 정지 대기다.
> ACTIVE 주차는 기존 제어권·완료 조건을 유지한다. 기존 외부 정지·명시 취소·입력 오류 처리는 유지한다.
> false는 이전 시간/거리 제한 회귀용이다. 새 실패 기억/정책 bus로 raw dump는 v15다.

> 2026-09-12 사용자 설정: v2 Zone 진입/이탈 확인은 각각 독립 GNSS **5회/5회**다.
> params.yaml을 통합 launch 기본값의 기준으로 사용한다. 실측 인증값이라는 뜻은 아니다.
> Zone 기준 탐색에서는 Parking 시간/거리 제한을 쓰지 않는다. 설정 파일 없이 MGM 단독 기동 시 Zone 확인 0=미설정 계약은 유지한다.

> 2026-09-12 순서 정정: 한라대는 **(01 또는 02)→03→04→05→(06 또는 07)**이다.
> `route_start_id`/`route_end_id`는 기본 미설정이며 실행 전에 사용자가 둘 다 명시한다.
> 시작 때 선택한 5개 CSV로 순서를 확정하고 run 중 변경하지 않는다. 01→02/06→07 연결은 사용하지 않는다.
> 출구 모델의 인식 결과를 경로 선택에 연결하는 runtime은 현재 v2에 없으며 임의 class→경로 매핑을 만들지 않는다.
> Mission 완료·실제 정지·새 reference 응답 조건은 유지한다. 선택 순서도 sequence hash에 포함한다.
> 이번 선택은 GPS manifest 해석 단계이며 선택과 탐색 정책의 현재 bus/raw dump는 v15다.
> 현재 사용자가 지정한 실차 run은 `route_start_id=01`, `route_end_id=07`이다.

> 2026-09-14 한라대 데이터: 손상민 PR #98 / `8796cdd`의 Path 4 state=4(idx 55),
> 1m 단축 종점 `(-63.042107,-71.940893)`과 GPS-only 구간 끝점을 사용한다.
> CSV state 1/2 주차점과 state 3 신호 예상 위치의 기존 역할은 유지한다.
> v2 기본 `avoid_zone_only=true`: state=4/GPS 회피 시작 구간 확인으로 AVOID_ACTIVE 진입,
> 실제 회피 후 waypoint 복귀 완료 및 GPS 0.1m/20° 정렬로 종료한다.
> 무검출·타이머·신호 해제로 회피를 끝내지 않으며 같은 표식은 재실행하지 않는다.
> [회피 Zone 상태 전이](docs/AVOID_ZONE_ENTRY.md)가 이전 회피 시작·종료 정책보다 우선한다.

> Integration v2 분리 기준: [docs/INTEGRATION_V2.md](docs/INTEGRATION_V2.md). 기존 main은 보존하며 v2 전용 소스/build/install에서만 통합한다.

> 2026-09-12 연속 경로: [docs/MGM_ROUTE_SEQUENCE.md](docs/MGM_ROUTE_SEQUENCE.md).
> 한라대는 사용자 지정 시작/종료 경로와 공통 03→04→05 순서다.
> 명시적 manifest의 순서와 endpoint_and_missions/missions_complete 조건을 MGM 순수 core가 관리한다. 중간 종점 정지 + 해당 경로 Mission 완료 +
> 다음 경로/새 GNSS reference 응답 후 자동 재개하며 마지막 경로만 FINISH다. GPS는 검증·선로딩과
> 명령 적용을 맡는다. 단일 CSV 동작과 경로 생성식은 유지한다. 새 bus/dump는 v15이다.

> **6차 기준:** [MGM_MBD_STATE_MACHINE_SPEC.md](docs/MGM_MBD_STATE_MACHINE_SPEC.md)가 현재 C++ 병행 Manager와 향후 MBD의 단일 명세다. 아래 3/4/5차와 legacy 설명은 이 명세에 종속한다.
> Zone은 독립 GNSS generation으로 확인하며 확인 표본 수는 0=미설정이다. 정의된 Zone의 확인 기준 미설정은 ZONE_CONTEXT_UNAVAILABLE 정지로 노출한다.
> Parking 제한 -1 및 Recovery OFF 유지. Traffic은 앞범퍼 잔여거리 1.5m 최초 seed, 실측 속도 절댓값 적분, 목표 1.0m다.

> 2026-09-11 5차: Mission Zone entry는 제어권 획득이 아닌 탐색 요청이다.
> [MGM_MISSION_PREPARATION.md](docs/MGM_MISSION_PREPARATION.md)를 우선한다.
> IDLE → PREPARE(일반 병행 주행 유지) → 현재 요청의 Parking ready → ACTIVE.
> Zone exit는 요청을 취소하지 않는다. 시간/거리 제한은 실차 calibration 전 미설정이며,
> 미설정 요청은 명시적으로 취소하여 무제한 탐색을 허용하지 않는다.

> 2026-09-11: `feat/state-machine`의 C++ MGM 공통 베이스 재구성은
> 4차 안전 계층 [MGM_REFERENCE_SAFETY.md](docs/MGM_REFERENCE_SAFETY.md)를 함께 적용한다.
> 실제 센서 입력에 근거한 Reference 생성 시각과 발행 시각을 분리한다. 기존
> provider별 timeout을 재사용하며 invalid/stale 선택 경로는 SAFE_STOP reason으로 정지한다.
> Mission/Avoidance 제어권을 유지하고 최종 송신 전 유효성을 다시 검사한다.
> Recovery OFF와 rear_clear 미연결 상태는 임의 변경하지 않는다.
> [MGM_BASE_STATE_MACHINE.md](docs/MGM_BASE_STATE_MACHINE.md)를 우선한다.
> 3차 수정: Mission은 MISSION_ZONE의 실제 포함 여부 edge로 시작한다.
> waypoint reached 토픽/ID trigger는 제거한다. GPS/주차 구간 정의는 stack_gps 설정을 재사용한다.
> Navigation/Avoidance/Traffic/Safety/Mission을 병행 관리한다. 아래 단일 5상태·
> 회피 중 신호 무시·estop에 의한 종점 해제 규칙은 legacy/generated v1.88 경로에만 해당한다.

> 이 파일은 팀의 아키텍처 설계 결론을 담는다. Claude Code는 모든 세션에서 이 문서를 프로젝트의 기준으로 삼을 것.
> 설계 변경은 반드시 이 문서를 갱신한 뒤 코드에 반영한다.

> ## ★ 세션 시작 시 먼저 확인할 것 — [HANDOVER.md](HANDOVER.md)
>
> 2026-08-24 에 팀이 **이기돈(팀장)·손상민** 두 명으로 줄었고, 실행 PC 도 김윤기 노트북에서
> 각자 PC/산업용 PC 로 옮겨간다. `HANDOVER.md` 에 **새 PC 세팅 순서·운용 함정·남긴 미완
> 작업**이 정리돼 있다.
>
> 사용자가 "인수인계 확인해줘", "이 PC 에서 돌리려면 뭐가 필요해?", "왜 RTK 가 안 잡혀?",
> "빌드가 안 돼" 같은 것을 물으면 **추측하지 말고 그 문서를 근거로 답할 것.**
> 이 문서(CLAUDE.md)는 *설계가 왜 이런가*를, HANDOVER.md 는 *어떻게 세팅하고 돌리는가*를 맡는다.

## 1. 프로젝트 개요

WHEELTEC 플랫폼 기반 자율주행 시스템. 시나리오: 차선 주행, GPS(waypoint) 주행, 장애물 회피, 신호등/정지선 정지, 돌발 장애물 긴급 정지, 라이다 주차.

- 상위: 산업용 PC, **Ubuntu 22.04 + ROS 2 Humble** — 인지(Signal processing) + 판단(Decision)
- 하위: dSPACE — 제어(Control) + 구동(Actuation)
- 통신: **CAN FD (ISO, nominal 1 Mbps / data 2 Mbps, BRS on, PC 측 Kvaser Leaf v3)**, 10ms 주기
  — dSPACE 측 Ethernet 사용 불가로 UDP에서 CAN 전환 (2026-07-29), classic 2.0A + PCAN 에서 CAN FD + Kvaser 로 재전환 (2026-08-28).
  같은 날 오후 **v5(PR #52, 손상민)로 재전환** — 64바이트 페이로드 + 참조점 1개. dSPACE 가 먼저 그 계약으로 넘어가 있어 PC 를 맞췄다(팀장 결정).
  `0x100` 헤더만 8바이트로 남고 `0x101`·`0x200` 이 64B float64 가 되며, **RX 가 1프레임이 되어 커밋(latch) 규칙이 소멸**했다. `str_ref`(MPC 명령 조향각)가 신규로 온다.
  ⚠ **참조점 1개는 측정된 위험을 알고 내린 결정** — §3 ② 참조. 단일 진실 원천은 `bridge_dspace/PROTOCOL.md`.

## 2. 아키텍처 (v1 — 현재 기준)

계층 모델: **Signal processing → Decision → Control → Actuation**

| 계층 | 이름 | 위치 | 주기 | 내용 |
|---|---|---|---|---|
| Signal processing | ADAS application | PC | 비동기 (각 센서 주기) | 차선 검출(camera 100ms), 신호등·정지선 인식(camera), GPS·IMU 융합(위치·헤딩), 장애물 인지·주차 로컬맵(LiDAR 100ms) |
| Decision | ADAS MGM | PC | **10ms 고정** | 주행 모드 스테이트 머신 + 요구 생성 + ref points 조립 + 종방향 병합 → target ref 확정 → CAN TX |
| Control | Vehicle MGM | dSPACE | 10ms | 통신 watchdog → 궤적 생성(quintic, feasibility) → MPC(횡·종 통합, 예측 지평 200ms/N=20 → str_ref, v_ref) → 상태 추정(kinematic bicycle model) |
| Actuation | 하위 제어 | dSPACE | **5ms 독립 태스크** | 엔코더 읽기 → PI → PWM 갱신 (구동 20kHz, 조향 서보 50Hz) |

- dSPACE는 multi-rate: 10ms 태스크(Vehicle MGM)와 5ms 태스크(Actuation)가 한 프로세서에서 병행. **주기는 합산되지 않음** — 10ms 틱 안에서 수신→궤적→MPC가 연쇄 실행.
- Actuation은 상위와 무관하게 항상 5ms로 돌며, 최신 목표값(str_ref, v_ref)을 hold하여 사용.
- 대안 v3: ROS 2 실시간성 검증 실패 시 ADAS MGM을 통째로 dSPACE로 이관 (§7 판정 기준 참조). 로직은 동일, 배치만 변경.

## 3. 통신 계약 (CAN, PC ↔ dSPACE) — 최우선 구현 대상

물리 계층은 CAN (classic, 8바이트/프레임)이므로 논리 계약을 여러 CAN 프레임으로 분할한다.
**CAN ID 맵·양자화 스케일·프레임 레이아웃의 단일 진실 원천은 `src/bridge_dspace/PROTOCOL.md`** — dSPACE 측(손상민)과 반드시 그 문서로 합의.

**TX (PC → dSPACE, 10ms, n_points+1 프레임 — 유효 점만 송신):**

| 필드 | CAN 매핑 | 설명 |
|---|---|---|
| ref_points[n] | **v5: `0x101` 1프레임, 64B float64 무양자화** (구 v3: `0x101`~`0x114`, 점당 1프레임 int16 양자화 1mm / 1e-4rad / 5e-4 1/m) | **vehicle frame** — 생성 시점 차량 위치 = (0,0,0). 모든 모드 동일 포맷. **⚠ 2026-08-17: dSPACE 조향에 PI가 추가되어(손상민) 응답 속도·수렴이 빨라졌다. 아래 ①②③의 실현율 수치는 전부 PI 이전 측정이며, 그에 맞춰 넣은 보상 기하(GPS 재합류 3종·avoid 20점 보간 첫 점)는 그만큼 과이득이 된다** — 실제로 GPS 구간에서만 오실레이션으로 나타났다(run_0817_011045: 같은 run의 차선 구간 대비 명령 곡률 3배·틱간 방위 흔들림 7배·실제 요레이트 3배). 1차 조치로 GPS 재합류의 ψₑ 변화율 감쇠항을 껐다(`REJOIN_RATE_DAMP_S` 0.6→0.0 — 그 항의 존재 이유가 "조향이 1초 늦게 반응한다"였고 PI가 그 지연을 없앴다). **2차 조치 (2026-08-17, run_0817_020112 dSPACE 병합):** 체인을 두 단으로 갈라 재니 **서보 PI 루프는 이미 회복됐다** — `실제δ/명령δ` = 차선 79.9%·GPS 91.7%·회피 107.4%(지연 160~180ms). 남은 손실은 MPC단(`명령δ/PC 기하δ` 43~59%)이고, **그마저 우리 잡음에 깎인 값**이다: 저역통과 폭을 늘리면 차선은 49%로 평평한데(진짜 플랜트 이득) GPS만 54→70%로 오른다 = GPS 명령의 1/4은 차가 못 따르는 고주파다. **즉 "ref[0]을 짧게 → 잡음↑ → 측정 실현율↓ → 더 짧게"는 잘못된 방향의 양의 되먹임이었다.** GPS ref 잡음의 증폭단 3개를 손봤다(stack_gps): ⓐ `cross_track_m`이 트랙 **선**이 아니라 최근접 **꼭짓점**까지 거리라 웨이포인트 간격이 주기 0.583s·진폭 0.081m 가짜 톱니를 만들던 것 → 선분 투영 수직거리(`_foot_on_track`), ⓑ 접근각 α가 쓰는 e에 종방향 성분이 누설(≈offset×sin ψₑ)하던 것 → 같은 수선의 발 사용, ⓒ ref[0] 거리의 **81.7%가 기하 하한 1.267m에 고착**(κ=2sin b/d 라 차선 2.04m 대비 1.61배 증폭) → `rejoin_target_min_m` 기본 1.8. 추가로 Δα의 54%가 백색잡음(sd 1.26cm, RTK 위치)이라 **e에만** 1차 저역통과(`rejoin_e_lpf_s` 0.15). **ψₑ에는 절대 걸지 말 것** — Δψₑ의 lag-1 자기상관이 +0.002로 백색 성분이 0이라(= 진짜 요운동) 필터가 실동작만 죽인다. 같은 run 재생 결과 Δ기하δ 1.41°→0.77°, p95 4.75°→2.99°(차선 0.39°/1.08°). **ⓒ는 실차 미검증** — 절차는 `RUNBOOK_avoid_field_test.md` §5-1b. **③의 실현율 곡선을 재측정하기 전까지 avoid 보상 기하는 그대로 둔다** — 절차는 `adas_mgm/RUNBOOK_avoid_field_test.md` §5-2. **조향 응답의 실전 제약 3가지 (2026-08-15 정리):** ① **v_ref ≥ 0.5 m/s** — 이기돈 검증: 0.5 이상이어야 조향이 제대로 반응 (구 0.4 기준을 상향). **어떤 스테이트도 "감속하면서 조향"을 설계하지 말 것** — 감속이 조향을 죽인다 (run_0812_234253: avoid 중 v_suggest 감속 0.44 → 직진 → estop). **정량 재현 (2026-08-15, run_0815_155344)** — 곡률 실현율이 속도로 갈린다: `0.30~0.45 m/s 9.7% | 0.45~0.50 m/s 14.0% | 0.50~0.55 m/s 23.9%`. **하한을 목표속도로 쓰면 안 된다** — v_base를 0.5로 두니 달성 속도가 중앙값 0.494, 주행 시간의 69%가 0.5 미만으로 깔렸다. 그래서 `v_base`·`target_speed_mps`를 **0.6으로 상향**(두 값은 항상 동일 유지 — 후자가 AVOID의 v_suggest 소스).
**2026-08-18: 차선·GPS만 1.0으로 재상향, 회피는 `v_avoid` 0.6으로 분리.** 2026-08-17의 1.0 시도(§5-1c)가 무너진 원인은 속도가 아니라 **회피 트리거 빈도**였고(진입 1회 = 12초 고정 × `detect_range` 3.0→5.0 상향), 이번엔 detect_range를 건드리지 않고 회피를 **구간 게이트**로 묶었다. 같은 덤프 재생 검증: 스테이트 시퀀스·정차 구간·immediate_stop 틱 **완전 동일**, v_ref만 차선 0.573→0.949 / gps 0.420→0.690 / 회피 0.546→0.549(진입 램프 0.25s). **속도와 세트로 반드시 함께 바꿀 것**: `ttc_stop` 0.8→1.3, estop 문턱 0.70/0.80→**1.20/1.35**(1.0 m/s 정지 필요거리 1.09m), dynamic 1.20→1.35. ⚠ 아직 남은 위반: `v_narrow` 0.2(narrow_gap 감속)는 이 규칙과 정면 충돌하며 같은 run의 10.2%에서 발동했다 — 이기돈과 정책 결정 필요. ② **점 수 — ⚠ 2026-08-28 에 v5(PR #52)로 1점이 되었다 (팀장 결정).** 아래 20점 근거는 **폐기가 아니라 미해소 위험으로 남는다**: 1점에서는 "첫 점을 목표의 1/20 에 두어 κ 를 부풀리는" 보상이 구조적으로 존재할 수 없으므로 **회피 거동을 실차에서 반드시 재확인할 것**(절차: `PROTOCOL.md` TX 절 경고, `RUNBOOK_avoid_field_test.md` §5-2). `dx`/`dy`/`dyaw` 가 그 정보를 나르도록 정의되면 재검토한다. 이하 v3 까지의 근거 — 실운용에서 검증된 포맷은 20점뿐(gps). 단일 목표점 소스(avoid 1점 계약)는 MGM 조립이 원점→목표 직선 보간으로 20점화해 송신, 인지 스택 계약은 불변. 참고: run_0812_234253의 무반응은 1점+v_ref 0.44가 **중첩**이라 단독 원인 분리 안 됨 — 8/8 "1점 무반응" 실측도 있으므로 둘 다 지킨다. ③ **ref[0]을 멀리 두지 말 것 — 2.5m 넘으면 조향이 죽는다** (2026-08-15 실측, 6런). dSPACE는 **명령 곡률의 10~55%만 실현**하며 그 비율이 첫 점 거리에 좌우된다: 0.8~1.6m에서 ~55%, **2.5m 초과 시 10% 아래로 붕괴**(명령 R 4m ↔ 실제 R 20~50m). 같은 방위 25°·같은 v_ref 0.5인데 거리만 달랐던 두 회피 복귀 — run_0815_142817(d 1.28~1.62m) `str` −0.21, 2초에 24° 선회, cross 0.43→0.10 **성공** / run_0815_144142(d 2.9~5.4m) `str` −0.014, 선회 없음, cross 1.88→5.04 **발산**. 그래서 GPS 재합류 목표는 1.27~1.8m로 클램프한다(`REJOIN_TARGET_MAX_M`) — "이탈이 크니 목표를 더 멀리"는 조향을 죽이는 양의 되먹임이다. **곡선 대응 2종 (2026-08-18 신설, stack_gps):** run_0818_180614에서 곡률 0.36(R 2.8m) 커브 진입 시 횡오차가 0.05→2.55m로 발산했다. 재생으로 갈라 보니 원인이 둘이었다. ① **선행 보상 부재** — 명령 방위 b가 ψₑ를 뒤따르기만 해서(idx 344~348: cross 0.09m인데 b가 ψₑ −8~−16°를 그대로 복사) 트랙이 휘는 만큼 미리 꺾지 않았고, ψₑ가 idx 353에서 −56°까지 쌓인 뒤에야 반응했다. ② **방위 상한 25° 포화** — idx 350부터 b가 −25.0°에 고착, d=1.8m라 κ 천장이 2·sin25°/1.8 = 0.469이고 dSPACE 실현율 43~59%를 곱하면 실제 R 3.5~5m로 트랙의 R 2.8m를 물리적으로 못 돈다. 그래서 ⓐ 트랙 곡률의 선행 보상 `b_ff = asin(d·κ/2)`를 b에 더하고(`rejoin_curve_ff`, 기본 1.0), ⓑ 곡선에서 목표를 당겨 같은 25°로 더 큰 곡률이 나오게 한다(`rejoin_curve_margin` 기본 2.0 → |κ|>0.235부터 당기기 시작, 기하 하한 1.267m에서 멈춤). **직선(κ≈0)에서는 두 항 모두 0이라 2026-08-17 잡음 대책이 그대로 보존된다** — 같은 run 재생에서 직선 구간 갱신당 Δ방위 중앙 0.217→0.231°, 커브 진입에서는 명령 곡률 1.3~7.4배(0.088→0.222, 0.276→0.469, 포화 구간 0.469→0.667). **실차 미검증** — 되돌리려면 `ros2 param set /stack_gps_node rejoin_curve_ff 0.0`.

**⚠ 반대 방향으로 "정직하게" 고치지 말 것:** avoid 20점 보간이 첫 점을 목표의 1/20(≈7.5cm)에 두어 κ=2y/L²을 10~20배 부풀리는 것은 **결함이 아니라 위 10~55% 미실현을 메우는 보상**이다. 2026-08-15에 이걸 1.2m로 "정상화"했다가 같은 회피 목표(|y|≈0.29m)에서 헤딩 +13.0°→+4.3°, `str` 0.089→0.047, 횡변위 0.26m→0.08m로 반토막 나 **콘 회피 실패·estop**했다(run_0815_153633·153456, 즉시 복구). **근본 해결은 dSPACE MPC 쪽** — 예측 지평 200ms × 0.5m/s = **0.1m**라 먼 목표를 구조적으로 못 본다(손상민 확인 필요). 그전까지 보상 기하를 건드리지 않는다. **§5.5 이중 트랙: Simulink 모델도 동일 반영 필요** |
| v_ref | `0x100` 헤더 프레임 (int16, 1mm/s — v5 에서도 8B 그대로) | 종방향 병합의 최종 목표 속도. 정지 = v_ref 0 (별도 정지 명령 없음) |
| flags | `0x100` 헤더 프레임 (state u8 + n_points u8 + counter u16) | **counter는 watchdog 필수 입력** |

- 헤더(`0x100`)는 매 주기 **마지막에** 송신 — dSPACE는 이 프레임에서 n_points개 세트를 원자적으로 latch (반쯤 갱신된 세트 방지).

**RX (dSPACE → PC, 10ms, 3프레임):**

| 필드 | CAN 매핑 | 설명 |
|---|---|---|
| vehicle_vector | **v5: `0x200` 1프레임 64B (f64, 커밋 규칙 소멸 — 수신 즉시 퍼블리시)** (구 v3: `0x200`~`0x202` f32, `0x202`가 커밋) | 상태 추정 결과 {x, y, yaw, v, str, **str_ref**} — PC의 localization 보정에 사용. `str_ref`(v5 신규)로 실현율을 직접 측정할 수 있다. **모든 스테이트에서 상시 송신 (parking 중에도 유지** — 주차 로컬맵·경로 추종의 입력**)** |

**watchdog (dSPACE):** `0x100` 헤더의 counter가 30ms(TX 3주기) 동안 미갱신 → v_ref = 0 (감속 정지), 조향은 직전 값 유지(급조향 금지). 타임아웃 값은 §7 검증 결과에 따라 조정 가능(예: 50ms). point 프레임 수신 여부는 watchdog 판정에 쓰지 않는다.
**dSPACE 로그 정합 (2026-08-17 확립, 2026-08-28 근거 규명):** dSPACE의 counter는 **PC 헤더 counter의 에코**라서(손상민 확인+실측) dSPACE가 `Out1.counter`를 로깅하면 `bag_index = counter − off` 로 **틱 단위 정확 정합**이 된다 (run_0817_020112: off=42, `target_speed`(km/h) vs PC `v_ref` 21300/21300 일치, CAN 유실 0). dSPACE 단위는 **속도 km/h · 조향 deg**, 조향 **부호는 PC ref y와 반대**(규약, 손상민 확인 필요). 도구는 `adas_mgm/tools/dspace_merge.py`, 측정 변수 목록은 `bridge_dspace/DSPACE_LOGGING.md`.
**왕복 지연 = 2틱(20ms), 지터 0 (2026-08-28 실측, 500샘플):** 같은 시점의 `PC TX counter − dSPACE RX counter`가 왕복 틱 수다. §7 실시간성 판정과 watchdog 타임아웃(30ms) 선택의 실측 근거. ⚠ 에코이므로 PC가 송신을 멈추면 이 값이 고정된다 — dSPACE 생존 판정에 쓰지 말 것(그건 프레임 도착 여부로).

**구현됨 (2026-08-31, 손상민 — 이슈 #49):** counter 가 **3회 이상 동일**하면 v_ref 와 MPC 를 정지한다. v_ref 가 0 일 때도 MPC 가 정지하도록 함께 설정됐다. 구 상태(2026-08-09 실측, 손상민 측 J-6)는 "미구현이라 PC 송신이 끊겨도 마지막 v_ref 를 무기한 유지" 였다. ⚠ **실차 미검증** — 아래 세 가지는 아직 눈으로 확인하지 못했다: ① 발동 시 `str_ref` 가 직전 값을 유지하는가(0 중립으로 풀면 코너에서 이탈한다) ② 부팅 직후 첫 수신 전 상태가 fault 인가(PC 없이 부팅한 차가 잔류 목표값으로 움직이는 것 방지) ③ 타임아웃이 튜너블인가. 확인 전까지 실차 launch 의 `can_zero` 가드를 그대로 유지한다 (stack_avoid `launch_parts.can_bridge_with_zero_guard`) — 가드는 watchdog 과 독립이고 겹쳐도 무해하다.

**버스 부하:** 전 스테이트 5프레임/10ms (TX 2 + RX 3) ≈ 68 kbit/s → 1 Mbps에서 ~7%, 500 kbps에서도 ~14%로 여유.

## 4. 스테이트 머신 (Decision 핵심 — 상세: `docs/state_machine_detail.drawio`)

스테이트 5개: **lane · waypoint · avoid · parking · traffic**. 판단 로직은 시스템 전체에서 이 스테이트 머신 한 곳에만 존재한다.

**전이 조건:**

| 전이 | 조건 |
|---|---|
| lane → waypoint | 차선 신뢰도 < 임계, N주기 연속 **또는 GPS 전용 구간 진입**(`GpsPath.gps_only_zone` — 2026-08-18. 진입은 **즉시**, 히스테리시스 없음: 차선을 못 믿겠다고 사람이 지정한 구간이라 신뢰도가 높게 나오는 동안 기다릴 이유가 없다) |
| waypoint → lane | **GPS 전용 구간 밖**(2026-08-18) AND 차선 신뢰도 > 복귀 임계, N주기 연속 (히스테리시스) **AND 트랙에 실제 재합류**(`GpsPath.cross_track_m` ≤ `lane_entry_max_cross_m`, 기본 0.5m) **AND** avoid 복귀 후 `avoid_return_hold_cycles`(기본 300틱=3s) 경과. 재합류 조건이 없으면 회피로 이탈한 채 카메라로 넘어가 **GPS 트랙 복귀를 영영 못 한다**(2026-08-14 실측: 복귀 시점 횡오차 0.90m·2.72m) |
| lane·waypoint → avoid | 장애물 감지 AND 회피 가능 (TTC·측방 여유 충분) **AND 회피 허용 구간 안**(`avoid_zone_only`를 켠 경우 — 2026-08-18. `GpsPath.avoid_zone`이 false면 전이하지 않는다. 시험 코스에서 회피를 지정 구간에만 쓰기 위한 운용 스위치이며 기본값은 끔=구동작. **launch 기본값도 끔이어야 한다** — 2026-08-19~08-25 실차 launch 2종이 이것을 `true` 로 기본 탑재해 **원주 전용 선택이 전 코스 기본**이 돼 있었고, 한라대 MBD 시험에서 회피가 통째로 막혔다(run_mbd_0825_162752: `avoidable` 1.49s 인데 구간 미지정으로 진입 차단 → 직진 → estop). 08-25 복구. 구간 제한이 필요한 코스에서 **인자로 켠다**. ⚠ 켜면 구간 밖 장애물의 방어선은 stack_estop뿐이다 — MGM의 TTC 안전 바닥은 AVOID 스테이트 안에서만 걸린다. **기동 중에는 관여하지 않는다**: 구간을 벗어났다고 회피를 중도 포기하면 장애물 옆에서 트랙으로 되꺾는 꼴이라, 이탈 상한은 `avoid_max_cycles`가 따로 지킨다) |
| lane·waypoint → avoid (**후진 탈출**) | **실제** estop이 `escape_after_cycles`(기본 0=끔, 실차 1000틱=10s) 연속 유지 AND 주행 무장(한 번이라도 명령 속도 > 0) AND 후방 여유(`escape_require_rear_clear` 켬이면 `EstopRequest.rear_clear`) — 2026-08-24 신설. 회피 불가 장애물 앞의 **교착 탈출**이며, 진입 후 AVOID 안에서 후진 페이즈로 돈다 (§4 아래 항목) |
| avoid → 복귀 | 기동 완료 → **waypoint로 복귀** (2026-08-12 개정, 팀장 — 회피 기동 직후 차는 차선을 벗어나 있어 차선 검출을 곧바로 신뢰할 수 없다. GPS 트랙으로 재합류 후 차선 신뢰도 회복 히스테리시스로 lane 자동 재전이. 구 "진입했던 스테이트로 복귀(복귀처 변수)"는 폐기. **§5.5 이중 트랙: Simulink 모델(김재민)도 동일 반영 필요**) |
| lane → parking | GPS 주차구간 AND 주차공간 인식 |
| parking → lane | 주차 완료 |
| lane·waypoint → traffic | `TrafficStop.red_active=true AND stopline_detected=true`(적색 투표 확정 + 최근 3/5 안정 정지선 segmentation). depth는 진단용 거리이며 진입 게이트가 아님. 둘 중 하나만 성립하면 진입하지 않음 (PR #78, 2026-09-04) |
| traffic → lane·waypoint | `TrafficStop.green_active=true`(초록 투표 확정). 적색/정지선 거리 래치를 폐기하고 TRAFFIC 진입 전 주행 상태로 즉시 복귀 |

주차구간은 기존 launch 인덱스 범위뿐 아니라 트랙 옆 `zones_<이름>.yaml`의
`parking_points`로도 지정할 수 있다. 항목은 장소가 재측량 뒤에도 유지되도록
위경도와 `mode: perpendicular|parallel`을 저장하고, `stack_gps`가 기동 시 현재
트랙의 인덱스 범위로 변환해 `GpsPath.parking_zone/parking_mode`만 발행한다.
주차 진입·종료 판단은 계속 MGM과 `stack_parking`의 책임이다.

**스테이트별 우선권 (매 10ms, 스테이트 내부에서 결정 — 전역 min/max 규칙 금지):**

- **lane · waypoint** — 종방향: 긴급 정지 > 신호등 정지(**2026-08-31 이후 TRAFFIC 스테이트로 이관** — 아래 "traffic" 항목 참조. 여기 남은 `traffic_stop_required` 즉시-0 분기는 `traffic_state_enabled=false`(생성 v1.88 백엔드 등 TRAFFIC 미지원 시)일 때만 쓰이는 안전망이다. traffic_state_enabled가 켜져 있으면 `traffic_entry`가 이보다 먼저 상태를 TRAFFIC으로 옮기므로 이 분기는 사실상 도달하지 않는다) > 트랙 종점 정지(**lane·waypoint 공통** — `GpsPath.at_end`. 종점 통과 시 ref가 차량 뒤로 가 유턴하는 구조를 방지, 2026-08-03 직선 run에서 정지 작동 실증. **2026-08-15에 "waypoint만"에서 확장**: GPS 트랙의 종점은 코스의 끝이지 특정 스테이트의 사정이 아니다. lane에서 종점을 지나면 아무도 세우지 않던 구멍이 실차에서 터졌다 — run_0815_163614: t=192.75에 at_end가 섰는데 스테이트가 lane이라 v_ref 0.60 유지, **7.41m를 더 달린 뒤** 차선 신뢰도가 떨어져 waypoint로 내려온 t=206.09에야 정지(13.3초 지연, 그 사이 횡오차 0.41→6.37m + 트랙 밖에서 AVOID 진입까지). **래치 설정 조건**: `gps_path`가 유효할 때만(`n > 0`) — lane 스테이트에서는 §5.7 ②의 gps 신선도 watchdog이 동작하지 않아 낡은/무효 at_end로 래치가 걸릴 수 있다. parking은 제외(그 구간엔 트랙 종점이 무의미). **래치 유지**: 정지 후 밀림·수동 이동으로 at_end가 풀려 재출발하지 않도록 방어적 유지, **실제 EstopRequest의 estop 인가 시에만** 해제 — §5.7 watchdog이 staleness 보정으로 강제한 estop으로는 해제되지 않음(스냅샷에 `estop_latch_release` 별도 필드, 2026-08-11: gps_path 0.5s 단절→복구 시 래치가 풀려 재출발 가능하던 구멍 차단)) > 역방향 정지(waypoint만 — |ref[0].yaw| > 120°가 0.5s 지속 = 차가 경로를 등짐. 유턴 후 트랙을 거꾸로 재추종하는 것 차단, 2026-08-03 2회 재현. **2026-08-16에 래치로 전환 + 헤딩 신뢰 게이트 추가**: `GpsPath.heading_source`가 `HEADING_TANGENT`(접선 폴백)이면 판정을 **세지도 풀지도 않는다**. 접선 폴백은 "최근접 트랙 접선 = 차량 헤딩" 가정이라 ref[0].yaw가 항상 0 부근으로 나와, 차가 트랙을 등지고 있어도 **정렬된 것처럼 보인다**. run_0816_184505에서 그 구멍이 왕복 고착을 만들었다 — 회피 중 130° 돌아버린 차가 가드에 걸려 정지 → 정지하니 COG가 속도 문턱(0.25 m/s) 미달로 무효 → 접선 폴백이 "오차 0°"를 내놓아 가드 해제 → 엉뚱한 방향으로 재출발 → 속도 붙자 COG 복귀 → 다시 130° → 정지. 이 왕복이 반복되며 횡오차가 2.7→5.1m로 벌어졌다(COG 자체는 정상이었다 — wrap하면 +85~+111°로 일관, 차가 진짜로 130° 틀어져 있었다). **해제 조건**: 신뢰 가능한 헤딩으로 정렬이 0.5s 지속되거나 실제 EstopRequest 인가(at_end 래치와 동일). 스테이트 전이로는 안 풀린다 — 스테이트가 바뀐다고 차가 돌아선 것은 아니다) > **지정 지점 정지**(2026-08-18 — `GpsPath.stop_zone`이 가리키는 트랙 위 지점에 도달하면 v_ref 0으로 서고 `stop_zone_hold_cycles`(기본 300틱=3s) 뒤 스스로 재출발. 언덕 정차 같은 시나리오 요구를 코스에 심는 장치다. 지점은 트랙 CSV 옆 구간 파일(`zones_*.yaml`, `ros2 run stack_gps mark_zone`이 현장에서 기록)에서 stack_gps가 **위경도**로 받아 웨이포인트 인덱스 구간으로 바꿔 싣는다 — 인덱스로 직접 잡으면 트랙을 다시 기록하는 순간 어긋나지만 위경도는 장소를 가리킨다. 설계 3가지: ① 지점 **번호**로 소진 관리(진입 즉시 확정) — bool이면 언덕에서 밀려 구간을 다시 밟을 때 재정지 루프가 된다 ② 기동 시점에 이미 지점 안이면 그 지점만 **임시 억제**(출발도 전에 정차를 까먹는 것 방지) — 단 구간을 벗어나면 억제가 풀린다. ①의 소진과 별도 변수다: 합치면 "지점 안에서 launch → 출발점으로 옮김 → 그 지점을 영영 안 섬"이 된다 ③ 카운트다운은 **실제로 멈춘 뒤** 시작 — 진입 시점부터 세면 감속에 쓴 0.4s만큼 정차가 짧아진다. estop/신호등/종점보다 **아래**다: 안전·코스 종료 요구가 먼저고, 정차 소진은 estop 인가로만 리셋된다) > 가속구간 > 기본 속도 / 횡방향: 차선 경로 / GPS 경로
- **avoid** — 기동 완료 우선, 정지 요구는 기동 이탈 후 적용. 단 TTC < 임계 → 즉시 정지(안전 바닥). 횡: 회피 경로 / 종: v_ref는 회피 기하로 결정(여유 폭 좁으면 감속), **그 위에 스테이트별 상한 `v_avoid` 적용**(2026-08-17 신설, 0 이하면 상한 없음 = 구동작). **왜 stack_avoid의 `target_speed_mps`를 내리지 않는가:** 그 값은 ① v_suggest ② **TTC 자차속도 폴백**(`/vehicle/vector` 미수신 시 — 실측 0건이라 항상 폴백)을 겸한다. v_base 1.0으로 달리며 그것만 0.6으로 내리면 TTC = gap/0.6이 되어 **1.67배 부풀고** avoidable 판정·TTC 안전 바닥이 그만큼 늦게 걸린다. 그래서 "AVOID에서 얼마로 달릴까"는 v_base·v_narrow와 같은 **스테이트별 속도로 MGM이 갖고**, `target_speed_mps`는 실제 주행 속도(=v_base)를 유지해 TTC를 정직하게 둔다. 같은 덤프 재생(`core_replay v_avoid=0 ↔ 0.6`) 검증: **스테이트 시퀀스·immediate_stop 완전 동일**, 회피 v_ref 평균 0.702→0.439만 변화(회피 밖 158틱은 복귀 직후 램프업). **§5.5 이중 트랙: Simulink 모델도 동일 반영 필요.**
- **avoid 안의 후진 탈출 페이즈** (2026-08-24 신설) — **AVOID 우선권 표의 최상위**. 회피 불가 장애물 앞에서 estop이 무한히 유지되는 교착을 끊는다. estop은 레벨 신호라 장애물이 치워져야 풀리는데, 시험 코스에는 치워질 일이 없는 장애물이 있다(길을 막은 구조물·주차된 차). 그러면 v_ref 0으로 영원히 서 있게 된다. 그래서 충분히 오래 갇혀 있었으면 **곧게 조금 물러나** 회피가 성립하는 거리를 만들고 그대로 AVOID에서 회피를 시도한다.
  **왜 스테이트로 승격하지 않는가:** ① 후진은 새 횡방향 경로 소스를 만들지 않는다 ② at_end·역방향처럼 상태를 가진 안전 동작도 주행 스테이트 안의 래치로 둔다(TRAFFIC은 영상 소실 뒤 거리 메모리와 초록 해제라는 독립 수명주기가 있어 별도 상태인 예외) ③ 후진의 목적 자체가 "회피가 성립하는 거리를 만드는 것"이라 **회피 기동의 첫 단계**로 보는 게 의미론적으로 정확하다. 물러난 뒤 그대로 AVOID 안에 있으므로 "후진 끝 → 회피로 인수인계"라는 취약한 이음매가 아예 생기지 않는다.
  **후진 중에는 estop 정지를 무시한다** — 시스템에서 estop이 참인 채로 차를 움직이는 유일한 자리다. 그래서 그 무시가 안전한 이유를 조건이 아니라 **구조**로 못박는다:
  ① `v_escape < 0`이 아니면 코어가 기능을 잠근다 → 이 분기는 전진 명령을 낼 수 없다. estop을 건 장애물은 차 앞에 있고 우리는 그 반대로만 간다. ② `escape_max_cycles > 0` 필수 → 상한 없는 후진은 만들지 않는다(기본 200틱 × 0.3m/s = 0.6m). ③ **주행 무장** — 명령 속도가 0을 넘은 적이 있어야 무장된다. 없으면 벽을 마주 보고 launch한 차가 출발 인가도 전에 10초 뒤 스스로 물러난다. ④ **실제 EstopRequest만 센다**(`estop_latch_release`) — §5.7 watchdog 보정이나 `wait_go` 대기로 걸린 estop은 교착이 아니라 안전 장치다. ⑤ 후방 여유는 진입뿐 아니라 **후진 중에도 매 틱 다시 본다** — 뒤에 뭔가 들어오면 그 자리에서 멈춘다. ⑥ 경로는 인지가 준 것이 아니라 조립 블록이 만드는 **전방 직선**(`MGM_SRC_ESCAPE`, y=0·yaw=0·κ=0)이라 후진 중 조향이 중립이다. 전진용 ref(차 앞의 목표점)를 그대로 두고 v_ref만 뒤집으면 차가 그 목표에서 멀어지는 쪽으로 꺾여 물러나며 엉뚱한 방향으로 돌아버린다.
  종료(시간 상한·후방 막힘·estop 해제)하면 `estop_hold_cnt`를 0으로 리셋한다 — 다시 갇히면 `escape_after_cycles`를 새로 채워야 또 물러나므로, 연속 후진으로 트랙에서 무한히 멀어지는 것을 시간이 막는다. `avoid_zone_only` 게이트는 **적용하지 않는다**: 그 스위치는 "평시 회피를 지정 구간에만 쓴다"는 운용 선택이지 구간 밖에서 갇힌 차를 갇힌 채로 두라는 뜻이 아니다. PARKING은 제외(주차 후진은 `parking_v_suggest`로 stack_parking이 결정).
  **미결 2건** — ⓐ `EstopRequest.rear_clear`를 채우는 구현이 stack_estop에 **없다**(후방/4-LiDAR 통합은 이기돈 판단). 없으면 항상 false라 기본 설정에서 기능이 자연히 잠긴다. ⓑ **실차 미검증** — 음수 v_ref에 대한 dSPACE MPC·하위 PI 동작은 팀 확인(2026-08-24)이나 실차 재확인 권장. 되돌리려면 `ros2 param set /adas_mgm_node escape_after_cycles 0`. 단위시험: `adas_mgm/test/escape_reverse_test.cpp`(9종 — 절반이 "후진하지 않아야 하는 경우"를 고정한다). **§5.5 이중 트랙: Simulink 모델도 동일 반영 필요.**
- **parking** — 경로 침범 정지 > 주차 진행. 신호등·가속구간 요구 비활성. 정적 경계(콘·연석)는 정지 트리거가 아니라 로컬맵 입력
- **traffic** — 횡방향은 lane 경로 유지. **거리 추적은 2026-09-02 개정(사용자 지정)으로 "정지선 소실 edge" 기준으로 확정됐다** — 그 전 두 시도(① 진입 시점 거리·속도로 고정한 운동학적 제동곡선 `v=sqrt(2·a·remaining)`, ② 안정 검출되는 매 틱 새 값을 계속 신뢰)는 정지선 인식이 간헐적으로만 성공하는 조건(2026-09-01 해질녘 실측: YOLO/HSV 후보조차 못 찾는 완전 실패 포함)에서 검증이 더 필요하다는 판단으로 모두 이 방식으로 되돌아갔다. 카메라 optical-Z 거리(`stop_distance`)는 검출이 불안정하면 즉시 무효가 돼 연속 신뢰 기준으로 못 쓴다 — 대신 `stopline_detected`가 true→false로 떨어지는 순간(=정지선이 화면에서 사라짐)을 "카메라 장착 기준 대략 고정된 거리" 시드(`traffic_ramp_distance_m`, 기본 1.5m)로 삼고, 그 뒤로는 dSPACE→PC `/vehicle/vector.v`로 dead-reckoning 감쇠한다. **이 추적은 스테이트와 무관하게 항상 돈다** — 빨간불이 아직 확정 안 된 채(=`traffic_red_active=false`) 감쇠값이 `traffic_stop_offset_m`(2026-09-02 사용자 지정으로 0.5→0.2→1.0m 순으로 조정. 1.0m는 seed와의 차 0.5m가 그대로 제동 여유라 빡빡함을 확인 후 결정 — 2m/s 목표속도 검토 시 `traffic_ramp_distance_m`도 같이 올릴지 재검토 필요) 이하로 떨어지면, 무관한 정지선을 스쳐 지나가며 쌓인 낡은 값이 나중에 빨간불이 뜨는 순간 그대로 급정지로 이어지는 걸 막기 위해 시드로 되돌리고 그 상태를 붙잡아둔다(=실질적으로 "시드에서 0.5m 이상 진행한 상태로 빨간불이 확정돼야만" 실제 정지가 성립). 빨간불 확정 후에는 같은 문턱 이하에서 완전 정지(`v_ref=0`), 그 위에서는 `v_ref = clamp01(traffic_stopline_distance / traffic_ramp_distance_m) × v_base`. **정지선 소실 edge를 한 번도 못 봤으면(=거리를 모르면, 정지선 미인지) v_base로 그냥 통과한다**(2026-09-02 확정, 사용자 지정). 한때 "안전 쪽 폴백"으로 즉시 정지를 넣었었는데, 검증 단계인 지금은 정지선 인지 자체가 미덥지 않아(위 해질녘 완전 실패 사례) "못 봤으면 무조건 정지"가 관련 없는 곳에서 잦은 오정지를 만든다는 판단으로 되돌렸다. traffic 또는 실차속도 입력이 stale이면 fail-safe 정지. 확정 초록만 상태를 해제하며(다음 신호를 위해 거리·래치 폐기) 미검출/unknown은 유지 근거다. 생성 `ADAS_MGR2 v1.88`에는 이 상태가 없으므로 `traffic_state_enabled=true`에서는 generated backend 기동을 거부하고, 모델 재생성 전까지 production core만 사용한다. bridge_dspace/tools/camera_traffic_ref_test.py로 벤치에서 먼저 검증한 방식. **실차 미검증** — 카메라 장착 고정 후 `traffic_ramp_distance_m`/`traffic_stop_offset_m` 재보정 권장.

  **통합 경로 보완(2026-09-04):** 위 항목 첫 문장의 “횡방향은 lane 경로 유지”는
  폐기됐다. TRAFFIC은 종방향 overlay이므로 LANE에서 진입하면 lane, WAYPOINT에서
  진입하면 GPS 경로를 유지하고 초록에서도 같은 상태로 복귀한다. MGM wrapper도
  실제로 선택된 lane/GPS 소스의 freshness와 GPS 유효성을 감시한다.

**Parking 인지 파이프라인 (2026-09-01):** 옆 RPLiDAR의 반복적인 USB 전원
탈락 때문에 주차 SLAM 입력은 `multi_lidar_fusion`이 `base_link`로 보정해 발행하는
전방 `/lidar/a1/cloud`와 후방 `/lidar/a2/cloud`만 사용한다. 둘을 timestamp 허용오차
안에서 한 쌍으로 소비하고 SLAM 갱신은 최대 10Hz로 제한한다. 좌표계는
`parking_map` 시작 자세 = `(0,0,0)`, `base_link` = 후축 중심·`+x` 전방·`+y`
좌측·`+yaw` 반시계다. 자세 prior는 dSPACE `VehicleVector.v`(실속도)와 HandsFree
IMU의 자이로 적분 yaw 증분으로 만들며 `VehicleVector.x/y/yaw`를 직접 자세로 쓰지
않는다. RTK FIXED인 `GpsPath`의 새 `update`만 위치 drift 보정에 사용한다.
`GpsPath.dx/dy/dyaw`는 **직전 vehicle frame** 표현이므로 SE(2) 합성으로 누적하고,
innovation gate와 작은 gain을 통과한 x/y만 prior에 반영한다. IMU가 끊긴 경우에만
`HEADING_FUSED` GPS `dyaw(k-1)`을 yaw 증분 폴백으로 사용한다.

**4-LiDAR 병합 입력 전환 (2026-09-02, 사용자 지정):** 위 front/rear-only 결정은
b1/b2(좌우) USB 전원 탈락이 **아직 해소됐다는 전제로** 4대 병합 입력으로 전환한다
— 즉 이 전제가 실제로 참인지는 이 세션에서 검증되지 않았다. `stack_parking_node`에
`merged_cloud_topic` 파라미터(기본값 빈 문자열 = 기존 front/rear 페어러 유지)를
추가해, 설정하면 `FrontRearCloudPairer`를 완전히 건너뛰고 그 토픽 하나를 그대로
ICP에 먹인다. 겹치는 시야 처리는 `lidar_fusion_v2`(PR #70)의
`/unified_lidar/scan`이 이미 한다 — 1도 bin마다 `np.minimum.at`으로 가장 가까운
거리만 남기는 nearest-wins 방식(`/unified_lidar/cloud`는 겹침 그대로인 raw
concat이라 이 용도에 안 맞음). `stack_parking/launch/parking_mapping_bench.launch.py`가
`scan_to_cloud` 노드(신규, `laser_geometry`로 LaserScan→PointCloud2)로 그 scan을
`/parking/nearest_merged_cloud`로 바꿔 물린다. **b1/b2가 다시 끊기면 이 모드는
전방/후방에도 없는 시야를 조용히 잃는다** — front/rear-only로 되돌리려면
`merged_cloud_topic:=''`.

Parking 내부 단계는 `SLAM → MAPPING → LOCALIZATION → PARKING`으로 분리한다.
SLAM은 초기 scan-to-map 정합을 확보하고, MAPPING은 정적 endpoint map을 계속
누적한다. 공간과 경로가 확정되면 map을 동결한 LOCALIZATION에서 연속 정합을
확인한 뒤에만 PARKING 출력을 MGM에 활성화한다. LOCALIZATION/PARKING 중에는
동적 물체로 정적 map을 오염시키지 않으며, 단계 전까지 `space_found=false`와
0속도 제안만 발행한다.

**히스테리시스 카운터 규약 (2026-08-14 개정 — run_0814_184624 실측으로 도출):**
"N주기 연속"은 **현재 스테이트 안에서** 세어야 한다. 카운터를 스테이트와 무관하게
누적하면 이전 스테이트에 있는 동안 쌓인 값으로 진입 즉시 되튄다 — AVOID로 5~9초
기동하는 동안 `lane_high_cnt`가 500~900까지 차서, waypoint 복귀 **한 틱(10ms)** 만에
lane으로 넘어갔다(복귀 4회 중 2회). §4의 "avoid→waypoint 복귀" 정책이 구조적으로
무력화된 상태였고, 110초에 전이 22회·횡오차 8m 발산으로 나타났다. 따라서:
- **전이가 일어나면 히스테리시스 카운터를 전부 리셋한다** (`lane_low_cnt`,
  `lane_high_cnt`, `wrongway_cnt`).
- 같은 이유로 **GPS 전용 구간 안에서는 `lane_high_cnt`를 0으로 묶는다** (2026-08-18).
  안 그러면 구간을 지나는 내내 쌓인 값으로 **벗어나는 순간 한 틱 만에** LANE이 된다 —
  차선을 못 믿겠다고 지정한 구간을 막 빠져나온 참에 그 구간 동안의 신뢰도로 곧장
  카메라를 믿는 꼴이다. 구간 이탈 후 새로 n_cycles 를 채워야 LANE으로 간다
  (단위시험으로 0틱 → 50틱 확인).
- 리셋만으로는 `n_cycles`(수백 ms)밖에 못 번다 — GPS 트랙 재합류에는 부족하다.
  그래서 **avoid→waypoint 복귀 시 `avoid_return_hold_cycles` 동안 lane 전이를 보류**한다.
  안전 전이(waypoint→avoid)는 보류 대상이 아니다.
- 같은 덤프 재생 비교(`core_replay`): 전이 22→16회, lane↔gps 진동 13→7회,
  복귀 후 gps 체류 0.01s→3.0s. **§5.5 이중 트랙: Simulink 모델도 동일 반영 필요.**

**원칙:**
- **긴급 정지·종점·지정 지점 정지는 스테이트가 아니다.** 단, 신호등은 정지선이 시야에서 사라진 뒤에도 실차속도 적분으로 접근 상태를 보존하고 초록에서 명시적으로 빠져나가야 하므로 TRAFFIC 스테이트로 관리한다.
- 회피는 avoid **스테이트**다. "회피 경로 요구"라는 별도 요구는 존재하지 않는다 (parking→avoid 전이 없음 = 주차 중 회피 금지가 구조적으로 보장됨).

## 5. 구현 금지·필수 사항

1. **조립/병합 블록에 판단 로직 금지.** ref points 조립·종방향 병합은 스테이트의 결정을 포맷 변환·표 적용하는 순수 실행부. `if (조건) v_ref = 0` 같은 조건 분기를 여기 넣는 순간 판단이 두 곳으로 흩어진다. 모든 "무엇을 할지"는 스테이트 머신으로.
2. **MGM 10ms 루프는 인지 노드와 별도 프로세스.** 인지 콜백이 루프를 블로킹하면 안 됨. 루프는 매 틱 "최신 인지 스냅샷"을 읽는 pull 방식. SCHED_FIFO 우선순위·CPU 코어 고정 적용 검토.
3. **주기 지터 로깅 필수.** MGM 루프에 주기 실측 로깅을 처음부터 내장 (§7의 판정 근거).
4. **모든 경로 소스는 동일 ref points 포맷.** 차선/GPS/회피/주차 어느 것이 이기든 dSPACE는 구분할 필요 없음 — MPC 무수정 원칙.
5. **PWM 주파수 혼동 금지.** 서보 = 50Hz(펄스폭이 위치), 구동 모터 드라이버 = 20kHz(duty가 전압). 
6. 스테이트 전환 시 ref 불연속 방지(전환 연속 처리), 급격한 v_ref 변화 rate limit — 조립 블록의 허용 업무.
7. **인지 입력 신선도 watchdog (MGM wrapper).** ① estop: EstopRequest 미수신(기동 직후) 또는 staleness(기본 250ms = stack_estop 하트비트 50ms × 5) 시 wrapper가 스냅샷의 estop을 true로 보정 (2026-07-30, PR #17 안전 이슈 시험에서 도출). ② lane_path/gps_path: 현재 스테이트가 해당 소스를 사용 중일 때(lane→LANE, gps→WAYPOINT) 미수신/staleness(기본 0.5s) 시 동일하게 estop=true로 보정 — 인지 노드 사망 시 마지막 값으로 계속 주행하던 구멍 차단 (2026-08-07 실차에서 발견, PR #28). gps는 **"신선하지만 무효"도 동일 취급**: stack_gps는 fix 상실 시 fix_quality=0인 빈 GpsPath를 계속 발행하므로 수신 시각만으론 통과 — WAYPOINT 중 fix_quality=0 또는 빈 points도 estop=true 보정 (2026-08-11 통합 점검에서 발견). ③ traffic_stop: **수신 이력이 있은 뒤** staleness(기본 0.5s) 시 stop_required=true 보정(estop 아닌 일반 감속 정지) — 미수신은 보정하지 않음(제약 입력이라 단독 스택 시험이 stack_traffic 없이 성립해야 함; false 상태로 사망 후 적색 점등되는 케이스를 막는 것이 목적, 2026-08-08 PR #21 검토에서 도출). ④ 출발 인가 게이트(`wait_go`, 실차 통합 launch 전용): `/operator/go` 인가 전까지 estop 보정 유지 — launch 직후 무점검 출발 방지 (2026-08-11). 인가는 `ros2 run adas_mgm go`가 점검(RTK FIXED·lane·scan·avoid·target_ref 수신) 통과 시 발행. ⑤ avoid: staleness(기본 0.5s) 시 진입 재료(obstacle_detected·avoidable)를 무효화(죽은 stack_avoid의 마지막 메시지로 AVOID 진입 차단)하고, **현재 스테이트가 AVOID면** lane/gps와 동일하게 estop=true 보정 — 회피 기동 중 stack_avoid 사망 시 낡은 회피 경로로 계속 주행하던 구멍 차단 (2026-08-12 회피 통합에서 도출). ⑥ **CAN 헬스 (2026-08-26)**: `bridge_dspace`가 발행하는 `/bridge/can_health`(`CanHealth.msg`)를 보고, `link_up`이 false거나 연속 송신 실패가 `can_fail_ticks`(기본 3 = 30ms, §3 dSPACE watchdog의 3주기와 같은 눈금) 이상이거나 헬스 자체가 stale(기본 0.5s)이면 estop=true 보정. ③ traffic과 같이 **수신 이력이 있은 뒤에만** 판정한다 — 미수신을 보정하면 브리지 없이 도는 단독 스택 시험·재생이 전부 estop이 된다. 다른 ①~⑤가 "인지가 살아 있는가"를 보는 데 비해 이것만 **"우리 명령이 실제로 버스에 나가고 있는가"**를 본다. ⚠ **이 보정의 목적은 "지금 세우는 것"이 아니다** — CAN이 죽어 있으면 v_ref 0조차 실어 보낼 수 없고(마지막 안전망 `can_zero`도 같은 통로를 쓴다), dSPACE counter watchdog(2026-08-31 구현, 실차 미검증)이 걸리기 전 30ms 동안 dSPACE는 마지막 v_ref를 유지한다. 목적은 **링크가 살아난 뒤 나가는 첫 프레임이 정지값이 되게 하는 것**이다. 그 구간에 차를 세울 수 있는 것은 dSPACE watchdog(손상민, 이슈 #49 — 2026-08-31 구현)뿐이다. **재출발 규약 — `can_relatch_sec`(기본 1.0s):** 불건전이 이 시간 넘게 지속되면 래치를 걸어, 링크가 살아나도 `/operator/go` 재인가 전까지 정지를 유지한다. `PROTOCOL.md`의 "복구는 자동 · 별도 해제 절차·래치 없음"과 갈리는 **유일한 지점**이며, 그 규정이 전제한 두절(프레임 몇 개, 수십 ms)은 문턱 아래라 **그대로 자동 복귀한다**. 어댑터 이탈 같은 초 단위 고장에서만 래치가 걸린다 — 그 사이 차가 마지막 v_ref로 굴러갔을 수 있고 사람이 차 옆에 서 있을 수 있어서다. 0 이하면 래치 없음 = PROTOCOL.md 규정 그대로. 기능 시험(CAN 하드웨어 없이): `adas_mgm/tools/can_watchdog_check.py`. ⑦ **parking (2026-08-31, PR #45 통합 검토 P0 ③)**: staleness(기본 0.5s) 시 진입 재료(`space_found`)와 `done`을 무효화하고, **현재 스테이트가 PARKING이면** estop=true 보정 — ⑤ avoid와 같은 구조다. `done`도 무효화하는 이유는, 죽기 직전 메시지의 done으로 PARKING을 빠져나가면 차가 어디에 서 있는지 모르는 채 주행 스테이트로 올라가기 때문이다(estop이 걸린 채 PARKING에 머무는 쪽이 안전하다). 주차는 후진이 섞여(`parking_v_suggest` 음수) 낡은 값으로 굴러가는 것이 특히 위험하다. 미수신도 stale로 보지만 **단독 스택 시험과 양립한다** — `space_found` 기본값이 false라 PARKING에 못 들어가고, PARKING이 아니면 estop 보정도 걸리지 않는다. ⚠ **`CoreSnapshot`에 `parking_updated`를 넣지 않았다**: 넣으면 덤프 포맷이 v6→v7이 되어 `drive_logs`의 기존 스냅샷이 전부 재생 불가가 된다(과거 run 재생이 회귀 검증의 주 수단이다). 수신 시각만으로 판정해 코어도 덤프도 그대로 두었다. 모두 §3 dSPACE counter watchdog의 PC측 대응물이며 판단이 아니라 입력 컨디셔닝 — 정지 판단(v_ref=0)은 여전히 코어 스테이트 머신이 한다. **코어(`mgm_step.cpp`)는 무수정** — CanHealth는 wrapper의 estop 보정으로만 흐르므로 `CoreSnapshot`도 덤프 포맷(v6)도 그대로다.
8. **인지 갱신 지연 틱의 ref hold (조립 블록).** MGM/CAN은 10ms 주기를 유지하지만, 새 인지/GNSS 표본이 오기 전에는 마지막 ref points를 그대로 반복 송신한다(예: GPS 10Hz이면 최대 9틱 hold). 직전 명령 속도로 `x -= v_cmd × 10ms`를 수행하던 경로는 실측 pose 보정이 아니므로 사용하지 않는다. 새 GPS 표본의 vehicle-frame pose delta는 별도 계약으로 한 번 적용한다. 이 규칙은 조립 블록의 데이터 유지이며 스테이트 판단이 아니다. **§5.5 이중 트랙에서는 생성 파일을 직접 편집하지 말고, 현행 ADAS_MGR2 v1.88의 호환 처리는 재생성에 안전한 `GeneratedMgmAdapter`에서 적용한다.**

## 5.5 MGM 개발 전략 (이중 트랙)

- **MGM 로직 코어는 `mgm_step()` 순수 함수로 격리** — ROS 의존성 없음. 시그니처: `출력(ref_points, v_ref, flags) = mgm_step(인지 스냅샷, 이전 상태)`. ROS 2 노드는 10ms 타이머에서 이 함수를 호출하는 wrapper일 뿐이며, **wrapper에 판단 로직 금지**.
- 두 구현이 이 인터페이스를 공유: ① ROS 2/C++ 직접 구현(김윤기, 레퍼런스 — 스프린트 기본 탑재), ② Simulink/Stateflow 모델 → 생성 C 코드(김재민, MBD).
- 검증: 동일 rosbag을 두 구현에 재생 → 출력(ref points, v_ref, 스테이트 전이) 비교. 차이 발견 시 분석 결과를 모델 쪽에 피드백.
- 생성 코드가 안정 동작하면 wrapper에서 함수 교체로 탑재 전환. 스펙의 단일 소스는 본 문서 §4 — 두 구현 모두 여기서만 파생하며, 스펙 변경은 문서 갱신이 선행.

## 6. 워크스페이스 구조

```
adas_ws/src/
├── common_interfaces/     # msg/인터페이스 정의 — 모든 스택의 공용 계약 (§3 포맷)
├── bridge_dspace/         # CAN 브리지 (SocketCAN, TX/RX) — ★ 최우선 구현, 배포 전 완성
├── adas_mgm/              # Decision — core/(§5.5 로직 코어, ROS 무의존) + src/(wrapper) + tools/(back-to-back 하네스)
├── stack_lane/            # 이현준 — 차선 검출(YOLO) → 차선 ref
├── stack_gps/             # 김윤기 — GPS·IMU 융합, RTK, waypoint ref
├── stack_parking/         # 손상민 — 주차공간 인식·로컬맵·주차 경로
├── stack_avoid/           # 이기돈 — 장애물 인지, 회피 가능 판정(TTC·측방), 회피 경로
├── stack_traffic/         # 김재민 — 신호등·정지선 인식 → 정지 요구
└── stack_estop/           # 박찬미 — 돌발 장애물 감지 → 긴급 정지 요구
```

- 각 stack 폴더에 `REQUIREMENTS.md` 포함 (담당자별 요구사항).
- 각 스택의 출력은 `common_interfaces`에 정의된 토픽/메시지로만 — MGM은 그 토픽들만 구독.
- **신호등·정지선 (2026-08-08 개정, PR #21·#28 — 구 stack_lane→stack_traffic 정지선 전달은 폐기):**
  stack_traffic(김재민)이 OAK-D RGB 한 대에서 신호등(YOLOv8n 상단 ROI + HSV 적/녹)과
  정지선(하단 ROI의 YOLO segmentation + 3/5 안정성 검사)을 **모두 자체 검출**한다.
  정지선 depth는 진단용이며 `stopline_detected`의 조건이 아니다. stack_lane은 정지선을
  발행하지 않는다. `StopLine.msg`는 발행자·구독자 모두 소멸 — 삭제 여부 미결.
  신호 페이즈 래치: 적색 3/5 확정 시 `red_phase_latched=true`와 해당 bbox anchor를
  저장하고, **fresh YOLO bbox 또는 저장된 적색 anchor 안의 초록 3/5에서만 false**.
  초록 원·좌회전 화살표 등 점등 모양은 제한하지 않되 anchor 밖 초록색은 무시한다.
  적색과 정지선이 서로 다른 프레임에 검출되는 실차 패턴을 허용한다
  (2026-08-30 run_0830_181646에서 적색과 정지선이 교대로 검출돼 미정지한 문제 수정).
  정지 래치 진입 = `red_phase_latched AND 정지선 근접`. 해제 = **위 target 내부의
  초록 3/5로만**(실차 launch 기준; 패키지 기본은 자동 해제 없음). 카메라 사망·정지선
  소실·bbox 소실은 신호 페이즈나 정지 래치의 해제 조건이 아님.
  두 OAK-D 동시 초기화 경쟁으로 traffic 프로세스가 즉사할 수 있어 통합 실차 launch는
  `stack_traffic_node`를 2초 간격으로 자동 respawn한다. 시작부터 traffic 토픽이 한 번도
  오지 않으면 MGM freshness watchdog은 아직 활성되지 않으므로 출발 전 노드 확인은 필수다.
  CPU 통합 운용은 두 모델 모두 `imgsz=320`이며 기동 중 1회 warm-up한다. 적색 뒤에는
  한 callback에서 정지선 또는 신호등 YOLO 하나만 실행한다. 정지선을 우선하되 신호등을
  3프레임마다 재확인해 초록 3/5 재출발을 유지하고, 그 프레임의 정지선 상태는 직전 결과를
  유지해 인위적인 miss로 세지 않는다. depth는 진단값이라 통합 launch 기본은 RGB-only다.
  **해제 정책 확정(2026-08-09, 팀장):** 시연 신호등은 적색=정지 / 초록=재출발 타입 —
  `resume_on_green`이 실차 표준, `resume_on_red_clear`는 불필요.
  **OAK-D 배분 확정 (2026-08-09, 팀장):** OAK-D Pro **2대** — stack_lane(이현준) 1대(차선용
  하향 pitch), stack_traffic(김재민) 1대(신호등·정지선용 상단 시야). 각자 독점 오픈은 유지하되
  **양쪽 노드에 MxID 핀닝 필수**(`dai.Device(pipeline, device_info)`) — 핀닝 없으면 어느 노드가
  어느 카메라를 잡을지 비결정적이라 부팅 순서에 따라 뒤바뀐다 (담당: 이현준·김재민 각자 자기 노드).
  **MxID 확정 (2026-08-12, 팀장 — 2대 동시 연결 상태에서 실물 열거로 확인):**
  | 용도 | MxID | 핀닝 위치 |
  |---|---|---|
  | 차선 (stack_lane) | `14442C105157D3D200` | `stack_lane/node.py` `camera_mxid` 기본값 (main 반영 완료) |
  | 신호등·정지선 (stack_traffic) | `14442C10B167CFD200` | `stack_traffic/node.py` `oak_mxid` 기본값 + `stack_traffic` launch·통합 실차 launch `traffic_mxid` (main 반영 완료) |
  MxID는 18자리다 — PR #29 본문 메모의 차선용 19자리 표기는 오타(잘못된 `C` 삽입)이며 이 표가 정본.
- **⚠ depthai v3 에서 인자 없는 `dai.Pipeline()` 을 부르지 말 것 (2026-08-29):** v3 는
  이때 **기본 장치를 암묵적으로 연다**(실측: `getDefaultDevice()` 가 Device 반환).
  즉 MxID 핀닝이 적용되기도 전에 아무 카메라나 잡는다 — 위 표의 핀닝이 무력화된다.
  세대 판별은 반드시 **클래스 속성**으로 한다(`hasattr(dai.Pipeline, 'start')` /
  `hasattr(dai.Pipeline, 'createColorCamera')`) — 하드웨어를 건드리지 않는다.
  stack_lane 은 2026-08-25(PR #48), stack_traffic 은 2026-08-29(PR #53)에 각각 고쳤다.
  v3 에는 `dai.node.XLinkOut` 도 없다(실측 `hasattr` = False) — v2 경로 전용이다.
- **⚠ OAK-D USB3가 GPS RTK를 죽인다 — 카메라는 반드시 USB2로 (2026-08-14 실측 확정):**
  OAK-D는 부팅되면 SuperSpeed(5Gbps)로 재열거되는데, 그 광대역 방사 잡음이 GNSS
  L1(1575MHz)을 덮어 **로버 C/N0를 최대 16.5dB 떨어뜨린다**. 같은 안테나 위치에서
  USB3 22dB(DGPS 고착) ↔ USB2 39dB(FIXED). 위성 수·HDOP·RTCM은 **정상값 그대로**라
  기존 상태줄로는 안 보인다 — **C/N0(GSV)를 봐야 한다**(`stack_gps/tools/rtk_probe.py`).
  RTK는 반송파 위상을 쓰므로 코드 기반 측위(DGPS)보다 훨씬 먼저 무너진다.
  | 대책 | 효과 |
  |---|---|
  | `usb_speed:=high`(stack_lane) / `oak_usb_speed:=high`(stack_traffic) | **-16.5dB → -2dB**. 링크 자체를 USB2로 내려 SuperSpeed 신호를 없앤다. **2026-08-24부터 이것이 기본값이다** — 노드(`stack_lane/node.py`, `stack_traffic/node.py`)와 통합 launch 2종 모두 `high`/`fps 10`으로 뒤집었다. 인자를 손으로 붙이는 걸 한 번 잊으면 위성 수·HDOP·RTCM이 전부 정상으로 보이는 채 FIXED만 안 잡혀 원인을 찾기 어렵기 때문이다. USB3가 필요하면 그때 명시적으로 올린다: `usb_speed:=super camera_fps:=30` |
  | `camera_fps` 하향 | **단독으론 무효** (30→10 해도 동일) — 페이로드일 뿐 링크는 계속 5Gbps. 단 USB2는 대역폭 ~40MB/s라 720p 30fps(83MB/s)가 안 들어가므로 **`camera_fps:=10`을 반드시 동반**할 것 |
  | 안테나 이격 | 유효하나 보조 수단 — USB2 적용 후엔 하늘 시야만 보고 위치를 잡으면 된다 |
  2대 동시 가동(둘 다 USB2) 실측: C/N0 39.4~42.4dB, 100초 FIXED 유지 · lane_path 10Hz 정상.
  수신기 교체는 무의미하다 — 로버·베이스 모두 이미 같은 ZED-F9P다.
- **⚠ 로버 수신기가 "열거된 채 출력만 죽는" 고장 (2026-08-18):** RTCM 쓰기는 성공하는데
  NMEA 가 한 줄도 안 오고 C/N0 가 0dB 로 나온다. USB 를 뽑았다 꽂아야 복구됐다.
  `GgaLink` 가 NMEA 무수신 20초에 `USBDEVFS_RESET` 으로 자동 재열거한다
  (`usb_reset_after_s`, 0=끔). **usbfs 권한 때문에 udev 규칙 1회 설치 필요** —
  `src/stack_gps/tools/99-ublox-f9p-usbreset.rules`. 판정은 fix 품질이 아니라
  **NMEA 무수신**으로만 한다(품질 저하는 정상 전파 상황이라 리셋 대상이 아니다).
  상태 로그가 RTCM·NMEA 를 짝으로 찍으므로 두 고장이 구분된다. 상세: RUNBOOK §5-1f.
- 부트스트랩 순서: ① bridge_dspace로 PC↔dSPACE 왕복 검증 (더미 ref로 바퀴 반응 + vehicle vector 회신 확인) → ② adas_mgm 10ms 루프 골격 + 지터 로깅 → ③ 각 스택 배포·병렬 개발.

## 7. 실시간성 검증 기준 (v1 유지 vs v3 이관)

- ref points는 매 주기 vehicle frame (0,0,0)으로 재생성 → **지연에 의한 추종 오차는 새 ref 수신 시 청산, 누적되지 않음.** 따라서 실질 제약은 watchdog 오탐 하나.
- **판정식: "최악 지연 × 2를 watchdog 타임아웃으로 잡았을 때, 그 타임아웃이 안전한가(PC 사망 시 정지 개시까지의 이동거리가 수용 가능한가)?"**
  - Yes → v1 유지 (필요 시 타임아웃 조정, 예: 30→50ms)
  - No (지연 꼬리가 유계되지 않음) → v3 이관
- 측정 조건: 인지 노드 풀가동 상태에서 장시간(수십 분~1시간) 주기 로깅, 최악값 기준(평균 아님).

## 8. 팀 담당 · 일정

| 이름 | 담당 | 8/2 산출물 |
|---|---|---|
| 김윤기 (팀장) | GPS — 베이스 설치, RTK, GPS 주행 | GPS 단독 주행 |
| 이현준 | 카메라 차선 주행 | 차선 단독 주행 |
| 손상민 | 라이다 주차 + MPC·Vehicle MGM (dSPACE 적용, 검증은 카메라·GPS 주행 연계) | 주차 단독 + MPC 검증 |
| 이기돈 | 장애물 회피 + 하위 세부 제어 보완(김재민에게서 인수) | 회피 단독 주행 |
| 김재민 | 신호등·정지선 + 하위제어 사수(기반 7/6~7/17 완료) | 신호등 정지 |
| 박찬미 | 돌발 장애물 긴급 정지 | 긴급 정지 |

미팅 주 2회(월·목). 마일스톤: **8/2 전원 센서 단독 주행 완료**.

## 9. 참조 문서

- `docs/system_architecture_v1.drawio` — 기본 아키텍처
- `docs/system_architecture_v3.drawio` — dSPACE 이관 대안
- `docs/state_machine_detail.drawio` — 스테이트 전이·우선권 표
- `docs/dynamic_architecture.drawio` — 한 제어 주기(10ms) 시퀀스
