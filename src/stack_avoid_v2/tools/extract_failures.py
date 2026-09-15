#!/usr/bin/env python3
"""Extract recorded failures and a source-backed scenario inventory, without replanning."""
import csv
import argparse
import hashlib
import json
import re
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
CORE = 'src/stack_avoid_v2/src/core.cpp'
NODE = 'src/stack_avoid_v2/src/node.cpp'
WATCH = 'src/stack_avoid_v2/tools/watchdog.py'
HEADER = 'src/stack_avoid_v2/include/stack_avoid_v2/core.hpp'
SOURCES = {p: (ROOT/p).read_text() for p in (CORE, NODE, WATCH, HEADER)}

# '코드 조건' means a condition/limitation exists in source, not that its full
# physical driving scenario has been tested. Potential contact is never inferred
# solely from an intentional HOLD.
CATALOG = [
    ('G01', '통로·차체', '벽 사이 폭이 차폭 0.62m보다 넓어도 양쪽 0.15m 여유가 안 들어감', '코드 조건', '기본 폭 0.92m 미만은 통로 탈락. 0.92m 이상도 통과 보장 없음', CORE, 'half=cfg_.width/2+cfg_.margin'),
    ('G02', '통로·차체', '직선에서는 들어가지만 S자에서 차량이 비스듬해져 앞·뒤 모서리가 벽에 가까워짐', '코드 조건', '길이 0.85m와 회전 자세를 포함한 사각형 검사에서 HOLD 가능', CORE, 'const double hx=(cfg_.front+cfg_.rear)/2+pad'),
    ('G03', '통로·차체', '좌우 차량이 앞뒤로 촘촘히 교대해 짧은 거리 안에 조향을 반전해야 함', '추가 재현', '이번 시험의 station 간격 1/1.5/2m는 HOLD, 2.5/3.5/4.5m는 완주. 전역 임계값 아님', CORE, 'for(double lookahead:{.8,1.1,.6,1.4})'),
    ('G04', '통로·차체', '급커브 또는 차량 사이 꺾임이 최대 조향각보다 작은 회전 반경을 요구함', '코드 조건', '축거 0.595m·최대 조향 0.476rad에서 후륜축 기준 반경 약 1.154m. 감속만으로 해소 불가', CORE, 'const double maximum=std::min'),
    ('G05', '통로·차체', '실제 비스듬한 통로는 있지만 GPS 수직 단면들의 자유 구간이 이어지지 않음', '코드 조건', '단면 간 겹침 검사와 전진 station 표현 때문에 존재하는 다른 경로를 놓칠 수 있음', CORE, 'std::max(low,p.low)>std::min(high,p.high)+.02'),
    ('G06', '통로·차체', '현재 차체는 비어 있는 곳에 있지만 첫 단면의 가용 중심 구간에서 벗어나 있음', '코드 조건', '첫 구간의 현재 횡위치 허용 범위 low−0.1~high+0.1 밖이면 초기 통로 선택 실패 가능', CORE, 'current_cross>=low-.1'),
    ('G07', '통로·차체', '왼쪽·오른쪽 길 중 선택한 저비용 통로만 동역학적으로 불가능함', '미검증 가설', '단면 DP의 선택 통로와 유한 평활화·선행거리 후보만 검사하므로 대안 경로 존재를 배제할 수 없음', CORE, 'int chosen=0;const auto & last=slices.back().gaps;'),
    ('G08', '통로·차체', '벽 끝이나 차량 모서리에서 중간점이 급변하고 선행 구간의 허용 범위가 서로 충돌함', '코드 조건', '평활화 전 가용 범위 교집합이 비면 해당 정책 탈락; 평활화 후에도 차체·제동 검사 실패 가능', CORE, 'if(bounds[i].low>bounds[i].high)'),
    ('G09', '통로·차체', '짧게 전진해 더 관측하면 되지만 현재 관측으로 충분한 길이의 경로를 만들 수 없음', '코드 조건', '8개 단면 미만 또는 검증 길이 1.1m 미만이면 HOLD. 짧게 움직여 관측을 늘리는 별도 정책 없음', CORE, 'if(slices.size()<8)'),
    ('G10', '통로·차체', '우회 공간이 GPS 단면 검색 ±4m 밖에 있거나 후진·큰 우회가 필요함', '코드 조건', '단면 범위 밖은 탐색하지 않으며 station 역행 후보는 버림', CORE, 'const double d=-4+j*dd'),
    ('G11', '통로·차체', '한 경로의 안전한 앞부분을 따라갔지만 더 앞에서 막다른 통로가 드러남', '미검증 가설', '6m 계획과 부분 경로 허용은 전체 코스 도달 가능성 증명이 아님. 이후 HOLD 가능', CORE, 'if(best_length<cfg_.preview+.1)'),
    ('V01', '속도·조향·제동', '좁은 곳을 늦게 발견해 감속 전에 제동 공간이 부족해짐', '기존 재현', '현재 조향의 정지 궤적이 막히면 경로 생성 전에 HOLD. 근접 차량 진단 포함', CORE, 'if (!braking_clear(current,grid))'),
    ('V02', '속도·조향·제동', '현재 통과 경로는 있으나 조향을 유지하며 정지하면 연석·차량을 침범함', '코드 조건', '현재·중간·말단 제동 검사가 후보를 거부. 다른 제동 조향 정책은 탐색하지 않음', CORE, 'return segment(s,s.steer,stopping_distance(s.speed)'),
    ('V03', '속도·조향·제동', '유동 속도를 넣어도 안내선과 실행 경로를 충분히 만들기 전에 HOLD함', '기존 재현', '기존 중앙 배치 3종의 최저 실행 속도는 1m/s. 초기 속도·순항 상한 변경 결과는 별도 표 참조', CORE, 'wall midline has no certified forward rollout'),
    ('V04', '속도·조향·제동', '0.2m/s보다 더 느린 진행이나 정지 후 조향이 필요한 틈', '코드 조건', '현재 속도 목표 하한은 0.2m/s. 정지 조향·후진을 포함한 별도 동작을 계획하지 않음', CORE, 'cfg_.crawl_speed,cfg_.cruise_speed'),
    ('V05', '속도·조향·제동', '측정 오차·추종 오차 때문에 안전한 이전 경로와 일치하지 않아 매번 새 계획을 만듦', '미검증 가설', '재사용 허용 오차는 위치 1.5cm, 속도 0.03m/s, yaw·조향 각각 0.02rad. 반복 재계획 시 50ms 속도 유지와 새 안내선이 감속·추종에 영향을 줄 수 있음', CORE, 'if(nearest<.015&&cut)'),
    ('V06', '속도·조향·제동', '일치 허용 범위 안의 실제 차량 자세에서 이전 경로로 연결했지만 그 연결을 실제 조향기로 못 따라감', '미검증 가설', '재사용은 차체·제동·가감속·조향 변화 검사를 수행하지만 조향 지연 모델로 전체 연결을 다시 생성하지 않음. 오차를 주입한 폐루프 시험 필요', CORE, 'retained.push_back(current)'),
    ('V07', '속도·조향·제동', '실제 조향 속도·지연·최대 조향각 또는 제동 성능이 설정 모델보다 나쁨', '미검증 실차 조건', '계획이 valid여도 추종 이탈·제동 거리 초과 가능. 0.8rad/s·0.33s·제동 1m/s²는 실측 확인 필요', CORE, 'State Planner::advance'),
    ('V08', '속도·조향·제동', '미끄러운 노면·경사·급격한 가감속 응답 때문에 bicycle 모델과 실제 운동이 다름', '미검증 실차 조건', '타이어 슬립·경사·종방향 액추에이터·jerk 모델은 없음. 가감속 수치 검사만으로 실제 추종 보장 불가', CORE, 'const double integration_speed=(start.speed+next_speed)/2'),
    ('V09', '속도·조향·제동', '유효한 재사용 경로를 100ms 실행한 상태가 같은 지도에서도 제동 검사를 통과하지 못함', '추가 재현', 'S자 중앙 3대·상한 0.2m/s: 23.3초 plan valid/reused 및 현재 제동 검사 통과, 100ms 뒤 차체 검사 통과지만 제동 검사 실패. 다음 23.4초 HOLD. 표본 검사 사이 실행 상태의 제동 가능성 보장이 빠진 재현 사례', CORE, 'for(double arc=std::min(.02,ds)'),
    ('L01', 'LiDAR·점유', '앞차가 두 번째·세 번째 차량 또는 안쪽 연석을 가림', '코드 조건', '가려진 영역은 unknown이므로 계획·제동 공간 부족으로 HOLD 가능. HTML 완주 시험은 실제 가림 재생이 아님', CORE, 'bool Grid::observed_free'),
    ('L02', 'LiDAR·점유', '자유 공간 관측이 0.3초 이상 갱신되지 않음', '기존 재현', 'free TTL 만료로 unknown 전환. 차량이 사라진 것으로 취급하지 않음', CORE, 'now-c.free_stamp<=cfg_.free_ttl'),
    ('L03', 'LiDAR·점유', '실제 장애물이 없어졌거나 오검출했지만 해당 셀을 다시 비췄다는 증거가 부족함', '코드 조건', '점유는 시간만으로 삭제되지 않고 서로 다른 새 clear 관측 2회가 필요. 유령 벽으로 진행 불가 가능', CORE, 'if (c.clear_count>=2)'),
    ('L04', 'LiDAR·점유', 'Inf/NaN 또는 범위 밖 반환만 오거나 설정 FOV가 실제 진행 공간을 제외함', '코드 조건', '빈 도로라도 쓸 수 있는 광선이 없으면 관측 실패 또는 unknown. Inf를 자유 공간으로 쓰지 않음', NODE, 'if(!std::isfinite(raw)'),
    ('L05', 'LiDAR·점유', '낮은 연석·검은 차체·유리·반사면을 빔이 놓치거나 잘못된 장거리 반환을 줌', '미검증 실차 조건', '실제 벽 검출 실패 가능. 2D 광선의 관측 가정이 깨지면 HOLD 또는 잘못된 자유 셀로 인한 접촉 위험', NODE, 'rays.push_back({origin,'),
    ('L06', 'LiDAR·점유', '빔 사이의 작은 물체 또는 연석 모서리가 10cm 격자에서 사라지거나 통로를 과하게 좁힘', '미검증 실차 조건', '격자 전체를 통과한 빔의 free로 표현하는 근사. 얇은 물체 누락과 보수적 폭 손실을 별도 측정해야 함', CORE, 'steps=int(std::ceil(len/(cfg_.resolution*.4)))'),
    ('L07', 'LiDAR·점유', 'LiDAR 외부 보정·장착 위치·yaw·거리 오프셋 또는 scan 시간 의미가 실제와 다름', '미검증 실차 조건', '유한값·프레임 문자열이 맞아도 수치 보정의 정확성은 검사하지 않음. 벽 위치 왜곡 가능', NODE, 'const Point origin{pose.x+std::cos(pose.yaw)*sensor.x'),
    ('L08', 'LiDAR·점유', '자기 차체 반사가 유효 범위 안에 들어와 장애물 벽으로 남음', '미검증 실차 조건', '이 입력부에는 별도의 자차 형상 마스크가 없음. 관측된 점유가 현재 차체와 겹치면 HOLD 가능', NODE, 'const double raw=scan.ranges[i]'),
    ('L09', 'LiDAR·점유', '센서 하나가 끊기거나 지연되고 나머지 센서만 정상임', '코드 조건', '설정된 모든 센서가 0.2초 안에 유효해야 계획. 대체 센서만으로 계속 가는 모드 없음', NODE, 'required sensor missing/stale'),
    ('L10', 'LiDAR·점유', '원시 scan 프레임·각도·시간 간격·거리 범위·포인트 수가 계약과 다름', '기존 일부 재현', 'time_increment≤0, 총 scan 시간>0.15s, 포인트 2~10000 밖 등을 거부. 프레임·시간 결손은 ROS smoke에서 확인', NODE, 'invalid raw scan frame/timing/envelope'),
    ('L11', 'LiDAR·점유', '빔 시각이 pose 이력의 미래/과거 밖이거나 pose 사이 공백이 0.15초를 넘음', '코드 조건', '외삽 없이 scan 거부. 마지막 빔 시각의 pose가 아직 안 들어오는 전달 순서도 시험 필요', NODE, 'missing per-beam pose history'),
    ('L12', 'LiDAR·점유', '고정 차량 가정이 깨져 차량·사람이 스캔 사이에 통로로 움직임', '범위 밖 조건', '시간에 따른 장애물 이동 예측이 없음. 현재 사용자 조건은 고정 차량이며 동적 장애물 회피 보장은 없음', CORE, 'void Grid::observe'),
    ('P01', 'GPS·위치', 'RTK FIXED가 아니거나 position/실측 heading/zone/경로 점이 유효하지 않음', '기존 일부 재현', 'fix_quality=4, 실측 heading 필수. 경로 접선 heading은 거부하고 HOLD', NODE, 'm.fix_quality!=4'),
    ('P02', 'GPS·위치', '처음 고정한 GPS–odometry 정합과 후속 GPS가 어긋남', '코드 조건', '위치>0.15m 또는 yaw>0.08rad면 alignment fault. 새 세션 전까지 재개 불가', NODE, 'GPS/odometry frame disagreement; new session required'),
    ('P03', 'GPS·위치', 'VehicleVector 위치/yaw가 순간적으로 튀거나 속도·조향이 유효 범위를 벗어남', '코드 조건', '위치 변화>max(v)×dt+0.10m 또는 yaw 변화>3dt+0.08rad는 정합 fault. 음수 속도/속도>3/조향각 초과/비유한값 거부', NODE, 'alignment_fault_=true;vehicle_valid_=false'),
    ('P04', 'GPS·위치', 'GPS 0.5초 또는 pose 0.1초 유효 기간을 넘기거나 시계가 역행함', '코드 조건', '스탬프 age가 음수여도 fresh가 아님. 중복/역순 GPS·pose가 반복되면 갱신이 끊겨 HOLD', NODE, 'const double age=now().seconds()-t'),
    ('P05', 'GPS·위치', 'GPS 경로와 경계의 frame/route ID 또는 활성 경로 instance·ack·connecting 상태가 다름', '코드 조건', '잘못된 경로를 쓰지 않도록 입력 거부. 경로 전환 시 메시지 도착 순서도 중단 원인 가능', NODE, 'm.route.instance_id!=gps_route_->route.instance_id'),
    ('P06', 'GPS·위치', 'S자 인접 구간에 잘못 투영되거나 정상 갱신 간격보다 크게 이동함', '미검증 가설', 'station 제한 창과 최근접 투영이 의도한 가지를 선택한다는 보장은 없음. cross>3m/투영 실패는 직접 거부', CORE, 'station_+std::fabs(current.speed)*cfg_.observation_gap*1.5+.5'),
    ('P07', 'GPS·위치', '차량 방향이 GPS 진행 방향과 1.2rad 이상 다르거나 후보가 뒤쪽 station으로 진행함', '코드 조건', '약 68.8도 이상 방향 차이와 역행 후보 거부. 실제 전진 경로가 있어도 GPS 표현과 어긋나면 HOLD 가능', CORE, 'vehicle heading opposes GPS direction'),
    ('P08', 'GPS·위치', '경계/GPS 경로 메시지가 주행 중 같은 내용으로 반복 발행됨', '미검증 가설', 'configure_course는 동일 내용 비교 없이 지도·정합·세션을 reset. 반복 배포 시 새 관측 대기로 진행이 끊길 수 있음', NODE, 'planner_->set_course(c);grid_->configure(c);course_ready_=true;reset();'),
    ('Z01', 'zone·출구', 'zone 진입 직후 첫 차량이 이미 제동 거리 안에 있음', '기존 재현', 'zone 밖에서는 인지하지 않으므로 진입 여유가 없으면 근접 HOLD. 진입 후 새 스캔 대기도 필요', NODE, 'grid_->configure(planner_->course());zone_enter_stamp_=t'),
    ('Z02', 'zone·출구', 'zone 신호가 없거나 실제 장애물 구간에서 false로 잘못 유지됨', '코드 조건', '유효성 결손은 HOLD. 아직 한 번도 진입하지 않은 상태에서 유효 false면 GPS 소유권이라 장애물 인지를 시작하지 않음', CORE, 'if(!perception_required())'),
    ('Z03', 'zone·출구', '출구를 지나도 후단 통과·GPS 횡오차 0.1m·yaw 약 20도·3회 관측·zone 해제가 동시에 충족되지 않음', '코드 조건', '완료/인계가 안 되고 벽 주행이 계속됨. 연속 관측 끊김이나 zone true 고정도 원인', CORE, 'completed_=completed_||(!in_zone_&&aligned_count_>=cfg_.completion_samples)'),
    ('Z04', 'zone·출구', '출구 뒤 GPS 경로가 너무 짧거나 코스 끝에서 1m 목표점·제동 공간을 못 만듦', '코드 조건', 'insufficient horizon 또는 preview/말단 제동 HOLD. 코스 끝 정지 전용 경로는 없음', CORE, 'insufficient planning horizon'),
    ('T01', '시간·유효기간', '복잡한 벽·많은 셀·CPU 경합으로 코어 20ms 또는 callback 35ms를 초과함', '코드 조건', '유효 출력 무효화/HOLD. 현재 코어 시간 측정은 센서 ingress부터 CAN까지 전체 50ms가 아님', NODE, 'callback processing budget exceeded'),
    ('T02', '시간·유효기간', 'trigger 센서 a1 주기가 10Hz보다 느리거나 불규칙함', '코드 조건', '계획은 고정 10Hz 타이머가 아니라 trigger scan 이벤트에 종속. 0.1초 요청 속도와 실제 갱신 간격 불일치 가능', NODE, 'if(id!=trigger_){return;}'),
    ('T03', '시간·유효기간', '메시지 또는 관측 generation이 0.15초 동안 갱신되지 않음', '기존 일부 재현', '독립 watchdog이 정지 진단 발행. 노드의 입력 허용 0.2초보다 먼저 발생할 수 있음', WATCH, 'now-self.generation_received > .15'),
    ('T04', '시간·유효기간', '현재 경로와 무관한 오래된 free 셀이 전체 계획 lease를 짧게 만듦', '코드 조건', '모든 usable free 셀의 가장 이른 만료를 사용. 발행 직전 lease 만료 또는 짧은 유효기간으로 HOLD 가능', CORE, 'valid_until_=std::min(valid_until_,c.free_stamp+cfg_.free_ttl)'),
    ('T05', '시간·유효기간', '코어 반환 뒤 직렬화·ROS 큐·CAN·액추에이터 지연이 증가함', '미검증 실차 조건', '전체 50ms 계측·보장이 없음. 유효 경로라도 소비 시점이 늦으면 모델의 반응 거리 가정과 불일치', NODE, 'const double processing=(steady()-start)*1000.'),
    ('T06', '시간·유효기간', '반복/0 generation, 비유한 현재 상태/시각 등 비정상 코어 입력', '코드 조건', 'invalid or repeated input으로 경로 생성 거부. last valid 경로를 무검사로 출력하지 않음', CORE, 'invalid or repeated input'),
    ('I01', '실행 연결', 'HOLD가 나왔지만 실제 속도 제어기가 계속 이전 명령으로 달림', '확인된 연결 미구현', '현재 패키지는 그림자 출력뿐. HOLD·watchdog_stop을 MGM/CAN/E-stop에 연결하지 않아 실제 정지 보장이 없음', NODE, 'Shadow-only ROS adapter'),
    ('I02', '실행 연결', '1m 앞 한 점만 기존 제어기에 넣고 점별 속도·계획 유효기간·소유권을 소비하지 않음', '미검증 실차 조건', 'HTML은 다점 경로의 이상적 실행. 기존 한 점 추종·MGM 인계·reference.v_suggest 소비와 동일한 결과 보장 불가', NODE, 'ref.v_suggest=valid?p.speed_limit:0'),
    ('I03', '실행 연결', '브레이크 검사에서는 통과했지만 실제 제동 중 조향이 풀리거나 반대로 바뀜', '미검증 실차 조건', '검사는 측정 조향 유지 궤적. 실제 비상정지 조향 동작이 다르면 검증한 정지 공간 밖으로 이동 가능', CORE, 'Hold measured steering through the stop'),
    ('I04', '실행 연결', '현재 속도가 측정치보다 빠르거나 속도 요청을 즉시 도달 속도로 해석함', '미검증 실차 조건', '경로 속도는 측정값에서 출발하고 요청값은 100ms 뒤 값. 속도 오차·단위/지연 불일치가 제동·추종 가정을 깨뜨릴 수 있음', CORE, 'out.speed_limit=sample_path_time(out.path,.1).speed'),
    ('B01', '지도·설정', '명시적 경계 또는 GPS 다점 경로가 없거나 frame/ID가 다름', '기존 일부 재현', '초기 지도 구성이 안 되고 HOLD. LiDAR만으로 지도/GPS 경로를 만드는 SLAM은 없음', NODE, 'GPS route and explicit boundary required'),
    ('B02', '지도·설정', 'GPS 중복 점·5m 초과 점 간격·자가교차 경계·경계 밖 중심선·부적절한 entry/exit', '코드 조건', 'Course.validate 예외로 코스 거부. 상세 오류 문자열은 아래 자동 추출 목록 참조', CORE, 'void Course::validate()'),
    ('B03', '지도·설정', '잘못된 차량/시간/격자 파라미터 또는 센서 FOV·보정 설정 결손', '코드 조건', 'Config 검증/노드 생성 예외. 유효한 수치라도 실차와 다른 보정은 이 검증으로 발견 못 함', CORE, 'void Config::validate()'),
    ('B04', '지도·설정', '500000 셀 이상 지도 또는 점 수 제한을 넘긴 대형 입력', '코드 조건', '격자 용량/코스 크기 제한으로 초기화 실패. 제한 아래에서도 처리 시간이 커질 수 있음', CORE, 'course exceeds grid capacity'),
    ('B05', '지도·설정', '실제 도로보다 지도 경계가 좁거나 실제 연석과 GPS 경계가 어긋남', '미검증 실차 조건', '지도 밖은 강제 금지이므로 false HOLD 가능. 지도가 넓고 LiDAR까지 연석을 놓치면 실제 경계 보호를 보장하지 못함', CORE, 'inside(p,course.boundary)'),
]


def location(path, needle):
    text = SOURCES[path]
    assert needle in text, (path, needle)
    return {'path': path, 'line': text[:text.index(needle)].count('\n')+1}


def main():
    page = (ROOT/'docs/avoid_v2_path_lab.html').read_text()
    data = json.loads(re.search(r'<script id="replay-data" type="application/json">(.*?)</script>', page, re.S)[1])
    core_hash = hashlib.sha256(((ROOT/HEADER).read_bytes()+(ROOT/CORE).read_bytes())).hexdigest()
    assert core_hash == data['coreHash'], 'HTML is stale relative to current core'
    cases = []
    for s in data['scenarios']:
        if s['outcome'] == 'complete':
            continue
        f = s['frames'][-1]
        cases.append({
            'id': s['id'], 'title': s['title'], 'expected_hold': s['expectedHold'],
            'outcome': s['outcome'], 'time': f['time'], 'state': f['ego'],
            'speed_min': min(p['ego'][3] for p in s['frames']), 'reason': f['reason'],
            'placements_station_offset': s['placements'], 'boxes_world': s['boxes'],
            'lane_width': s['laneWidth'], 'boundary': s['boundary'], 'center': s['center'],
            'last_valid_path': next((p['path'] for p in reversed(s['frames']) if p['valid']), []),
            'vehicle_contact': s['audit']['vehicleContact'], 'curb_contact': s['audit']['curbContact'],
        })
    probe = list(csv.DictReader((ROOT/'docs/avoid_v2_failure_probe.csv').open()))
    assert len(probe) == 88, 'incomplete bounded sweep'
    assert all(r['result'] in ('complete', 'hold', 'contact', 'timeout', 'invalid_fixture') for r in probe)
    inventory = []
    for key, group, scenario, status, effect, path, needle in CATALOG:
        inventory.append(dict(id=key, group=group, scenario=scenario, status=status,
                              effect=effect, source=location(path, needle)))
    # Preserve source failure messages and configuration exceptions as an appendix.
    reasons = []
    excluded = ('outside obstacle zone; GPS owns reference', 'wall midpoint corridor certified',
                'wall corridor speed profile revalidated')
    for path in (CORE, NODE):
        for line, text in enumerate(SOURCES[path].splitlines(), 1):
            if any(k in text for k in ('finish(', 'publish_blocked(', 'fail(', '.reason=', 'throw std::')):
                for message in re.findall(r'"([^"\n]+)"', text):
                    if message not in excluded:
                        reasons.append(dict(reason=message, source=dict(path=path, line=line)))
    result = dict(generated=datetime.now(timezone.utc).isoformat(), core_hash=core_hash,
                  scope='Current source conditions + existing replay failures + bounded 88-case sweep; not exhaustive physical-state proof',
                  model=data['model'], recorded_failures=cases, probe=probe,
                  inventory=inventory, source_messages=reasons,
                  braking_probe=(ROOT/'docs/avoid_v2_failure_braking_probe.txt').read_text())
    (ROOT/'docs/avoid_v2_failure_catalog.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    counts = Counter(r['result'] for r in probe)
    out = [
        '# 현재 회피 로직의 실패 가능 시나리오', '',
        f"추출 시각: {result['generated']} · `New_Avoid_v2` · 코어 SHA-256 `{core_hash}`", '',
        '**범위:** 현재 소스의 실패 분기·알고리즘 제약, 기존 HTML/시험의 재현 실패, 추가 88개 조건을 추출했다. 연속적인 모든 실차 상태를 전수 검사했다는 뜻은 아니다. `HOLD`는 경로 미완주이며 충돌 발생과 구별한다. 현재 연결에서는 HOLD 이후 실차 정지가 보장되지 않는다.', '',
        f"현재 소스에서 분류한 조건은 **{len(inventory)}종**, 추가 시험은 **88개 중 완주 {counts['complete']} / HOLD {counts['hold']} / 접촉 {counts['contact']} / 시간초과 {counts['timeout']} / 잘못된 배치 {counts['invalid_fixture']}**다. 추가 시험은 각 1회이며 전체 공간의 실패 확률이 아니다.", '',
        '## 1. 기존 HTML에서 추출한 미완주 배치', '',
        '일반 배치 3종이 각 3회 실패해 기존 60회 일반 시험에서 9회 HOLD했다. 별도의 예상 HOLD 6종은 각 3회로 18회다. 아래 시각·상태는 현재 HTML의 한 기록에서 직접 추출했다.', '',
        '| ID | 배치 | 구분 | HOLD 시각(s) | 최저 실행 속도(m/s) |',
        '| --- | --- | --- | --- | --- |',
    ]
    for c in cases:
        out.append(f"| `{c['id']}` | {c['title']} | {'예상 HOLD' if c['expected_hold'] else '**미완주**'} | {c['time']:.1f} | {c['speed_min']:.3f} |")
    out += ['', '자차·모든 고정 차량은 0.85×0.62m, 도로 폭 3m다. 일반 미완주의 공통 반환 사유는 `wall midline has no certified forward rollout`이다. 이 사유는 통로 안내선은 만들어도 필요한 길이의 차체·조향·제동 검증 경로를 확보하지 못했다는 뜻이다. 최종 문자열만으로 조향·제동·충돌 후보 탈락 중 하나를 유일한 원인으로 확정할 수는 없다.', '',
            'S자 중앙 배치는 GPS station 7m에 법선 방향 ±1m로 차량 2대를 둔다. 중심 간 법선 거리는 2m지만 차량 자세가 지도 +X 방향이므로 실제 벽 면 사이 법선 간격은 단순히 2−0.62m로 계산할 수 없다. 직선 중앙 배치는 station 5·8m 및 5·8·11m에 둔다.', '',
            '별도 기존 평행 벽 시험은 폭 0.64·0.80·1.00·1.10m에서 HOLD, 1.20·1.30m에서 완주했다. 이 역시 S자 전체의 통과 한계값은 아니다.', '',
            '## 2. 추가 속도·앞뒤 간격 시험', '',
            '추가 시험도 C++ 코어와 기존 동일 차량 fixture를 사용한다. 10Hz 관측과 시간 기반 실행, 차체·연석의 독립 사각형 검사, 0.01초 실행 검사를 유지했다. 미완주 시 HOLD 프레임에서 종료하며 실제 제동 실행은 검사하지 않는다. 차량 위치·도로를 넓혀 성공시키지 않았다.', '']
    for family, title, detail in [
        ('entry_speed', '진입 속도만 변경', '순항 상한은 1m/s로 유지한다. 낮은 속도로 진입해도 이후 가속할 수 있으므로 지속적인 저속 주행 시험과 다르다.'),
        ('cruise_speed', '순항 상한과 진입 속도를 함께 변경', '0.2/0.4/0.6/0.8/1.0m/s를 각각 상한과 초기 속도에 적용한다.'),
        ('longitudinal_spacing', '좌우 교대 차량의 앞뒤 station 간격 변경', '순항 상한·초기 속도 1m/s, 법선 좌우 오프셋 ±0.6m, 첫 차량 station 5m. 간격은 GPS 중심선 호길이상의 배치 위치 차이이며 차량 외곽 사이 거리·유클리드 최단거리가 아니다.'),
    ]:
        sub = [r for r in probe if r['family'] == family]
        values = ['1', '1.5', '2', '2.5', '3.5', '4.5'] if family == 'longitudinal_spacing' else ['0.2', '0.4', '0.6', '0.8', '1']
        out += [f'### {title}', '', detail, '', '| 배치 | '+' | '.join(values)+' |', '| --- | '+' | '.join(['---']*len(values))+' |']
        for scene in dict.fromkeys(r['scene'] for r in sub):
            rows = [r for r in sub if r['scene'] == scene]
            assert len(rows) == len(values)
            out.append('| `'+scene+'` | '+' | '.join('완주' if r['result'] == 'complete' else '**'+r['result'].upper()+'**' for r in rows)+' |')
        out.append('')
    out += ['측정한 간격 사이를 연속적으로 탐색하지 않았으며, 특정 간격 이상이면 항상 통과한다는 단조성도 증명하지 않았다. S자 형상·차량 자세·측방 오프셋·초기 상태가 바뀌면 결과가 달라진다. `s_center_gap_2`와 `_3`의 차이는 세 번째 차량이 안내선·속도 계획에 영향을 주기 때문에 단순 장애물 개수 증가 문제로 해석할 수 없다. 정확한 원인 분리는 추가 진단이 필요하다.', '',
            '### 추가 발견: valid 경로의 다음 실행 상태에서 제동 검사 실패', '',
            '별도 재현에서 S자 중앙 3대·순항 상한 0.2m/s의 23.3초 계획은 valid이며 기존 경로를 재사용했다. 현재 상태의 제동 검사도 통과했다. 그러나 **동일한 지도에서 그 경로를 100ms 실행한 상태는 차체 검사를 통과하면서 제동 검사는 실패**했다. 다음 23.4초 계획은 `current braking envelope blocked or unknown`으로 HOLD했다. 새 센서 가림·지도 갱신 없이도 나타났으므로 이 사례는 기존 경로의 표본 검사가 실행 시각의 제동 가능성을 충분히 보장하지 못함을 보여준다. 실제 접촉이 발생했다는 뜻은 아니다.', '',
            '[추적 로그](avoid_v2_failure_braking_probe.txt), [재현 CSV](avoid_v2_failure_braking_probe.csv). 순항 상한을 바꾸면 속도뿐 아니라 상한 대비 속도 비율로 정하는 lookahead도 달라지므로, 낮은 속도에서 실패했다는 결과를 물리적인 저속 불리함으로 해석하면 안 된다.', '',
            '## 3. 소스 기반 전체 조건 목록', '',
            '`코드 조건`은 해당 제한/분기가 존재한다는 뜻이며 그 실차 상황을 모두 재현했다는 뜻은 아니다. `미검증` 항목은 발생을 확정한 결함이 아니라 우선 시험할 조건이다.', '']
    for group in dict.fromkeys(c['group'] for c in inventory):
        out += [f'### {group}', '', '| ID | 실패 가능 상황 | 근거 수준 | 결과·한계 | 소스 |', '| --- | --- | --- | --- | --- |']
        for c in inventory:
            if c['group'] != group:
                continue
            loc=c['source']; link=f"[{Path(loc['path']).name}:{loc['line']}](../{loc['path']}#L{loc['line']})"
            out.append(f"| {c['id']} | {c['scenario']} | {c['status']} | {c['effect']} | {link} |")
        out.append('')
    out += ['## 4. 우선 확인할 실패', '',
            '1. **실행 상태의 제동 보장:** V09의 동일 지도 재현을 먼저 고정하고, valid 경로의 실제 소비 시각에서 제동 가능성이 유지되는지 검사한다.',
            '2. **실행 연결:** HOLD 시 실제 차량이 무엇을 하는지와 브레이크 중 조향 동작을 확인해야 한다. 현재 코드만으로 접촉 없는 정지를 주장할 수 없다.',
            '3. **경로 생성:** 중앙 배치 3종과 station 간격 2m 이하 좌우 교대 배치를 대상으로 통로 선택·후보별 차체/동역학/제동 탈락 원인을 분리한다.',
            '4. **실제 센서:** 뒤 차량 가림, 연석 높이, 자기 차체 반사, scan 끝 시각의 pose 도착 순서와 GPS 정합 오차를 시험한다.',
            '5. **속도 실행:** 이상적인 경로 실행 대신 조향·속도 오차와 종방향 액추에이터 지연을 주입해 경로 재사용/감속이 유지되는지 확인한다.',
            '6. **시간:** trigger 주기와 ROS 전달·CAN까지 전체 지연, 0.15초 watchdog과 0.2초 센서 허용의 관계를 계측한다.', '',
            '## 5. 자동 추출한 오류/거부 문자열', '',
            '코어·ROS 어댑터의 경로 거부, 구성 예외, 발행 무효화 문자열이다. 일부는 정상적인 방어 동작/설정 변경 거부이며 충돌을 뜻하지 않는다. Python watchdog은 위 T03에 별도 분류했다.', '',
            '| 문자열 | 소스 |', '| --- | --- |']
    for r in reasons:
        p=r['source'];out.append(f"| `{r['reason']}` | [{Path(p['path']).name}:{p['line']}](../{p['path']}#L{p['line']}) |")
    out += ['', '## 재현과 산출물', '',
            '[구조화된 추출 JSON](avoid_v2_failure_catalog.json)은 기존 실패의 배치·마지막 상태·마지막 유효 경로와 소스 조건을 포함한다. [88개 추가 시험 CSV](avoid_v2_failure_probe.csv)는 성공도 함께 보존해 실패만 골랐을 때 생기는 해석 오류를 피한다. [기존 78회 원시 결과](avoid_v2_adaptive_results.txt), [기존 검사 로그](avoid_v2_adaptive_checks.txt).', '',
            '```bash',
            'g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I src/stack_avoid_v2/include \\',
            '  src/stack_avoid_v2/src/core.cpp src/stack_avoid_v2/tools/failure_probe.cpp \\',
            '  -o /tmp/avoid_v2_failure_probe',
            '/tmp/avoid_v2_failure_probe > docs/avoid_v2_failure_probe.csv',
            '/tmp/avoid_v2_failure_probe --trace-slow-center \\',
            '  > docs/avoid_v2_failure_braking_probe.csv 2> docs/avoid_v2_failure_braking_probe.txt',
            'python3 src/stack_avoid_v2/tools/extract_failures.py', '```', '']
    (ROOT/'docs/AVOID_V2_FAILURE_SCENARIOS.md').write_text('\n'.join(out))
    print(json.dumps({'inventory':len(inventory), 'recorded_failures':len(cases),
                      'probe':dict(counts), 'source_messages':len(reasons)}, ensure_ascii=False))


def after_fixes():
    """Keep the original audit immutable; record current outcomes separately."""
    baseline = json.loads((ROOT/'docs/avoid_v2_failure_catalog.json').read_text())
    page = (ROOT/'docs/avoid_v2_path_lab.html').read_text()
    data = json.loads(re.search(r'<script id="replay-data" type="application/json">(.*?)</script>', page, re.S)[1])
    core_hash = hashlib.sha256((ROOT/HEADER).read_bytes()+(ROOT/CORE).read_bytes()).hexdigest()
    assert data['coreHash'] == core_hash, 'rebuild HTML for the current core first'
    probe = list(csv.DictReader((ROOT/'docs/avoid_v2_failure_probe_after_fixes.csv').open()))
    assert len(probe) == 88
    changes = {
        'V09': ('수정 · 실행 시각 회귀 검사 통과', CORE, 'bool Planner::braking_transition_clear'),
        'P08': ('수정 · 동일/변경 경로 ROS 재수신 검사', NODE, 'if(course_ready_&&boundary_&&gps_route_&&applied_boundary_'),
        'G01': ('개선 · 자유 경계 정밀화, 물리적 폭 제한 유지', CORE, 'Refine free/blocked boundaries'),
        'G02': ('개선 · 실제 조향 범위로 이동 여유 계산, 차체 여유 유지', CORE, 'const double steering_bound='),
        'G07': ('개선 · 반대쪽 통로 후보 추가, 완전 탐색 보장 없음', CORE, 'struct Policy {double lead,smoothing,bias;}'),
        'V01': ('개선 · 전체 제동 거리만큼 감속 선행, 근접 HOLD 유지', CORE, 'cfg_.front+stopping_distance(speed)'),
        'V03': ('개선 · S자 중앙 통로 완주, 직선 중앙 HOLD 잔존', CORE, 'for(bool crawl_policy:'),
        'T01': ('진단 수정 · callback 기한 초과 플래그, 시간 제약 유지', NODE, 'plan.deadline_hit=true;plan.reason="callback'),
    }
    inventory = []
    for old in baseline['inventory']:
        item = dict(old)
        item['baseline_source'] = item.pop('source')
        item['current_status'] = '조건 유지 또는 미검증; 수정 완료로 분류하지 않음'
        if item['id'] in changes:
            status, path, needle = changes[item['id']]
            item['current_status'] = status
            item['current_source'] = location(path, needle)
        inventory.append(item)
    failures = []
    for scene in data['scenarios']:
        if scene['outcome'] == 'complete':
            continue
        frame = scene['frames'][-1]
        failures.append(dict(id=scene['id'], title=scene['title'], expected_hold=scene['expectedHold'],
                             time=frame['time'], state=frame['ego'], reason=frame['reason'],
                             placements=scene['placements'], boxes=scene['boxes']))
    result = dict(generated=datetime.now(timezone.utc).isoformat(), core_hash=core_hash,
                  baseline_core_hash=baseline['core_hash'], inventory=inventory,
                  current_recorded_failures=failures, probe=probe,
                  probe_counts=dict(Counter(p['result'] for p in probe)),
                  next_braking_failures=sum(int(p['next_braking_failures']) for p in probe))
    target = ROOT/'docs/avoid_v2_failure_catalog_after_fixes.json'
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(f'{target}: {result["probe_counts"]}, next braking failures={result["next_braking_failures"]}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', action='store_true', help='Requires the original baseline source and replay')
    args = parser.parse_args()
    if not args.baseline and (ROOT/'docs/AVOID_V2_FIXES.md').exists():
        after_fixes()
    else:
        if args.baseline and (ROOT/'docs/avoid_v2_failure_catalog.json').exists():
            expected = json.loads((ROOT/'docs/avoid_v2_failure_catalog.json').read_text())['core_hash']
            actual = hashlib.sha256((ROOT/HEADER).read_bytes()+(ROOT/CORE).read_bytes()).hexdigest()
            assert expected == actual, 'restore baseline source before rebuilding the historical report'
        main()
