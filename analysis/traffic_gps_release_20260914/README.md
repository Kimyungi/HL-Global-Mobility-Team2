# 적색 해제 후 GPS 전환

사용자 조건 `(!red) || (!red && green)`을 `!red`로 구현했다.
base manager에서 이전 신호 상태가 IDLE이 아닐 때 적색이 false가 되면
SIGNAL_IDLE 및 GPS_BACKUP/GPS_ONLY_NAV로 전환한다. LINE 신뢰도 누적을
초기화해 해제 틱부터 곧바로 차선이 재선택되는 일을 막는다. 이후 기존 차선
신뢰도 확인 조건을 충족하면 일반 내비게이션 선택을 따른다.

해제 시 라이다가 유효하고 장애물이 없으며 후진 중이 아니면 MGM의 잔류 회피
상태, 카운터, 참조 확인 기록을 비운다. 현재 검출된 장애물은 회피 우선권을
유지한다. GPS 참조 상실, CAN/외부 정지, AUTO_ESTOP, 주차 권한은 각각 유지한다.
stack_avoid 자체의 표면 기억/3차 경로 생성 방향은 변경하지 않았다.

통합 런처에서 resume_on_red_absence=true를 전달한다. 정상 영상의 기존 투표창에
미검출/무색 관측도 0표로 반영하고 적색 판정이 해제되면 인지 적색/정지 래치도
해제한다. 카메라 읽기 실패는 투표를 갱신하지 않고 고장 발행 시 직전 적색을
보존한다. 단독 인지 노드의 기본값 false 및 legacy MGM은 기존 정책을 유지한다.

검증:
- scripts/v2 build: 13 패키지 성공.
- CTest: 24/24 통과. 새 traffic_gps_release_test 47개 확인 포함.
- stack_traffic Python: 133 통과, 3 건너뜀.
- v2 런처 구조 테스트: 25 통과; ROS 프로세스 실행 없이 launch graph 검사.
- scripts/v2 check 및 git diff --check 통과.
- 새 코어 시험: 적색/초록 4조합, 성숙한 LINE에서 GPS 전환, 잔류 빈 회피,
  기존/센서전체상실 SAFE_STOP 정책, 현재 장애물, GPS 상실, 외부/CAN 정지,
  AUTO_ESTOP, 주차 권한, 신호 없는 일반 주행.
- 인지 callback 시험: 초기 적색 5표와 적색/정지 래치에서 정상 빈 영상
  5프레임 후 red=false, green=false, stop=false; 고장 발행 시 적색 보존.

실차 런처 실행/인가를 수행하지 않았다. 다음 기존 통합 런처 재시작부터 적용된다.
