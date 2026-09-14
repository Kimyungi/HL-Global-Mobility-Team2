# 차량 위치별 MGM 상태 플롯

대상: `drive_logs/v2_20260914_014412_894120` (2026-09-14 KST).

- `vehicle_position_states.html`: 외부 연결 없이 열리는 인터랙티브 플롯. 색상 기준 선택, 시간 슬라이더, 위치별 전체 상태/속도 조회.
- `vehicle_position_states.png` / `.svg`: 경로 소스·내비게이션·안전 상태 및 회피 구간 확대.
- `positions_with_states.csv`: GPS 2,396개 샘플과 해당 시각 직전 MGM 상태.
- `events.json`: 주요 이벤트의 정확한 시각, 가장 가까운 GPS 샘플과 시간차.
- 재생성: `python3 plot_positions.py` (NumPy, Matplotlib 필요).

CAN vehicle_vector의 x/y가 0으로 기록돼 GPS 위경도를 사용했다. 첫 GPS 위치를 원점으로 WGS84 동/북 거리(m)로 변환했다. 지도 배경이나 경로 스냅을 적용하지 않았다.

스테이트는 기존 검증된 core_replay 결과를 사용했다. GPS 발행 시각에 가장 가까운 직전 스냅샷을 매칭했으며, 시차 중앙값은 약 8ms, 최대 248ms이다(스냅샷 기록 간격이 일시적으로 벌어진 구간). 시각은 GPS fix 자체의 측정 시각이 아니라 lateral.csv 발행 시각이다. fix_age_s를 CSV에 함께 제공한다. GPS 2,316개 샘플은 RTK FIXED, 80개는 FLOAT로, FLOAT를 검은 테두리로 표시했다.

A: 01:46:23.235 AVOID_ACTIVE 진입.
B: 01:46:36.185 회피 reference 소실로 SAFE_STOP.
C: 01:47:40.646 외부 정지 추가.
D: 01:48:13.546 마지막 스냅샷 (마지막 GPS 샘플은 01:48:13.484).

SAFE_STOP은 제어 명령 상태다. B 이후에도 GPS 위치 변화와 CAN 속도 수신이 기록되어 있다. 이 플롯만으로 물리적 정지나 위치 변화의 원인을 확정하지 않는다. 원본 로그와 실행 중인 차량 프로세스는 변경하지 않았다.
