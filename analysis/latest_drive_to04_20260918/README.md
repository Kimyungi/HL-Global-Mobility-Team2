# 마지막 주행 · 경로 04 분석

세션: v2_20260918_211819_564358

범위: 148.96초, 14897틱. 05 진입 직전까지.

검증: transitions.csv의 252개 값과 복원 출력 대조 일치.

주요 파일: index.html, route04_report.pdf, route04_all_metrics.csv, state_intervals.csv.

ref source: 0=LINE, 1=GPS, 2=AVOID, 3=PARKING, 4=RECOVERY.
속도 주체: 0=NAVIGATION, 1=AVOIDANCE, 2=TRAFFIC, 3=MISSION, 4=SAFETY, 5=FINISH.
Nav: 0=LINE, 1=GPS_BACKUP, 2=GPS_ONLY. Avoid: 0=INACTIVE, 1=ACTIVE, 3=GPS_RETURN. Safety: 0=NORMAL, 3=SAFE_STOP, 4=ESTOP.

- 목표속도 및 상태는 기록 당시 파라미터와 로컬 빌드 core_replay로 복원하고 transitions.csv와 대조함.
- 05 구간 ESTOP은 범위 밖이므로 제외.
- map 좌표는 해당 세션 GPS 원점 기준. CSV east/north와 원점을 동일하다고 가정하지 않음.
- LiDAR +inf는 유한 장애물 거리 없음. 미수신/무효 입력은 별도 validity로 확인.
- 주차 세부 phase 및 원시 영상/점군은 이 스냅샷에 없어 복원하지 않음. lane_frames.csv와 mission_events.csv는 헤더만 있어 그래프 없음.
- 입력 스냅샷 수신값에는 이전 메시지의 유지값이 포함될 수 있음. age/valid와 함께 해석.