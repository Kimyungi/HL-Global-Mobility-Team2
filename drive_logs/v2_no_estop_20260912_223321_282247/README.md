# 한라대 v2 주행 로그 — 2026-09-12

`integration/v2_main`의 `v2_no_estop_20260912_223321_282247` 기록이다.
이번 커밋은 CSV와 메타데이터만 포함한다. 카메라 원본을 포함한 rosbag DB와
MGM 바이너리 덤프는 같은 폴더에 로컬 보관하며 Git에는 포함하지 않는다.

## 기록 범위

- rosbag: 2026-09-12 22:33:21.880 ~ 22:52:24.818 KST, 약 19분 3초, 492,927개 메시지.
- 통합 노드 종료: 22:53:55 KST. CSV·MGM 덤프는 rosbag 종료 이후에도 기록됐다.
- 코스 선택: 01 → 03 → 04 → 05 → 07. 전체 구간의 성공을 뜻하지 않는다.
- 실행 설정: 회피 OFF, LiDAR E-stop OFF, Zone 진입/이탈 확인 5/5,
  주차 탐색은 해당 Zone 내부로 제한. 카메라·GPS·MGM 제어 출력은 1점.
- 실행 시 Git HEAD는 `8697bcb`였으며 미커밋 소스 변경을 빌드해 실행했다.
  이 로그 커밋의 소스 트리만으로 당시 실행 코드를 재현할 수는 없다.
- `session.json`은 실제 실행 인자와 종료 확인 정보를 담는다. 물리 차량의
  조이스틱/자율 모드 전환 시점을 별도로 입증하는 기록은 아니다.

## 포함 파일

| 파일 | 내용 |
|---|---|
| `lateral.csv` | GNSS 품질, 경로 index, 헤딩, station 탐색 범위와 preview |
| `lane_frames.csv` | 차선 인식 프레임 진단 |
| `vehicle_vector.csv` | dSPACE 차량 피드백 |
| `transitions.csv` | MGM 상태 전이 |
| `zone_observations.csv` | Zone 관측 |
| `mission_events.csv` | Mission 이벤트 |
| `mgm_jitter.csv` | MGM 실행 주기 진단 |
| `rosbag/metadata.yaml` | rosbag 토픽, 메시지 수, 기록 시간 |
| `manifest.json`, `SHA256SUMS` | 원본 파일 크기와 SHA-256; 로컬 보관 바이너리도 포함 |
| `session.json` | 실행 인자와 기록 종료 정보 |

`rosbag/metadata.yaml`만으로 bag을 재생할 수는 없다. 원본을 전달받으면
아래 상대 경로에 놓고 `SHA256SUMS`로 일치 여부를 확인한다.

## 로컬 보관 원본 — Git 제외

- `rosbag/rosbag_0.db3`: 49,195,806,720 bytes.
- `mgm_snapshots.bin`: 609,063,072 bytes, raw dump v18.

MGM raw dump는 기록 당시와 동일한 layout/ABI 및 판단 로직의 빌드가 필요하다.
원본 파일을 삭제하거나 재인코딩하지 않았다.
