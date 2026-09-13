# Integration v2 통합 RViz 화면

v2 워크스페이스에서 `./scripts/v2 view`를 실행한다. 차량은 화면 중앙에 고정되며 전방이 위쪽이다.
확대/축소는 마우스로 조절한다. 창을 닫으면 표시 노드도 함께 종료한다.
이 명령은 센서·MGM·CAN·로깅·출발 인가를 실행하지 않는다. 실행 중인 통합 스택의 데이터만 구독한다.

| 표시 | 내용 |
|---|---|
| 흰 차량 윤곽 / 0.5m 격자 | 기존 parking_params.yaml의 차체 치수 사용 |
| 주황 경로 / GPS 화살표 | 현재 CSV 전체 경로와 GPS가 발행한 preview 1점 |
| 하늘색 CAMERA | 카메라가 발행한 preview 1점·yaw |
| 분홍 MGM | 최종 선택된 실제 TargetRef 목표점 |
| 붉은 정지선 | MGM traffic_remaining_m을 앞범퍼 기준으로 놓은 거리 추정 표시 |
| 회색 점군 | SLAM 주변 지도와 관측된 벽 |
| 전·후·좌·우 점군 | 4개 LiDAR의 기존 base_link 변환 결과 |
| 녹색 경계·경로 | 검출된 주차 공간, 전체 계획·현재 실행 경로, 기동/목표 pose |
| 오른쪽 패널 | 신호등 RED/GREEN/UNKNOWN, GPS 상태·CSV·idx, 목표/실제 속도, MGM 상태, 주차 탐색·ready·단계·후방 벽 거리, 두 카메라 영상 |

GPS 전체 경로는 GPS ENU 위치·heading으로, SLAM 지도·주차 경로는 SLAM pose로 각각 차량 좌표로 변환한다.
서로 다른 두 map 원점을 같은 것으로 취급하지 않는다. GPS heading이 접선 폴백이면 패널에 TANGENT estimate로 표시한다.
새 주차 요청 이전의 주차 표시와 새 CSV 이전의 GPS 트랙은 재사용하지 않는다.

정지선의 가로 폭·방향은 거리 확인용 표시다. 실측 정지선 형상은 아니다.
MGM 거리가 미확정이면 그리지 않는다. 음수 잔여거리는 범퍼를 지난 위치로 유지한다.
카메라 optical-Z 값은 패널에 별도 표시하며 MGM 범퍼 거리로 대신 사용하지 않는다.
신호등 아이콘도 상태 표시이며 신호등의 실제 위치를 뜻하지 않는다.

카메라 영상은 통합 launch의 `lane_debug:=true traffic_show_debug:=false`로 사용한다.
차선은 기존 `/perception/lane_debug_image`, 신호등은 `/perception/traffic_debug_image`를 받는다.
신호등 노드는 구독자가 있을 때만 디버그 영상을 발행하므로 RViz 안에서 영상을 볼 수 있다.
`traffic_show_debug=true`는 기존 별도 OpenCV 창이므로 한 창 운용에서는 false로 둔다.
원본 카메라/추적 입력 버퍼에 주석을 그리지 않는다.

수신이 끊기거나 기준 시각이 오래되면 해당 점/선/지도/영상을 지우고 NO DATA/STALE로 표시한다.
주차 모듈이 공간이나 경로를 아직 찾지 않았으면 임의 벽·주차 경로를 만들지 않는다.

표시 노드는 `/integration_v2/view/*`와 전용 TF `base_link → v2_vehicle_view`만 발행한다.
제어 입력과 실제 GPS/SLAM 좌표계는 변경하지 않는다. 화면 배치는
`src/adas_mgm/config/integration_v2.rviz`, 실행은 `integration_v2_view.launch.py`에 저장한다.

검증: GPS/SLAM 좌표 정합, 범퍼 거리·음수 거리, RGB 영상 행 패딩, stale/이전 요청 제거,
신호등 영상의 별도 창 차단 및 원본 버퍼 보존을 센서 없이 시험했다.
