# LiDAR 4대 통합 실행

현재 v2 통합은 `lidar_fusion_v2`의 `unified_lidar_v2` 노드를 사용한다.
`scripts/v2 lidar`는 이 작업 폴더의 `install_v2` 환경에서 드라이버 4대,
통합 노드, RViz만 실행한다. 차량 제어 노드는 포함하지 않는다.

저장소 루트에서 실행:

```bash
# USB 배선 변경 후 위치 링크가 없거나 접근 권한이 없을 때.
# 실행 중인 LiDAR/차량 노드를 종료한 상태에서 4대 일련번호 확인 및 링크 복구.
sudo /usr/bin/python3 scripts/v2_recover_lidars.py --apply

# 앞/뒤/좌/우 장치 링크 확인 후 실행
ls -l /dev/lidar_front /dev/lidar_rear /dev/lidar_left /dev/lidar_right
./scripts/v2 lidar
# 화면 없이 실행하려면: ./scripts/v2 lidar rviz:=false
# 종료: 실행 터미널에서 Ctrl-C
```

| 위치 | 슬롯 | 입력 |
|---|---|---|
| 앞 | a1 | `/lidar/a1/scan` |
| 뒤 | a2 | `/lidar/a2/scan` |
| 좌 | b1 | `/lidar/b1/scan` |
| 우 | b2 | `/lidar/b2/scan` |

통합 출력은 `/unified_lidar/scan` (`LaserScan`)과
`/unified_lidar/cloud` (`PointCloud2`), 좌표계는 `base_link`다.
기본 발행 주기는 10 Hz이며 로그의 `active=['a1', 'a2', 'b1', 'b2']`로
네 센서가 통합에 기여하는지 확인한다. 일부 센서가 끊겨도 나머지 출력은
계속되므로 통합 토픽의 존재만으로 네 대 정상 수신을 판단하지 않는다.

장착 위치와 yaw/FOV 설정은
`src/lidar_fusion_v2/config/fixed_geometry.yaml`에 있다.
기존 2026-09-02 장착 보정값을 사용하므로 센서를 새 위치에 장착했다면
실측 및 벽 정렬 검증이 필요하다. 보정 절차는
[패키지 문서](../src/lidar_fusion_v2/README.md)를 참고한다.

기존 `run_4lidar_v2.sh`는 일반 `install` 경로를 사용하므로 이 v2 작업 폴더에서는
위 `scripts/v2 lidar` 진입점을 사용한다. 전체 차량 실행 전에 LiDAR 전용 실행을
Ctrl-C로 종료해 동일 시리얼 포트를 중복 사용하지 않는다.
