# FMA CAN 링크 파라미터 — ★ 단일 진실 원천 ★
#
# 이 파일이 유일한 정의다. can_up.sh(udev 경로)·can-iface@.service(systemd 경로)·
# 80-can0.network(networkd 경로) 세 곳이 같은 값을 써야 하는데, 예전에는 세 파일에
# 값을 각각 적어 놔서 **한쪽만 갱신되는 사고**가 반복됐다:
#   - 2026-08-03: networkd 에 500k 잔재가 남아 udev 의 1Mbps 를 매 삽입마다 뒤집음 → ERROR-PASSIVE
#   - 2026-08-28: /usr/local/bin/can_up.sh 만 classic 시절 버전으로 남음 → 재삽입 시 FD 실패
# 그래서 이제 install.sh 가 이 파일을 읽어 나머지 셋을 **생성**한다. 값은 여기서만 고친다.
#
# ★ 이 값들은 dSPACE 측(RTI CAN FD 블록셋)과 반드시 일치해야 한다.
#   하나라도 어긋나면 ERROR-PASSIVE / BUS-OFF 로 떨어진다.
#   근거·합의 표: src/bridge_dspace/PROTOCOL.md §공통 "FD 파라미터"

CAN_NOM_BITRATE=1000000      # nominal (중재 구간) 비트레이트
CAN_NOM_SAMPLE_POINT=0.8     # nominal 샘플포인트
CAN_DATA_BITRATE=2000000     # data 구간 비트레이트 (BRS 로 전환되는 구간)
CAN_DATA_SAMPLE_POINT=0.8    # data 샘플포인트
CAN_RESTART_MS=100           # bus-off 자동 복구 지연
CAN_TXQUEUELEN=100
