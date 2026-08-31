// socketcan_fd_test — classic / CAN FD 와이어 포맷 계약 고정 (2026-08-28 FD 이관).
//
// 실제 링크는 CAN 인터페이스가 있어야 돌아가므로, 인터페이스 없이 검증 가능한
// **바이트 레이아웃**만 여기서 못박는다. socketpair 로 sendCanFrame 이 실제로 쓴
// 바이트를 되읽어 검사하므로 write/read 경로 자체가 그대로 exercise 된다.
//
// 이 테스트가 지키는 것:
//   ① FD 로 보내도 8바이트 페이로드(0x100 헤더)는 classic 과 **완전히 동일**하다
//   ①-b v5 (PR #52) 64바이트 페이로드의 바이트 오프셋이 DBC 와 일치한다
//      (x@0 y@8 yaw@16 curvature@24 / str_ref@40 counter@48) — dSPACE 실물 프레임 기준
//   ② BRS 플래그가 실제로 실린다 (끄면 FD 인데 속도 이득이 0이라 조용히 손해)
//   ③ readCanFrame 이 16B·72B 를 모두 같은 형태로 돌려준다 (과도기 무수신 방지)
#include <linux/can.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <string>

#include "src/can_protocol.hpp"
#include "src/socketcan.hpp"

using namespace bridge_dspace;  // NOLINT(build/namespaces)

static int failures = 0;

static void check(bool ok, const std::string & what)
{
  if (!ok) {
    std::printf("  [FAIL] %s\n", what.c_str());
    ++failures;
  }
}

int main()
{
  check(isCanReconnectError(ENODEV), "USB 장치 소멸은 재연결 대상");
  check(isCanReconnectError(ENXIO), "재열거 전 소켓 ENXIO는 재연결 대상");
  check(isCanReconnectError(ENETDOWN), "인터페이스 down은 재연결 대상");
  check(!isCanReconnectError(EAGAIN), "수신 timeout은 재연결 대상이 아님");
  check(!isCanReconnectError(EMSGSIZE), "프로토콜 길이 오류는 재연결 대상이 아님");

  // ── 구조체 크기 = 커널이 포맷을 가르는 기준 (read/write 반환 바이트 수)
  check(sizeof(can_frame) == CAN_MTU, "can_frame == CAN_MTU(16)");
  check(sizeof(canfd_frame) == CANFD_MTU, "canfd_frame == CANFD_MTU(72)");

  // ── 두 프레임의 앞 5바이트(can_id, len)가 같은 자리여야 readCanFrame 이 성립한다
  check(offsetof(can_frame, can_id) == offsetof(canfd_frame, can_id), "can_id 오프셋 일치");
  check(offsetof(can_frame, can_dlc) == offsetof(canfd_frame, len), "len 오프셋 일치");

  int sv[2];
  check(::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv) == 0, "socketpair 생성");

  // 실제 헤더 페이로드로 왕복 — 값이 아니라 **바이트**가 같은지 본다
  TargetHeaderPayload hdr{};
  hdr.counter = 0x1234;
  hdr.state = 1;                          // waypoint
  hdr.n_points = 20;
  hdr.v_ref = quantize(0.949, kVelScale);  // 949 mm/s
  hdr.reserved = 0;

  uint8_t classic_bytes[8]{}, fd_bytes[8]{};

  // ── classic 송신
  check(sendCanFrame(sv[0], kIdTargetHeader, hdr, /*fd=*/false), "classic 송신 성공");
  {
    CanRxFrame rx{};
    check(readCanFrame(sv[1], rx), "classic 수신 성공");
    check(!rx.fd, "classic 은 fd=false 로 판정");
    check(rx.id == kIdTargetHeader, "classic ID 보존");
    check(rx.len == 8, "classic len == 8");
    std::memcpy(classic_bytes, rx.data, 8);
  }

  // ── FD 송신 (BRS on)
  check(sendCanFrame(sv[0], kIdTargetHeader, hdr, /*fd=*/true, /*brs=*/true), "FD 송신 성공");
  {
    CanRxFrame rx{};
    check(readCanFrame(sv[1], rx), "FD 수신 성공");
    check(rx.fd, "FD 는 fd=true 로 판정");
    check(rx.brs, "BRS 플래그가 실려 있다");
    check(rx.id == kIdTargetHeader, "FD ID 보존");
    check(rx.len == 8, "FD len == 8 (패딩 없음)");
    std::memcpy(fd_bytes, rx.data, 8);
  }

  // ★ 1단계 이관의 핵심 계약: 와이어 포맷만 바뀌고 페이로드는 한 바이트도 안 바뀐다
  check(std::memcmp(classic_bytes, fd_bytes, 8) == 0, "FD 페이로드 == classic 페이로드");

  // 되읽은 바이트가 원래 구조체와 같아야 한다 (little-endian 유지)
  check(std::memcmp(fd_bytes, &hdr, sizeof(hdr)) == 0, "헤더 구조체 바이트 보존");

  // ── BRS off 도 확인 (dSPACE 가 BRS 를 못 켜는 경우의 대조 설정)
  check(sendCanFrame(sv[0], kIdTargetHeader, hdr, /*fd=*/true, /*brs=*/false), "FD(BRS off) 송신");
  {
    CanRxFrame rx{};
    check(readCanFrame(sv[1], rx), "FD(BRS off) 수신");
    check(rx.fd && !rx.brs, "BRS off 면 brs=false");
  }

  // ── ref point 도 같은 계약 (모든 TX 프레임에 적용된다는 확인)
  RefPointPayload pt{
    quantize(1.8, kPosScale), quantize(-0.25, kPosScale),
    quantize(0.12, kYawScale), quantize(0.469, kCurvScale)};
  check(sendCanFrame(sv[0], kIdRefPointBase + 7, pt, /*fd=*/true), "REF_POINT FD 송신");
  {
    CanRxFrame rx{};
    check(readCanFrame(sv[1], rx), "REF_POINT FD 수신");
    check(rx.id == kIdRefPointBase + 7, "REF_POINT ID = 0x101 + i");
    check(rx.len == 8, "REF_POINT len == 8");
    RefPointPayload back{};
    std::memcpy(&back, rx.data, sizeof(back));
    check(back.x == pt.x && back.y == pt.y && back.yaw == pt.yaw &&
      back.curvature == pt.curvature, "REF_POINT 필드 보존");
  }

  // ── v5 (PR #52) 64바이트 페이로드 — 바이트 레이아웃이 DBC 와 맞아야 한다
  {
    MpcTargetFdPayload tgt{};
    tgt.x = 1.8; tgt.y = -0.25; tgt.yaw = 0.12; tgt.curvature = 0.469;
    tgt.dx = 0.06; tgt.dy = -0.01; tgt.dyaw = 0.005; tgt.update = 42;
    check(sendCanFrame(sv[0], kIdRefPointBase, tgt, /*fd=*/true), "MPC_TARGET(64B) 송신");
    CanRxFrame rx{};
    check(readCanFrame(sv[1], rx), "MPC_TARGET(64B) 수신");
    check(rx.fd && rx.len == 64, "0x101 은 FD 64바이트");
    // DBC 오프셋: x@0 y@8 yaw@16 curvature@24 dx@32 dy@40 dyaw@48 update@56
    double back[7];
    std::memcpy(back, rx.data, sizeof(back));
    check(back[0] == 1.8 && back[1] == -0.25 && back[2] == 0.12 && back[3] == 0.469,
      "x/y/yaw/curvature 가 DBC 오프셋 0/8/16/24");
    check(back[4] == 0.06 && back[5] == -0.01 && back[6] == 0.005,
      "dx/dy/dyaw 필드 보존");
    uint64_t upd = 1;
    std::memcpy(&upd, rx.data + 56, sizeof(upd));
    check(upd == 42, "update 필드 보존 (오프셋 56)");
  }
  check(stateUsesGpsDelta(0), "LANE(camera)는 GPS delta 사용");
  check(stateUsesGpsDelta(1), "WAYPOINT(GPS)는 GPS delta 사용");
  check(!stateUsesGpsDelta(2), "AVOID는 GPS delta 미사용");
  check(!stateUsesGpsDelta(3), "PARKING은 GPS delta 미사용");
  {
    GpsDeltaUpdateGate gate;
    check(gate.shouldApply(0, 42), "LANE의 새 update는 1회 적용");
    check(!gate.shouldApply(0, 42), "LANE의 반복 update는 재적용 금지");
    check(gate.shouldApply(1, 43), "WAYPOINT의 다음 update는 적용");
    check(!gate.shouldApply(2, 44), "AVOID update는 적용 금지");
    check(!gate.shouldApply(3, 44), "PARKING update는 적용 금지");
  }
  {
    VehFeedbackFdPayload fb{};
    fb.x = 6.6; fb.y = 0.5; fb.yaw = 0.3; fb.v = 0.949;
    fb.str = -0.21; fb.str_ref = -0.34; fb.counter = 2057;
    check(sendCanFrame(sv[0], kIdVehFeedback, fb, /*fd=*/true), "VEH_FEEDBACK(64B) 송신");
    CanRxFrame rx{};
    check(readCanFrame(sv[1], rx), "VEH_FEEDBACK(64B) 수신");
    check(rx.fd && rx.len == 64, "0x200 은 FD 64바이트");
    // DBC 오프셋: x@0 y@8 yaw@16 v@24 str@32 str_ref@40 counter@48 reserved@56
    double d[6];
    std::memcpy(d, rx.data, sizeof(d));
    check(d[3] == 0.949, "v 가 오프셋 24");
    check(d[5] == -0.34, "str_ref 가 오프셋 40 (v5 신규)");
    uint64_t cnt = 0;
    std::memcpy(&cnt, rx.data + 48, sizeof(cnt));
    check(cnt == 2057, "counter 가 오프셋 48 — 실차 dSPACE 프레임과 같은 자리");
  }

  // 8바이트를 넘는 페이로드는 classic 프레임에 못 실린다 (조용히 잘리면 안 된다)
  {
    MpcTargetFdPayload tgt{};
    check(!sendCanFrame(sv[0], kIdRefPointBase, tgt, /*fd=*/false),
      "64B 페이로드는 classic 송신이 거부된다");
  }

  ::close(sv[0]);
  ::close(sv[1]);

  if (failures == 0) {
    std::printf("socketcan_fd_test: OK — FD 와 classic 의 페이로드가 동일하다\n");
  }
  return failures == 0 ? 0 : 1;
}
