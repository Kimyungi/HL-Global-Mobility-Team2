// socketcan_fd_test — classic / CAN FD 와이어 포맷 계약 고정 (2026-08-28 FD 이관).
//
// 실제 링크는 CAN 인터페이스가 있어야 돌아가므로, 인터페이스 없이 검증 가능한
// **바이트 레이아웃**만 여기서 못박는다. socketpair 로 sendCanFrame 이 실제로 쓴
// 바이트를 되읽어 검사하므로 write/read 경로 자체가 그대로 exercise 된다.
//
// 이 테스트가 지키는 것:
//   ① FD 로 보내도 페이로드 8바이트·ID·레이아웃이 classic 과 **완전히 동일**하다
//      (1단계 이관의 핵심 전제 — PROTOCOL.md 논리 계약 무변경)
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

  ::close(sv[0]);
  ::close(sv[1]);

  if (failures == 0) {
    std::printf("socketcan_fd_test: OK — FD 와 classic 의 페이로드가 동일하다\n");
  }
  return failures == 0 ? 0 : 1;
}
