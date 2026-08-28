// Linux SocketCAN 소켓 헬퍼 — can_bridge_node / dspace_sim_node 공용.
//
// classic CAN 2.0A 와 CAN FD 를 **한 소켓에서** 모두 다룬다 (PROTOCOL.md §공통).
//   - 수신: CAN_RAW_FD_FRAMES 를 켠 소켓은 classic(16B read)·FD(72B read)를 **둘 다**
//     받는다. 그래서 RX 는 포맷을 가리지 않는다 — 한쪽만 FD 로 넘어간 마이그레이션
//     과도기에도 프레임이 보이고, "안 보인다"가 곧 배선·비트레이트 문제로 좁혀진다.
//   - 송신: 같은 소켓에 16바이트(can_frame)를 쓰면 classic, 72바이트(canfd_frame)를
//     쓰면 FD 프레임이 나간다. 어느 쪽을 쓸지는 노드의 `can_fd` 파라미터가 정한다.
//
// ⚠ 이 비대칭(RX 는 자동, TX 는 명시)은 의도한 것이다. RX 를 파라미터로 묶으면
//   dSPACE 가 먼저 FD 로 넘어간 순간 PC 소켓이 프레임을 **에러 없이 통째로 버려**
//   "배선은 멀쩡한데 무수신"이 된다 — 2026-08-25 RX 0건 사고와 같은 종류의 침묵이다.
#ifndef BRIDGE_DSPACE__SOCKETCAN_HPP_
#define BRIDGE_DSPACE__SOCKETCAN_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace bridge_dspace
{

// 인터페이스 MTU 조회 — CAN_MTU(16)=classic 전용, CANFD_MTU(72)=FD 가능.
// 실패(장치 없음 등)면 -1.
inline int canIfaceMtu(const std::string & ifname)
{
  const int sock = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock < 0) {
    return -1;
  }
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  const int mtu = (::ioctl(sock, SIOCGIFMTU, &ifr) < 0) ? -1 : ifr.ifr_mtu;
  ::close(sock);
  return mtu;
}

// 이 인터페이스에 FD 프레임을 실을 수 있는가 (= `ip link ... fd on` 이 걸려 있는가).
inline bool canIfaceSupportsFd(const std::string & ifname)
{
  return canIfaceMtu(ifname) >= static_cast<int>(CANFD_MTU);
}

// RAW CAN 소켓 생성·바인드. filters 지정 시 해당 ID만 수신 (빈 벡터 = 전부 수신).
// rcv_timeout_ms: recv 타임아웃 — 수신 루프의 종료 플래그 확인 주기.
// want_fd: CAN_RAW_FD_FRAMES 활성화 (classic·FD 양쪽 수신 + FD 송신 가능).
inline int openCanSocket(
  const std::string & ifname,
  const std::vector<can_filter> & filters,
  int rcv_timeout_ms = 100,
  bool want_fd = false)
{
  const int sock = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock < 0) {
    throw std::runtime_error("CAN socket creation failed");
  }

  if (!filters.empty()) {
    ::setsockopt(
      sock, SOL_CAN_RAW, CAN_RAW_FILTER,
      filters.data(), filters.size() * sizeof(can_filter));
  }

  timeval tv{rcv_timeout_ms / 1000, (rcv_timeout_ms % 1000) * 1000};
  ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
    ::close(sock);
    throw std::runtime_error("CAN interface not found: " + ifname);
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(sock);
    throw std::runtime_error("CAN bind failed on " + ifname);
  }

  // FD 옵션은 **bind 뒤에** 건다 — 이 시점에야 커널이 인터페이스 MTU 를 검증하므로
  // classic 전용 인터페이스면 여기서 확실히 실패한다 (bind 전에 걸면 무조건 성공).
  if (want_fd) {
    const int on = 1;
    if (::setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
      const int mtu = canIfaceMtu(ifname);
      ::close(sock);
      throw std::runtime_error(
              "CAN FD 활성화 실패: " + ifname + " (MTU " + std::to_string(mtu) +
              ", FD 는 72 필요). 실기: sudo /usr/local/bin/can_up.sh " + ifname +
              "  /  루프백: sudo ip link set vcan0 down && sudo ip link set vcan0 mtu 72 "
              "&& sudo ip link set vcan0 up");
    }
  }
  return sock;
}

// 수신 프레임 정규화 — classic(16B)·FD(72B) 를 한 형태로 돌려준다.
// can_frame 과 canfd_frame 은 앞 5바이트(can_id + dlc/len) 레이아웃이 같아서
// canfd_frame 하나로 양쪽을 받아 읽을 수 있다.
struct CanRxFrame
{
  canid_t id{};        // 플래그 제거된 ID
  uint8_t len{};       // 페이로드 길이 (1단계 프로토콜은 전부 8)
  bool fd{};           // true = CAN FD 프레임으로 도착
  bool brs{};          // FD 프레임이 데이터 구간 비트레이트 전환을 썼는가
  uint8_t data[CANFD_MAX_DLEN]{};
};

// 한 프레임 수신. false = 타임아웃/에러 (호출자는 종료 플래그 확인 후 재시도).
// raw_len 을 주면 read() 반환값을 그대로 넘긴다 (진단 로그용).
inline bool readCanFrame(int sock, CanRxFrame & out, ssize_t * raw_len = nullptr)
{
  canfd_frame f{};
  const ssize_t n = ::read(sock, &f, sizeof(f));
  if (raw_len != nullptr) {
    *raw_len = n;
  }
  if (n != static_cast<ssize_t>(CAN_MTU) && n != static_cast<ssize_t>(CANFD_MTU)) {
    return false;
  }
  out.id = f.can_id & CAN_EFF_MASK;   // EFF/RTR/ERR 플래그 제거
  out.fd = (n == static_cast<ssize_t>(CANFD_MTU));
  out.brs = out.fd && ((f.flags & CANFD_BRS) != 0);
  out.len = f.len;
  std::memcpy(out.data, f.data, std::min<size_t>(f.len, CANFD_MAX_DLEN));
  return true;
}

// 8-byte 페이로드 구조체를 송신.
//   fd  = true  → CAN FD 프레임 (페이로드·ID·레이아웃은 classic 과 **완전히 동일**)
//   brs = true  → 데이터 구간 비트레이트 전환(BRS) 요청. dSPACE 설정과 일치해야 한다:
//                 끄면 FD 프레임이지만 전 구간이 nominal 비트레이트라 속도 이득이 0.
template<typename Payload>
inline bool sendCanFrame(
  int sock, uint32_t id, const Payload & payload,
  bool fd = false, bool brs = true)
{
  static_assert(sizeof(Payload) <= CAN_MAX_DLEN, "payload exceeds classic CAN frame");
  if (fd) {
    canfd_frame f{};
    f.can_id = id;
    f.len = sizeof(Payload);
    f.flags = brs ? CANFD_BRS : 0;
    std::memcpy(f.data, &payload, sizeof(Payload));
    return ::write(sock, &f, CANFD_MTU) == static_cast<ssize_t>(CANFD_MTU);
  }
  can_frame f{};
  f.can_id = id;
  f.can_dlc = sizeof(Payload);
  std::memcpy(f.data, &payload, sizeof(Payload));
  return ::write(sock, &f, CAN_MTU) == static_cast<ssize_t>(CAN_MTU);
}

}  // namespace bridge_dspace

#endif  // BRIDGE_DSPACE__SOCKETCAN_HPP_
