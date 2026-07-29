// Linux SocketCAN 소켓 헬퍼 — can_bridge_node / dspace_sim_node 공용.
#ifndef BRIDGE_DSPACE__SOCKETCAN_HPP_
#define BRIDGE_DSPACE__SOCKETCAN_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace bridge_dspace
{

// RAW CAN 소켓 생성·바인드. filters 지정 시 해당 ID만 수신 (빈 벡터 = 전부 수신).
// rcv_timeout_ms: recv 타임아웃 — 수신 루프의 종료 플래그 확인 주기.
inline int openCanSocket(
  const std::string & ifname,
  const std::vector<can_filter> & filters,
  int rcv_timeout_ms = 100)
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
  return sock;
}

// 8-byte 페이로드 구조체를 can_frame으로 송신.
template<typename Payload>
inline bool sendCanFrame(int sock, uint32_t id, const Payload & payload)
{
  static_assert(sizeof(Payload) <= CAN_MAX_DLEN, "payload exceeds classic CAN frame");
  can_frame f{};
  f.can_id = id;
  f.can_dlc = sizeof(Payload);
  std::memcpy(f.data, &payload, sizeof(Payload));
  return ::write(sock, &f, sizeof(f)) == sizeof(f);
}

}  // namespace bridge_dspace

#endif  // BRIDGE_DSPACE__SOCKETCAN_HPP_
