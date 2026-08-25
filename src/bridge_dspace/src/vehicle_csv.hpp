// vehicle_csv — dSPACE RX 피드백(/vehicle/vector)의 CSV 한 줄 포맷 (2026-08-25).
//
// 왜 헤더로 갈라 두는가: 기록 자체는 can_bridge_node 의 rx 스레드가 하지만, 그 경로는
// 실제 CAN 인터페이스가 있어야만 돌아 단위시험에서 못 만진다. 포맷만 떼어 두면
// "열·자릿수·타임스탬프가 lateral.csv 와 겹칠 수 있는가"를 CAN 없이 검증할 수 있다.
//
// 타임스탬프는 header.stamp 와 같은 epoch 초다 — bag 과 CSV 를 같은 축에 놓기 위해서.
// counter 는 dSPACE 측 로그와의 틱 정합 키다 (CLAUDE.md §3: bag_index = counter − off).
#ifndef BRIDGE_DSPACE__VEHICLE_CSV_HPP_
#define BRIDGE_DSPACE__VEHICLE_CSV_HPP_

#include <iomanip>
#include <ostream>

#include "fma_interfaces/msg/vehicle_vector.hpp"

namespace bridge_dspace
{

inline const char * vehicleCsvHeader()
{
  return "stamp_s,counter,x,y,yaw,v,str\n";
}

inline void writeVehicleCsvRow(
  std::ostream & out, const fma_interfaces::msg::VehicleVector & vv)
{
  const double stamp_s =
    static_cast<double>(vv.header.stamp.sec) +
    static_cast<double>(vv.header.stamp.nanosec) * 1e-9;
  const std::streamsize prev = out.precision();
  out << std::fixed << std::setprecision(3) << stamp_s << ','
      << vv.counter << ','
      << std::setprecision(4)
      << vv.x << ',' << vv.y << ',' << vv.yaw << ','
      << vv.v << ',' << vv.str << '\n';
  out.precision(prev);
}

}  // namespace bridge_dspace

#endif  // BRIDGE_DSPACE__VEHICLE_CSV_HPP_
