// vehicle_csv_test — dSPACE RX 피드백 CSV 한 줄 포맷 고정 (2026-08-25).
// 실제 기록 경로(can_bridge_node rx 스레드)는 CAN 인터페이스가 있어야 돌아가므로,
// CAN 없이 검증 가능한 포맷 계약만 여기서 못박는다.
#include <cstdio>
#include <sstream>
#include <string>

#include "src/vehicle_csv.hpp"

using bridge_dspace::vehicleCsvHeader;
using bridge_dspace::writeVehicleCsvRow;
using fma_interfaces::msg::VehicleVector;

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
  // ── 헤더 열 이름과 개수는 dspace_merge.py / 분석 스크립트의 계약이다
  check(std::string(vehicleCsvHeader()) == "stamp_s,counter,x,y,yaw,v,str\n",
    "헤더 열 이름");

  VehicleVector vv;
  vv.header.stamp.sec = 1787132883;
  vv.header.stamp.nanosec = 659000000u;
  vv.counter = 4242u;
  vv.x = 1.23456f;
  vv.y = -0.5f;
  vv.yaw = 0.7853982f;    // 45deg
  vv.v = 0.9487f;
  vv.str = -0.2109f;

  std::ostringstream out;
  writeVehicleCsvRow(out, vv);
  const std::string row = out.str();

  // stamp 는 lateral.csv 와 같은 epoch 초·소수 3자리 — 두 파일을 같은 축에 겹치기 위해
  check(row.rfind("1787132883.659,", 0) == 0, "stamp_s epoch 초 3자리: " + row);
  check(row.find(",4242,") != std::string::npos, "counter 는 정수 그대로");
  check(row.find("1.2346") != std::string::npos, "x 4자리 반올림");
  check(row.find("-0.5000") != std::string::npos, "y 음수 4자리");
  check(row.find("0.7854") != std::string::npos, "yaw 4자리");
  check(row.find("0.9487") != std::string::npos, "v 4자리");
  check(row.find("-0.2109") != std::string::npos, "str 음수 4자리 (부호는 PC ref y 와 반대 — CLAUDE.md §3)");
  check(!row.empty() && row.back() == '\n', "줄바꿈으로 끝난다");

  // 열 개수가 헤더와 같아야 한다
  size_t commas = 0;
  for (char c : row) {if (c == ',') {++commas;}}
  check(commas == 6, "열 7개 (쉼표 6개)");

  // ── 스트림 상태를 되돌려 놓는가 (뒤이어 쓰는 쪽이 fixed/precision 에 오염되면 안 됨)
  std::ostringstream mixed;
  mixed << std::setprecision(2);
  const std::streamsize before = mixed.precision();
  writeVehicleCsvRow(mixed, vv);
  check(mixed.precision() == before, "호출 뒤 precision 복원");

  // ── 여러 줄을 이어 써도 각 줄이 독립적인가
  std::ostringstream multi;
  multi << vehicleCsvHeader();
  for (int i = 0; i < 3; ++i) {
    vv.counter = static_cast<uint32_t>(i);
    writeVehicleCsvRow(multi, vv);
  }
  size_t lines = 0;
  for (char c : multi.str()) {if (c == '\n') {++lines;}}
  check(lines == 4, "헤더 1 + 데이터 3줄");

  std::printf("vehicle csv format: failures=%d\n", failures);
  return failures == 0 ? 0 : 1;
}
