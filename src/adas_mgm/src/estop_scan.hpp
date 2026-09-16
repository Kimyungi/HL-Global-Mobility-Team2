#ifndef ADAS_MGM_ESTOP_SCAN_HPP
#define ADAS_MGM_ESTOP_SCAN_HPP
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
namespace adas_mgm {
// Same calibrated sensor frame/FOV/range offsets as lidar_fusion_v2.
// Returns the closest exterior clearance, not distance to the sensor origin.
inline float body_clearance(const std::vector<float> & ranges, double angle_min,
  double increment, double range_min, double range_max, const std::vector<double> & mount,
  double front, double rear, double half_width) {
  if (mount.size() != 8) {return std::numeric_limits<float>::quiet_NaN();}
  constexpr double rad = 3.14159265358979323846 / 180.;
  float best = std::numeric_limits<float>::infinity();
  bool observed = false;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double angle = angle_min + i * increment;
    const double degrees = std::remainder(angle / rad, 360.);
    const double normalized = degrees < mount[3] ? degrees + 360. : degrees;
    if (normalized < mount[3] || normalized > mount[4]) {continue;}
    const float raw = ranges[i];
    if (raw == std::numeric_limits<float>::infinity()) {observed = true; continue;}
    if (!std::isfinite(raw) || raw < std::max(range_min, mount[6]) || raw > std::min(range_max, mount[7])) {continue;}
    observed = true;
    const double r = raw + mount[5];
    const double x = mount[0] + r * std::cos(angle + mount[2]*rad);
    const double y = mount[1] + r * std::sin(angle + mount[2]*rad);
    if (x >= -rear && x <= front && std::fabs(y) <= half_width) {continue;} // vehicle self returns
    const double dx = std::max({-rear-x, x-front, 0.});
    const double dy = std::max(std::fabs(y)-half_width, 0.);
    best = std::min(best, static_cast<float>(std::hypot(dx, dy)));
  }
  return observed ? best : std::numeric_limits<float>::quiet_NaN();
}
}
#endif
