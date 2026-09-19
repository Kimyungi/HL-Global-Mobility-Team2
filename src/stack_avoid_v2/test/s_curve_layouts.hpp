#pragma once
#include "stack_avoid_v2/core.hpp"
#include <cmath>
#include <vector>

// Shared geometry for regression tests and HTML export. Left/right are relative
// to the forward GPS tangent, not the fixed frame's Y coordinate.
namespace synthetic
{
struct SCurveLayout
{
  const char * id;
  const char * title;
  const char * description;
  std::vector<avoid_v2::Point> station_offset;
  bool requires_overtake{true};
};
inline std::vector<SCurveLayout> s_curve_layouts()
{
  return {
    {"s_left_right","S자 · 좌 → 우 2대","GPS 진행 방향 기준 왼쪽 0.6m, 오른쪽 0.6m에 차량을 엇갈려 배치합니다.",{{5,.6},{8.5,-.6}}},
    {"s_right_left","S자 · 우 → 좌 2대","같은 S자 코스에서 좌우 배치 순서만 반대로 바꿉니다.",{{5,-.6},{8.5,.6}}},
    {"s_left_right_left","S자 · 좌 → 우 → 좌 3대","station 5m·8.5m·11m에서 GPS 중심선 좌우 0.6m에 세 차량을 배치합니다.",{{5,.6},{8.5,-.6},{11,.6}}},
    {"s_right_left_right","S자 · 우 → 좌 → 우 3대","세 차량의 좌우 순서를 뒤집어 연속 추월과 GPS 복귀를 확인합니다.",{{5,-.6},{8.5,.6},{11,-.6}}},
    {"s_both_sides","S자 · 좌우 동시 / 중앙 통로","같은 station 7m의 좌우 1.15m에 차량을 배치합니다. GPS 중앙 통로가 비어 있으면 추월을 시작하지 않습니다.",{{7,1.15},{7,-1.15}},false}
  };
}
inline avoid_v2::Point place_on_normal(const avoid_v2::Course & route,avoid_v2::Point sd)
{
  const auto center=route.at(sd.x);
  const auto tangent=route.project(center,sd.x-.01,sd.x+.01).yaw;
  return {center.x-std::sin(tangent)*sd.y,center.y+std::cos(tangent)*sd.y};
}
inline avoid_v2::Course s_curve()
{
  avoid_v2::Course c;c.id="synthetic-s-lateral";c.entry=1.;c.exit=15.;
  for(int i=0;i<=88;++i){double x=i*.25;c.center.push_back({x,1.1*std::sin(x*.35)});}
  c.boundary.push_back({-2,-3.5});
  for(auto p:c.center){c.boundary.push_back({p.x,p.y-3.5});}
  c.boundary.push_back({24,c.center.back().y-3.5});c.boundary.push_back({24,c.center.back().y+3.5});
  for(auto i=c.center.rbegin();i!=c.center.rend();++i){c.boundary.push_back({i->x,i->y+3.5});}
  c.boundary.push_back({-2,3.5});c.validate();return c;
}
}
