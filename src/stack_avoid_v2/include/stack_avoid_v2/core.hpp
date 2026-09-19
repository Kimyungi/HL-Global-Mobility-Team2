#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace avoid_v2
{
struct Point {double x{}, y{};};
struct State {double x{}, y{}, yaw{}, speed{}, steer{};};
struct TimedState {double time{}; State state;};
double wrap(double angle);
bool finite(const State & state);
State sample_path(const std::vector<State> & path, double distance);
State sample_path_time(const std::vector<State> & path, double seconds);

// No silent extrapolation across an unobserved pose interval.
class PoseHistory
{
public:
  bool push(TimedState sample);
  bool at(double time, State & state, double max_gap = .15) const;
  void clear() {samples_.clear();}
private:
  std::deque<TimedState> samples_;
};

struct Config
{
  double resolution{.10}, free_ttl{.30};
  double width{.62}, front{.76}, rear{.09}, margin{.15};
  double wheelbase{.595}, max_steer{.476}, steer_rate{.8}, steer_tau{.33};
  double cruise_speed{.6}, brake_decel{1.0}, brake_delay{.20};
  bool adaptive_speed{true};
  double crawl_speed{.20}, acceleration{.35}, planning_decel{.70};
  double observation_gap{.10}, processing_delay{.05}, longitudinal_margin{.20};
  double lateral_accel{.6}, preview{1.0}, horizon{6.0}, step{.25}, sample_step{.05};
  double cross_tolerance{.10}, yaw_tolerance{.34906585};
  double budget_ms{20.0};
  size_t max_nodes{6000}, max_cells{500000};
  unsigned completion_samples{3};
  void validate() const;
};

struct Projection {double station{}, cross{}, yaw{};};
struct Course
{
  std::string id;
  std::vector<Point> center, boundary;
  double entry{}, exit{};
  std::vector<double> stations;
  void validate();
  Projection project(Point p, double low, double high) const;
  Point at(double station) const;
  double length() const {return stations.empty() ? 0 : stations.back();}
};

struct Ray
{
  Point origin, end;
  double stamp{};
  bool hit{true};
};

struct GridSnapshot
{
  Point origin;
  double resolution{};
  int width{}, height{};
  // 0 unknown, 1 observed free, 2 occupied, 3 outside the course.
  std::vector<uint8_t> cells;
};

struct SearchEdge
{
  Point start, middle, end;
  uint8_t result{};  // 0 admitted, 1 body/dynamics, 2 braking, 3 direction, 4 dominated
};
struct PlanTrace
{
  size_t limit{96};
  std::vector<SearchEdge> edges;
  std::vector<size_t> counts = std::vector<size_t>(5, 0);
  // Selected opposing wall endpoints and their raw midpoint, followed by the
  // continuous corridor guide. These are geometry, not steering primitives.
  std::vector<Point> wall_left, wall_right, midpoints, corridor;
};

// Positive occupancy persists until two independent, newer clear observations.
// Unknown and expired free cells are forbidden; a new sensor cannot renew old cells.
class Grid
{
public:
  explicit Grid(Config config);
  void configure(const Course & course);
  void observe(const std::vector<Ray> & rays, uint64_t generation);
  void prepare(double now);
  bool clear(const State & state, double extra = 0) const;
  bool occupied(Point p) const;
  bool observed_free(Point p) const;
  bool blocks_gps(const Course & course, double low, double high) const;
  double clearance(const State & state) const;
  size_t size() const {return cells_.size();}
  double valid_until() const {return valid_until_;}
  GridSnapshot debug_snapshot(double now) const;
private:
  struct Cell
  {
    bool inside{}, hit{};
    double stamp{-1}, free_stamp{-1};
    uint64_t clear_generation{};
    unsigned clear_count{};
  };
  Config cfg_;
  Point origin_;
  int nx_{}, ny_{};
  std::vector<Cell> cells_;
  std::vector<int> distance_;
  bool prepared_{};
  double valid_until_{};
  int index(Point p) const;
  double point_clearance(Point p) const;
  double disc_clearance(const State & state) const;
};

enum class Phase : uint8_t {READY, PASS, REJOIN, EXIT_FOLLOW, HOLD, COMPLETE, WALL_FOLLOW};
struct Plan
{
  bool valid{}, complete{}, deadline_hit{}, reused{};
  bool perception_active{}, obstacle_detected{}, maneuver_active{}, gps_follow{};
  Phase phase{Phase::HOLD};
  std::string reason;
  uint64_t generation{}, id{};
  double station{}, compute_ms{}, min_clearance{}, stop_distance{};
  double speed_limit{}, valid_until{};
  std::vector<State> path;
  State reference;
  size_t expanded{};
};

class Planner
{
public:
  explicit Planner(Config config);
  void set_course(Course course);
  void set_zone(bool valid, bool in_zone);
  bool perception_required() const {return zone_valid_ && (in_zone_ || (entered_ && !completed_));}
  bool episode_active() const {return entered_ && !completed_;}
  Plan plan(const State & current, Grid & grid, double now, uint64_t generation,
    PlanTrace * trace = nullptr);
  // Predict the entire body during reaction, braking and a stationary margin.
  bool braking_clear(const State & current, const Grid & grid, double extra = 0) const;
  // Certify interpolated states between path samples, including their stop arcs.
  bool braking_transition_clear(const State & a,const State & b,const Grid & grid,
    unsigned depth = 0) const;
  double stopping_distance(double speed) const;
  const Course & course() const {return course_;}
  void reset();
private:
  State advance(State s, double target_steer, double distance, double speed) const;
  bool segment(State start, double target, double length, double speed,
    const Grid & grid, std::vector<State> & samples, double extra = 0) const;
  bool wall_corridor(const State & current,const Grid & grid,double goal,
    std::vector<Point> & guide,PlanTrace * trace,double lead,double smoothing,double side_bias,
    std::chrono::steady_clock::time_point deadline) const;
  Config cfg_;
  Course course_;
  double station_{};
  uint64_t generation_{}, plan_id_{};
  unsigned aligned_count_{};
  bool initialized_{}, completed_{};
  bool zone_valid_{}, in_zone_{}, entered_{};
  std::vector<Point> previous_corridor_;
  std::vector<State> previous_path_;
  PlanTrace previous_geometry_;
};
}  // namespace avoid_v2
