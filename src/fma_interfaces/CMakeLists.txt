cmake_minimum_required(VERSION 3.8)
project(fma_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RefPoint.msg"
  "msg/TargetRef.msg"
  "msg/VehicleVector.msg"
  "msg/LanePath.msg"
  "msg/GpsPath.msg"
  "msg/GpsRoute.msg"
  "msg/RouteFeedback.msg"
  "msg/RouteControl.msg"
  "msg/AvoidStatus.msg"
  "msg/AvoidCourse.msg"
  "msg/AvoidPlan.msg"
  "msg/ParkingStatus.msg"
  "msg/ParkingWallStatus.msg"
  "msg/ParkingCommand.msg"
  "msg/MissionObservation.msg"
  "msg/TrafficStop.msg"
  "msg/StopLine.msg"
  "msg/EstopRequest.msg"
  "msg/CanHealth.msg"
  "msg/MgmState.msg"
  "msg/ZoneContext.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
