cmake_minimum_required(VERSION 3.8)
project(fma_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RefPoint.msg"
  "msg/TargetRef.msg"
  "msg/VehicleVector.msg"
  "msg/LanePath.msg"
  "msg/GpsPath.msg"
  "msg/AvoidStatus.msg"
  "msg/ParkingStatus.msg"
  "msg/TrafficStop.msg"
  "msg/StopLine.msg"
  "msg/EstopRequest.msg"
  "msg/CanHealth.msg"
  DEPENDENCIES std_msgs
)

ament_package()
