cmake_minimum_required(VERSION 3.8)
project(touch_pack_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
# geometry_msgs/Point[]: lista de waypoints do modo MATRIX_MAP.
find_package(geometry_msgs REQUIRED)
# builtin_interfaces/Time: carimbo de emissão do PalpationStart, que
# distingue um comando novo do latch TRANSIENT_LOCAL de outra sessão.
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PalpationStart.msg"
  "msg/PalpationStatus.msg"
  "msg/SyncedTouch.msg"
  "msg/MatrixPoint.msg"
  "msg/LoadCellSample.msg"
  "msg/TouchFrame.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
