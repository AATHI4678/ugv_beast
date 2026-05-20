cmake_minimum_required(VERSION 3.8)
project(ugv_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeliveryStatus.msg"
  "msg/BatteryState.msg"
  "msg/MotorStatus.msg"
  "msg/WifiStatus.msg"
  "srv/SetDeliveryWaypoints.srv"
  "srv/GetDeliveryStatus.srv"
  "srv/EmergencyStop.srv"
  "action/DeliverPackage.action"
  "action/ReturnHome.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs action_msgs
)

ament_package()
