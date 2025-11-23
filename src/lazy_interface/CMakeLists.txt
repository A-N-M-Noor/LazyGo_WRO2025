cmake_minimum_required(VERSION 3.8)
project(lazy_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectionTowerInfo.msg"
  "msg/DetectionTowerList.msg"
  "msg/LidarTowerInfo.msg"
  "msg/GlobalTowerInfo.msg"
  "msg/LidarTowerList.msg"
  "msg/BotDebugInfo.msg"
  "msg/BotLidarInfo.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
