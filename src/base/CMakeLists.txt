cmake_minimum_required(VERSION 3.8)
project(ros2_mecabot2_sim_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Build directories
install(DIRECTORY
  launch
  robot_desc
  config
  rviz2
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
