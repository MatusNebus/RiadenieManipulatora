cmake_minimum_required(VERSION 3.8)
project(pose_teacher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(abb_irb4600_ikfast REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  ${abb_irb4600_ikfast_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pose_teacher src/pose_teacher.cpp)
ament_target_dependencies(
  pose_teacher
  geometry_msgs
  interactive_markers
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)
target_link_libraries(pose_teacher ${abb_irb4600_ikfast_LIBRARIES})

install(TARGETS pose_teacher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
