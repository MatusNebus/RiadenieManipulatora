cmake_minimum_required(VERSION 3.8)
project(model_spawner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(model_spawner src/model_spawner.cpp)
ament_target_dependencies(model_spawner rclcpp visualization_msgs)

install(TARGETS model_spawner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
