cmake_minimum_required(VERSION 3.8)
project(openarm_mtc_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mtc_node src/mtc_node.cpp)
ament_target_dependencies(mtc_node moveit_task_constructor_core rclcpp)
target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)

add_executable(mtc_node_air_pickplace src/mtc_node_air_pickplace.cpp)
ament_target_dependencies(mtc_node_air_pickplace moveit_task_constructor_core rclcpp)
target_compile_features(mtc_node_air_pickplace PUBLIC c_std_99 cxx_std_17)

install(TARGETS mtc_node mtc_node_air_pickplace
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
