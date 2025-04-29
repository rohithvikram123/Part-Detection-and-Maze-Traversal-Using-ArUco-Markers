cmake_minimum_required(VERSION 3.8)
project(maze_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include/maze_solver)

add_executable(maze_solver src/maze_solver.cpp src/main.cpp)
ament_target_dependencies(maze_solver rclcpp mage_msgs geometry_msgs std_msgs tf2_ros tf2 tf2_geometry_msgs ros2_aruco_interfaces nav_msgs)

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS maze_solver DESTINATION lib/${PROJECT_NAME})

ament_package()
