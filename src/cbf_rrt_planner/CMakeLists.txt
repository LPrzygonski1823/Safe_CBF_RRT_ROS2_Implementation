cmake_minimum_required(VERSION 3.8)
project(cbf_rrt_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(planner_node
  src/planner_node.cpp
  src/obstacle_segmentation.cpp
  src/poisson_solver.cpp
  src/psf_generator.cpp
  src/collision_checker.cpp
  src/spatial_grid.cpp
  src/rrt_star_planner.cpp
  src/planning_metrics.cpp
  src/ros_conversions.cpp
  src/path_smoother.cpp
)

target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(planner_node PUBLIC cxx_std_17)

ament_target_dependencies(planner_node rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs std_srvs nav2_msgs rclcpp_action)

target_link_libraries(planner_node Threads::Threads)

install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()