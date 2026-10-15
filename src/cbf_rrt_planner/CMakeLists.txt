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
find_package(action_msgs REQUIRED)

# ROS-independent planner core, shared by the ROS node and the offline benchmark
add_library(cbf_rrt_planner_core STATIC
  src/obstacle_segmentation.cpp
  src/poisson_solver.cpp
  src/psf_generator.cpp
  src/collision_checker.cpp
  src/map_inflation.cpp
  src/map_loader.cpp
  src/spatial_grid.cpp
  src/rrt_star_planner.cpp
  src/planning_metrics.cpp
  src/path_smoother.cpp
  src/path_clearance.cpp
)

target_include_directories(cbf_rrt_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(cbf_rrt_planner_core PUBLIC cxx_std_17)

add_executable(planner_node
  src/planner_node.cpp
  src/ros_conversions.cpp
)

ament_target_dependencies(planner_node rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs std_srvs nav2_msgs rclcpp_action action_msgs)

target_link_libraries(planner_node cbf_rrt_planner_core Threads::Threads)

add_executable(cbf_benchmark
  src/cbf_benchmark.cpp
)

target_link_libraries(cbf_benchmark cbf_rrt_planner_core)

install(TARGETS
  planner_node
  cbf_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
