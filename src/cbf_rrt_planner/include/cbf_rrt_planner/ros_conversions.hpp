#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

#include "cbf_rrt_planner/occupancy_grid_data.hpp"
#include "cbf_rrt_planner/rrt_star_types.hpp"

namespace cbf_rrt_planner
{
    // ONLY PLACE where planner logic connects to ROS2

    // conversion of an occupancy map from the ROS2 type to a lightweight representation used by the PSF/RRT* modules
    OccupancyGridData toGridData(const nav_msgs::msg::OccupancyGrid & msg);

    // conversion of the planning result to ROS2
    nav_msgs::msg::Path toPathMsg(const PlanningResult & result, const std::string & frame_id, const rclcpp::Time & stamp);

}