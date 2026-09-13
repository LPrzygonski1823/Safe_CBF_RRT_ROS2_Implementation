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
    // every pose is oriented along its outgoing segment, and the last one takes the requested
    // goal orientation - that is the yaw the Nav2 goal checker compares against
    // segments are resampled to pose_spacing because Nav2 controllers assume a dense path:
    // RPP discards plan poses farther than half the local costmap extent, so a raw RRT* path
    // (whose rewired edges reach neighbor_radius) can leave it with nothing to follow
    nav_msgs::msg::Path toPathMsg(
        const PlanningResult & result,
        const std::string & frame_id,
        const rclcpp::Time & stamp,
        const geometry_msgs::msg::Quaternion & goal_orientation,
        double pose_spacing);

}