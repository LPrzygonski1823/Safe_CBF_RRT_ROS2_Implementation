#include "cbf_rrt_planner/ros_conversions.hpp"

namespace cbf_rrt_planner
{

    OccupancyGridData toGridData(const nav_msgs::msg::OccupancyGrid & msg)
    {
        OccupancyGridData grid;
        grid.width = static_cast<int>(msg.info.width);
        grid.height = static_cast<int>(msg.info.height);
        grid.resolution = msg.info.resolution;
        grid.origin_x = msg.info.origin.position.x;
        grid.origin_y = msg.info.origin.position.y;
        grid.data = msg.data;
        return grid;
    }

    nav_msgs::msg::Path toPathMsg(const PlanningResult & result, const std::string & frame_id, const rclcpp::Time & stamp)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = frame_id;
        path_msg.header.stamp = stamp;

        path_msg.poses.reserve(result.path.size());
        for (const auto & point : result.path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = frame_id;
            pose.header.stamp = stamp;
            pose.pose.position.x = point.first;
            pose.pose.position.y = point.second;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        return path_msg;
    }

}