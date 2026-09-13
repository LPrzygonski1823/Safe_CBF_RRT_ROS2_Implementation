#include "cbf_rrt_planner/ros_conversions.hpp"

#include <cmath>

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

    nav_msgs::msg::Path toPathMsg(
        const PlanningResult & result,
        const std::string & frame_id,
        const rclcpp::Time & stamp,
        const geometry_msgs::msg::Quaternion & goal_orientation,
        double pose_spacing)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = frame_id;
        path_msg.header.stamp = stamp;

        if (result.path.empty()) {return path_msg;}

        auto make_pose = [&](double x, double y, double yaw) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = frame_id;
            pose.header.stamp = stamp;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;
            // planar rotation about z, so the quaternion reduces to these two terms
            pose.pose.orientation.z = std::sin(yaw * 0.5);
            pose.pose.orientation.w = std::cos(yaw * 0.5);
            return pose;
        };

        const double spacing = std::max(pose_spacing, 1e-3);
        path_msg.poses.reserve(static_cast<size_t>(result.total_length / spacing) + result.path.size());

        double previous_yaw = 0.0;
        for (size_t i = 0; i + 1 < result.path.size(); ++i) {
            const double x1 = result.path[i].first, y1 = result.path[i].second;
            const double x2 = result.path[i + 1].first, y2 = result.path[i + 1].second;
            const double seg_length = std::hypot(x2 - x1, y2 - y1);

            // a degenerate segment carries no heading - keep the previous one
            double yaw = seg_length < 1e-9 ? previous_yaw : std::atan2(y2 - y1, x2 - x1);
            previous_yaw = yaw;

            // the segment endpoint is emitted by the next iteration, or after the loop for the goal
            int steps = std::max(1, static_cast<int>(std::ceil(seg_length / spacing)));
            for (int s = 0; s < steps; ++s) {
                double t = static_cast<double>(s) / steps;
                path_msg.poses.push_back(make_pose(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t, yaw));
            }
        }

        auto goal_pose = make_pose(result.path.back().first, result.path.back().second, previous_yaw);
        goal_pose.pose.orientation = goal_orientation;
        path_msg.poses.push_back(goal_pose);

        return path_msg;
    }

}