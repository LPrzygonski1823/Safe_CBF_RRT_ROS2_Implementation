#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cbf_rrt_planner/psf_generator.hpp"

#include <filesystem>

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("planner_node")
  {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
    );

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Planner node initialized, waiting for the map and goal...");
  }

private:
  // conversion from ROS2 type (nav_msgs::msg::OccupancyGrid) to light, ROS2-independent representation used by PSF module
  // math logic separated from rclcpp
  cbf_rrt_planner::OccupancyGridData toGridData(const nav_msgs::msg::OccupancyGrid & msg) const
  {
    cbf_rrt_planner::OccupancyGridData grid;
    grid.width = static_cast<int>(msg.info.width);
    grid.height = static_cast<int>(msg.info.height);
    grid.resolution = msg.info.resolution;
    grid.origin_x = msg.info.origin.position.x;
    grid.origin_y = msg.info.origin.position.y;
    grid.data = msg.data;
    return grid;
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(), "Received map with size: %d x %d",
      msg->info.width, msg->info.height);

    // NOTE (known limitation, to be fixed in the integration phase / Module C4):
    // this call is SYNCHRONOUS and will block the executor for the duration of the PDE
    // calculations (2x Laplace + 1x Poisson). In the long term, it should be moved to std::async
    // so that receiving other messages (e.g. /goal_pose) is not blocked during that time.
    auto grid_data = toGridData(*msg);
    psf_generator_.generate(grid_data);

    RCLCPP_INFO(
      this->get_logger(), "PSF generated. Detected %zu obstacles.",
      psf_generator_.obstacleCount());

    // export to csv for visualisation (tools/visualize_psf.py)
    std::filesystem::create_directories("/tmp/psf_debug");
    psf_generator_.exportToCsv("/tmp/psf_debug");
    RCLCPP_INFO(this->get_logger(), "PSF fields exported to /tmp/psf_debug for visualization.");
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(), "New goal received: [X: %.2f, Y: %.2f]",
      msg->pose.position.x, msg->pose.position.y);

    if (!psf_generator_.isReady()) {
      RCLCPP_WARN(this->get_logger(), "Goal received but PSF is not ready yet - ignoring.");
      return;
    }

    // temp overview - module B input (Safe-CBF-RRT*)
    double h = psf_generator_.getH(msg->pose.position.x, msg->pose.position.y);
    RCLCPP_INFO(this->get_logger(), "Safety value h at goal: %.6f", h);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  cbf_rrt_planner::PSFGenerator psf_generator_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}