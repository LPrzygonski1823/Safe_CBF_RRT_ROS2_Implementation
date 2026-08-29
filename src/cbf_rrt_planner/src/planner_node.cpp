#include <chrono>
#include <filesystem>
#include <future>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "cbf_rrt_planner/collision_checker.hpp"
#include "cbf_rrt_planner/planning_metrics.hpp"
#include "cbf_rrt_planner/psf_generator.hpp"
#include "cbf_rrt_planner/ros_conversions.hpp"
#include "cbf_rrt_planner/rrt_star_planner.hpp"

using namespace std::chrono_literals;

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("planner_node")
  {
    declareParameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("odom_topic").as_string(), 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
    );

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
    );

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    RCLCPP_INFO(
      this->get_logger(),
      "Planner node initialized, waiting for the map, odometry and goal...");
  }

private:
  // ROS2 params initialization
  void declareParameters()
  {
    this->declare_parameter<double>("step_size", 2.0);
    this->declare_parameter<int>("max_iterations", 2000);
    this->declare_parameter<double>("goal_bias", 0.05);
    this->declare_parameter<double>("goal_tolerance", 1.0);
    this->declare_parameter<double>("neighbor_radius", 5.0);
    this->declare_parameter<double>("gamma", 5.0);

    this->declare_parameter<bool>("enable_cbf", true);
    this->declare_parameter<double>("kappa", 20.0);
    this->declare_parameter<double>("safety_weight_c", 1.0);
    this->declare_parameter<double>("nominal_velocity", 1.0);
    this->declare_parameter<int>("cbf_samples_per_edge", 5);
    this->declare_parameter<int>("occupied_threshold", 65);

    this->declare_parameter<std::string>("metrics_csv_path", "/tmp/psf_debug/planning_metrics.csv");
    this->declare_parameter<std::string>("path_frame_id", "map");
    this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
  }

  cbf_rrt_planner::PlannerParams loadPlannerParams() const
  {
    cbf_rrt_planner::PlannerParams params;
    params.step_size = this->get_parameter("step_size").as_double();
    params.max_iterations = static_cast<int>(this->get_parameter("max_iterations").as_int());
    params.goal_bias = this->get_parameter("goal_bias").as_double();
    params.goal_tolerance = this->get_parameter("goal_tolerance").as_double();
    params.neighbor_radius = this->get_parameter("neighbor_radius").as_double();
    params.gamma = this->get_parameter("gamma").as_double();

    params.enable_cbf = this->get_parameter("enable_cbf").as_bool();
    params.kappa = this->get_parameter("kappa").as_double();
    params.safety_weight_c = this->get_parameter("safety_weight_c").as_double();
    params.nominal_velocity = this->get_parameter("nominal_velocity").as_double();
    params.cbf_samples_per_edge =
      static_cast<int>(this->get_parameter("cbf_samples_per_edge").as_int());
    params.occupied_threshold =
      static_cast<int>(this->get_parameter("occupied_threshold").as_int());
    return params;
  }

  // tf: function used by odomCallback and goalCallback
  // transforms an arbitrary PoseStamped into the "map" coordinate frame
  std::optional<geometry_msgs::msg::PoseStamped> transformToMapFrame(
    const geometry_msgs::msg::PoseStamped & pose_in) const
  {
    if (pose_in.header.frame_id == "map" || pose_in.header.frame_id.empty()) {
      return pose_in;
    }

    geometry_msgs::msg::PoseStamped pose_out;
    try {
      geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform("map", pose_in.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(pose_in, pose_out, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "TF transform '%s' -> 'map' failed: %s",
        pose_in.header.frame_id.c_str(), ex.what());
      return std::nullopt;
    }
    return pose_out;
  }

  // map: asynchronous psf
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    bool first_time = !map_received_;
    current_grid_data_ = cbf_rrt_planner::toGridData(*msg);
    map_received_ = true;

    if (first_time) {
      RCLCPP_INFO(
        this->get_logger(),
        "First map received: %d x %d (resolution %.3f m/cell). "
        "Waiting for a goal to trigger PSF generation.",
        current_grid_data_.width, current_grid_data_.height, current_grid_data_.resolution);
    }
  }

  void triggerPsfGenerationIfNeeded()
  {
    if (psf_generation_triggered_ || !map_received_) {return;}
    psf_generation_triggered_ = true;

    auto grid_data_snapshot = current_grid_data_;
    RCLCPP_INFO(
      this->get_logger(),
      "Freezing map at %d x %d and starting PSF generation (one-time, asynchronous)...",
      grid_data_snapshot.width, grid_data_snapshot.height);

    psf_future_ = std::async(
      std::launch::async,
      [this, grid_data_snapshot]() {
        psf_generator_.generate(grid_data_snapshot);
      });
  }

  void pollPsfReadiness()
  {
    if (psf_ready_ || !psf_future_.valid()) {return;}

    if (psf_future_.wait_for(0s) == std::future_status::ready) {
      try {
        psf_future_.get();
        psf_ready_ = true;

        std::filesystem::create_directories("/tmp/psf_debug");
        psf_generator_.exportToCsv("/tmp/psf_debug");
        RCLCPP_INFO(
          this->get_logger(), "PSF ready (%zu obstacles). Exported to /tmp/psf_debug.",
          psf_generator_.obstacleCount());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "PSF generation failed: %s", e.what());
      }
    }
  }

  // odometry
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose_in;
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;

    auto pose_in_map = transformToMapFrame(pose_in);
    if (!pose_in_map) {
      return;
    }

    current_x_ = pose_in_map->pose.position.x;
    current_y_ = pose_in_map->pose.position.y;
    has_odom_ = true;
  }

  // goal transformed to map
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto goal_in_map = transformToMapFrame(*msg);
    if (!goal_in_map) {
      RCLCPP_WARN(this->get_logger(), "Could not transform goal pose to map frame - ignoring.");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "New goal received: [X: %.2f, Y: %.2f] (frame: map)",
      goal_in_map->pose.position.x, goal_in_map->pose.position.y);

    if (!map_received_) {
      RCLCPP_WARN(this->get_logger(), "Goal received but no map yet - ignoring.");
      return;
    }

    cbf_rrt_planner::PlannerParams params = loadPlannerParams();

    if (params.enable_cbf) {
      triggerPsfGenerationIfNeeded();
      pollPsfReadiness();

      if (!psf_ready_) {
        RCLCPP_WARN(
          this->get_logger(),
          "PSF still computing (triggered by this or an earlier goal request) - "
          "try sending the goal again in a moment.");
        return;
      }
    }

    if (!has_odom_) {
      RCLCPP_WARN(this->get_logger(), "Goal received but no odometry yet - ignoring.");
      return;
    }

    cbf_rrt_planner::CollisionChecker checker(current_grid_data_, params.occupied_threshold);
    const cbf_rrt_planner::PSFGenerator * psf_ptr = params.enable_cbf ? &psf_generator_ : nullptr;

    cbf_rrt_planner::RrtStarPlanner planner(checker, psf_ptr, params);

    RCLCPP_INFO(
      this->get_logger(), "Planning from (%.2f, %.2f) to (%.2f, %.2f)...",
      current_x_, current_y_, goal_in_map->pose.position.x, goal_in_map->pose.position.y);

    auto result = planner.plan(
      current_x_, current_y_,
      goal_in_map->pose.position.x, goal_in_map->pose.position.y);

    if (!result.success) {
      RCLCPP_WARN(
        this->get_logger(),
        "Planning FAILED after %d iterations - no path found.", result.iterations_used);
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Planning SUCCESS: length=%.2f mean_h=%.4f points=%zu rejection_ratio=%.3f",
      result.total_length, result.mean_h, result.path.size(), result.rejectionRatio());

    auto path_msg = cbf_rrt_planner::toPathMsg(
      result, this->get_parameter("path_frame_id").as_string(), this->now());
    path_pub_->publish(path_msg);

    std::string csv_path = this->get_parameter("metrics_csv_path").as_string();
    std::filesystem::create_directories(std::filesystem::path(csv_path).parent_path());
    std::string label = params.enable_cbf ?
      ("cbf_c" + std::to_string(params.safety_weight_c)) : "baseline_rrt_star";
    cbf_rrt_planner::PlanningMetricsLogger::appendResult(csv_path, label, result);
  }

  // subs/publishers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // tf
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // inner state
  cbf_rrt_planner::PSFGenerator psf_generator_;
  cbf_rrt_planner::OccupancyGridData current_grid_data_;
  bool map_received_ = false;
  std::future<void> psf_future_;
  bool psf_generation_triggered_ = false;
  bool psf_ready_ = false;

  double current_x_ = 0.0;
  double current_y_ = 0.0;
  bool has_odom_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}