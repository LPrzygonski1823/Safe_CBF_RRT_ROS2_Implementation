#include <chrono>
#include <filesystem>
#include <future>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "cbf_rrt_planner/path_smoother.hpp"
#include "cbf_rrt_planner/cbf_utils.hpp"

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

    rclcpp::QoS map_qos(10);
    map_qos.transient_local();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("odom_topic").as_string(), 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
    );

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cbf_goal_pose", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
    );

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

    refresh_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/refresh_map",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!map_received_) {
          response->success = false;
          response->message = "No map received yet - there is nothing to freeze.";
          return;
        }

        if (psf_future_.valid() && psf_future_.wait_for(0s) != std::future_status::ready) {
          response->success = false;
          response->message = "PSF generation currently in progress - cannot refresh yet.";
          return;
        }
        
        psf_generation_triggered_ = false;
        psf_ready_ = false;
        
        triggerPsfGenerationIfNeeded(this->get_parameter("occupied_threshold").as_int());

        response->success = true;
        response->message = "Map re-frozen and PSF generation started in background.";
        RCLCPP_INFO(this->get_logger(), "Map refresh triggered manually.");
      });


    // the Nav2 stack is brought up with use_sim_time:=True - a mismatch here makes every
    // stamp comparison against odometry and TF meaningless
    if (!this->get_parameter("use_sim_time").as_bool()) {
      RCLCPP_WARN(
        this->get_logger(),
        "use_sim_time is false - pass '-p use_sim_time:=true' when running against the Gazebo stack.");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Planner node initialized, waiting for the map, odometry and goal...");
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PlannerNode::pollPsfReadiness, this));
  }

private:
  // ROS2 params initialization
  void declareParameters()
  {
    this->declare_parameter<double>("step_size", 1.0);
    this->declare_parameter<int>("max_iterations", 10000);
    this->declare_parameter<double>("goal_bias", 0.05);
    this->declare_parameter<double>("goal_tolerance", 1.0);
    this->declare_parameter<double>("neighbor_radius", 5.0);
    this->declare_parameter<double>("gamma", 5.0);

    this->declare_parameter<bool>("enable_cbf", true);
    this->declare_parameter<double>("kappa", 5.0);
    this->declare_parameter<double>("safety_weight_c", 0.2);
    this->declare_parameter<double>("nominal_velocity", 1.0);
    this->declare_parameter<int>("occupied_threshold", 65);
    this->declare_parameter<bool>("enable_smoothing", false);

    this->declare_parameter<std::string>("metrics_csv_path", "/workspace/src/cbf_rrt_planner/maps/psf_debug/planning_metrics.csv");
    this->declare_parameter<std::string>("path_frame_id", "map");
    this->declare_parameter<double>("path_pose_spacing", 0.05); // resampling of the published path
    this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    this->declare_parameter<double>("odom_timeout", 1.0); // max age [s] of the pose used as planning start
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
    params.edge_sample_step = current_grid_data_.resolution * 0.5;

    params.enable_cbf = this->get_parameter("enable_cbf").as_bool();
    params.kappa = this->get_parameter("kappa").as_double();
    params.safety_weight_c = this->get_parameter("safety_weight_c").as_double();
    params.nominal_velocity = this->get_parameter("nominal_velocity").as_double();
    params.occupied_threshold =
      static_cast<int>(this->get_parameter("occupied_threshold").as_int());
    return params;
  }

  // tf: function used by odomCallback and goalCallback
  // transforms an arbitrary PoseStamped into the "map" coordinate frame
  std::optional<geometry_msgs::msg::PoseStamped> transformToMapFrame(
    const geometry_msgs::msg::PoseStamped & pose_in)
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
      // throttled: odometry arrives at tens of Hz, and before SLAM publishes map->odom
      // every single message would fail and flood the log
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "TF transform '%s' -> 'map' failed: %s",
        pose_in.header.frame_id.c_str(), ex.what());
      return std::nullopt;
    }
    return pose_out;
  }

  // map: asynchronous psf
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    current_grid_data_ = cbf_rrt_planner::toGridData(*msg);
    if (!map_received_) {
      RCLCPP_INFO(this->get_logger(), "First map received. Awaiting /refresh_map to generate PSF.");
      map_received_ = true;
    }
  }

  void triggerPsfGenerationIfNeeded(int occupied_threshold)
  {
    if (psf_generation_triggered_ || !map_received_) {return;}

    frozen_origin_x_ = current_grid_data_.origin_x;
    frozen_origin_y_ = current_grid_data_.origin_y;
    frozen_width_ = current_grid_data_.width;
    frozen_height_ = current_grid_data_.height;

    psf_generation_triggered_ = true;
    
    // safe: applied before async thread starts
    psf_generator_.setOccupiedThreshold(occupied_threshold);

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

        std::string debug_dir = "/workspace/src/cbf_rrt_planner/maps/psf_debug";
        std::filesystem::create_directories(debug_dir);
        psf_generator_.exportToCsv(debug_dir);
        RCLCPP_INFO(
          this->get_logger(), "PSF ready (%zu obstacles). Exported.",
          psf_generator_.obstacleCount());
      } catch (const std::exception & e) {
        // the future is consumed at this point, so without clearing the trigger flag the node
        // would stay stuck reporting "generation in progress" forever
        psf_generation_triggered_ = false;
        RCLCPP_ERROR(
          this->get_logger(), "PSF generation failed: %s - will be retried on the next goal.",
          e.what());
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
      // the previous pose is kept, but last_odom_stamp_ is not refreshed, so goalCallback
      // will refuse to plan once it goes stale
      return;
    }

    current_x_ = pose_in_map->pose.position.x;
    current_y_ = pose_in_map->pose.position.y;
    last_odom_stamp_ = msg->header.stamp;
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

    // the PSF is needed to enforce the CBF condition, and useful even for the baseline run,
    // where it still supplies the mean_h and rejection ratio columns of the paper
    pollPsfReadiness();

    if (!psf_ready_ && !psf_generation_triggered_) {
      triggerPsfGenerationIfNeeded(params.occupied_threshold);
    }

    if (!psf_ready_) {
      if (params.enable_cbf) {
        RCLCPP_WARN(
          this->get_logger(),
          "PSF generation in progress - send the goal again once it is ready.");
        return;
      }
      RCLCPP_WARN(
        this->get_logger(),
        "PSF not ready - baseline will run without the mean_h and rejection ratio metrics.");
    }

    if (psf_ready_) {
      double tolerance = current_grid_data_.resolution * 0.5;

      if (current_grid_data_.width != frozen_width_ ||
          current_grid_data_.height != frozen_height_ ||
          std::abs(current_grid_data_.origin_x - frozen_origin_x_) > tolerance ||
          std::abs(current_grid_data_.origin_y - frozen_origin_y_) > tolerance) {
          
          RCLCPP_WARN(
            this->get_logger(),
            "MAP MISMATCH DETECTED! Live map (origin: %.2f, %.2f, size: %dx%d) differs from frozen PSF map (origin: %.2f, %.2f, size: %dx%d). "
            "CBF evaluations will be physically shifted and inaccurate! Please call /refresh_map.",
            current_grid_data_.origin_x, current_grid_data_.origin_y, current_grid_data_.width, current_grid_data_.height,
            frozen_origin_x_, frozen_origin_y_, frozen_width_, frozen_height_);
      }
    }

    if (!has_odom_) {
      RCLCPP_WARN(this->get_logger(), "Goal received but no odometry yet - ignoring.");
      return;
    }

    double odom_timeout = this->get_parameter("odom_timeout").as_double();
    double odom_age = (this->now() - last_odom_stamp_).seconds();
    if (odom_age > odom_timeout) {
      RCLCPP_WARN(
        this->get_logger(),
        "Odometry is %.2f s old (limit %.2f s) - refusing to plan from an outdated start pose.",
        odom_age, odom_timeout);
      return;
    }

    cbf_rrt_planner::CollisionChecker checker(current_grid_data_, params.occupied_threshold);
    const cbf_rrt_planner::PSFGenerator * psf_ptr = psf_ready_ ? &psf_generator_ : nullptr;

    cbf_rrt_planner::RrtStarPlanner planner(checker, psf_ptr, params);

    RCLCPP_INFO(
      this->get_logger(), "Planning from (%.2f, %.2f) to (%.2f, %.2f)...",
      current_x_, current_y_, goal_in_map->pose.position.x, goal_in_map->pose.position.y);

    auto result = planner.plan(
      current_x_, current_y_,
      goal_in_map->pose.position.x, goal_in_map->pose.position.y);

    if (!result.success) {
      switch (result.failure) {
        case cbf_rrt_planner::PlanningFailure::StartBlocked:
          RCLCPP_WARN(
            this->get_logger(),
            "Planning REJECTED: start (%.2f, %.2f) falls into an occupied or unknown cell.",
            current_x_, current_y_);
          break;
        case cbf_rrt_planner::PlanningFailure::GoalBlocked:
          RCLCPP_WARN(
            this->get_logger(),
            "Planning REJECTED: goal (%.2f, %.2f) falls into an occupied or unknown cell.",
            goal_in_map->pose.position.x, goal_in_map->pose.position.y);
          break;
        default:
          RCLCPP_WARN(
            this->get_logger(),
            "Planning FAILED after %d iterations - no path found.", result.iterations_used);
          break;
      }
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Planning SUCCESS: length=%.2f mean_h=%.4f min_h=%.4f points=%zu rejection_ratio=%.3f "
      "(first solution at iteration %d, refined over %d iterations)",
      result.total_length, result.mean_h, result.min_h, result.path.size(), result.rejectionRatio(),
      result.iterations_to_first_solution, result.iterations_used);

    bool enable_smoothing = this->get_parameter("enable_smoothing").as_bool();

    if (enable_smoothing) {
      cbf_rrt_planner::PathSmoother smoother(checker, psf_ptr, params);
      result.path = smoother.smoothPath(result.path);
      RCLCPP_INFO(this->get_logger(), "Path smoothed. Remaining points: %zu", result.path.size());

      double final_len = 0.0;
      double final_weighted_h_sum = 0.0;
      for (size_t i = 0; i + 1 < result.path.size(); ++i) {
          double x1 = result.path[i].first, y1 = result.path[i].second;
          double x2 = result.path[i + 1].first, y2 = result.path[i + 1].second;
          double seg_len = std::hypot(x2 - x1, y2 - y1);
          final_len += seg_len;
          if (psf_ptr != nullptr) {
              final_weighted_h_sum += cbf_rrt_planner::computeAverageH(*psf_ptr, x1, y1, x2, y2, params.edge_sample_step) * seg_len;
          }
      }
      result.total_length = final_len;
      if (psf_ptr != nullptr && final_len > 1e-9) {
          result.mean_h = final_weighted_h_sum / final_len;
      }
    }

    std::string csv_path = this->get_parameter("metrics_csv_path").as_string();
    std::filesystem::create_directories(std::filesystem::path(csv_path).parent_path());
    std::string label = params.enable_cbf ? ("cbf_c" + std::to_string(params.safety_weight_c)) : "baseline_rrt_star";
    cbf_rrt_planner::PlanningMetricsLogger::appendResult(csv_path, label, result);

    auto path_msg = cbf_rrt_planner::toPathMsg(
      result, this->get_parameter("path_frame_id").as_string(), this->now(),
      goal_in_map->pose.orientation, this->get_parameter("path_pose_spacing").as_double());
    path_pub_->publish(path_msg);

    if (!follow_path_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(this->get_logger(), "FollowPath action server not available - is controller_server running?");
      return;
    }

    cancelActiveFollowPathGoal();

    auto goal_msg = FollowPath::Goal();
    goal_msg.path = path_msg;
    goal_msg.controller_id = "FollowPath"; // nazwa pluginu zdefiniowana w nav2_rpp_params.yaml

    rclcpp_action::Client<FollowPath>::SendGoalOptions send_goal_options;

    // without this a rejected goal produces no output at all
    send_goal_options.goal_response_callback =
        [this](FollowPathGoalHandle::SharedPtr goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(
          this->get_logger(),
          "FollowPath: goal REJECTED by controller_server - check its log for the reason.");
        return;
      }
      follow_path_goal_handle_ = goal_handle;
      RCLCPP_INFO(this->get_logger(), "FollowPath: goal accepted, the controller is driving.");
    };

    send_goal_options.feedback_callback =
        [this](FollowPathGoalHandle::SharedPtr,
               const std::shared_ptr<const FollowPath::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "FollowPath: %.2f m to go at %.2f m/s.", feedback->distance_to_goal, feedback->speed);
    };

    send_goal_options.result_callback =
        [this](const FollowPathGoalHandle::WrappedResult &result)
    {
      follow_path_goal_handle_.reset();

      switch (result.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "FollowPath: robot reached the end of the path.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(
          this->get_logger(),
          "FollowPath: ABORTED by controller_server - its log holds the actual reason "
          "(progress checker, collision or controller exception).");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "FollowPath: canceled.");
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "FollowPath: finished with an unknown result code.");
        break;
      }
    };

    follow_path_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Sent path to controller_server (FollowPath action).");
  }

  // a previous goal left running would race the new one and surface as an ambiguous
  // ABORTED/CANCELED, so it is cancelled explicitly before sending the next path
  void cancelActiveFollowPathGoal()
  {
    if (!follow_path_goal_handle_) {return;}

    int8_t status = follow_path_goal_handle_->get_status();
    if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
    {
      RCLCPP_INFO(this->get_logger(), "Canceling the FollowPath goal still in progress.");
      follow_path_client_->async_cancel_goal(follow_path_goal_handle_);
    }
    follow_path_goal_handle_.reset();
  }
  
  // subs/publishers
  using FollowPath = nav2_msgs::action::FollowPath;
  using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  FollowPathGoalHandle::SharedPtr follow_path_goal_handle_; // currently executing goal, if any
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr refresh_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

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

  double frozen_origin_x_ = 0.0;
  double frozen_origin_y_ = 0.0;
  int frozen_width_ = 0;
  int frozen_height_ = 0;

  double current_x_ = 0.0;
  double current_y_ = 0.0;
  bool has_odom_ = false;
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME}; // stamp of the pose behind current_x_/current_y_
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}