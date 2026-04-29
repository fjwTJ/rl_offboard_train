#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ReturnHeightCoordinatorNode : public rclcpp::Node
{
public:
  ReturnHeightCoordinatorNode()
  : Node("return_height_coordinator_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("main_odometry_topic", "/fmu/out/vehicle_odometry");
    declare_parameter<std::string>("target_odometry_topic", "/px4_2/fmu/out/vehicle_odometry");
    declare_parameter<std::string>("main_state_active_topic", "/uav/state_active");
    declare_parameter<std::string>("target_state_active_topic", "/target_uav/state_active");
    declare_parameter<std::string>("reset_topic", "/rl/reset");
    declare_parameter<std::string>("target_topic", "/perception/target_xyz");
    declare_parameter<std::string>("control_frame", "base_link_frd");
    declare_parameter<std::string>("main_return_alt_topic", "/rl/main_return_alt");
    declare_parameter<std::string>("target_return_alt_topic", "/rl/target_return_alt");
    declare_parameter<std::string>("return_height_aligned_topic", "/rl/return_height_aligned");
    declare_parameter<std::string>("mission_start_ready_topic", "/rl/mission_start_ready");
    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<double>("base_return_alt", 5.0);
    declare_parameter<double>("height_diff_tolerance", 0.25);
    declare_parameter<double>("return_alt_tolerance", 0.35);
    declare_parameter<double>("home_xy_tolerance", 0.5);
    declare_parameter<double>("max_alt_adjust", 0.8);
    declare_parameter<double>("min_return_alt", 4.0);
    declare_parameter<double>("max_return_alt", 6.0);
    declare_parameter<double>("perception_z_tolerance", 0.25);
    declare_parameter<double>("perception_adjust_gain", 0.4);
    declare_parameter<double>("target_timeout_sec", 0.5);
    declare_parameter<double>("odom_timeout_sec", 0.5);
    declare_parameter<double>("state_timeout_sec", 0.5);
    declare_parameter<double>("tf_timeout_sec", 0.08);
    declare_parameter<int>("stable_ticks_required", 5);

    main_odometry_topic_ = get_parameter("main_odometry_topic").as_string();
    target_odometry_topic_ = get_parameter("target_odometry_topic").as_string();
    main_state_active_topic_ = get_parameter("main_state_active_topic").as_string();
    target_state_active_topic_ = get_parameter("target_state_active_topic").as_string();
    reset_topic_ = get_parameter("reset_topic").as_string();
    target_topic_ = get_parameter("target_topic").as_string();
    control_frame_ = get_parameter("control_frame").as_string();
    main_return_alt_topic_ = get_parameter("main_return_alt_topic").as_string();
    target_return_alt_topic_ = get_parameter("target_return_alt_topic").as_string();
    return_height_aligned_topic_ = get_parameter("return_height_aligned_topic").as_string();
    mission_start_ready_topic_ = get_parameter("mission_start_ready_topic").as_string();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    base_return_alt_ = get_parameter("base_return_alt").as_double();
    height_diff_tolerance_ = get_parameter("height_diff_tolerance").as_double();
    return_alt_tolerance_ = get_parameter("return_alt_tolerance").as_double();
    home_xy_tolerance_ = get_parameter("home_xy_tolerance").as_double();
    max_alt_adjust_ = get_parameter("max_alt_adjust").as_double();
    min_return_alt_ = get_parameter("min_return_alt").as_double();
    max_return_alt_ = get_parameter("max_return_alt").as_double();
    perception_z_tolerance_ = get_parameter("perception_z_tolerance").as_double();
    perception_adjust_gain_ = get_parameter("perception_adjust_gain").as_double();
    target_timeout_sec_ = get_parameter("target_timeout_sec").as_double();
    odom_timeout_sec_ = get_parameter("odom_timeout_sec").as_double();
    state_timeout_sec_ = get_parameter("state_timeout_sec").as_double();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
    stable_ticks_required_ = std::max(1, static_cast<int>(get_parameter("stable_ticks_required").as_int()));

    if (max_return_alt_ < min_return_alt_) {
      std::swap(max_return_alt_, min_return_alt_);
    }
    base_return_alt_ = std::clamp(base_return_alt_, min_return_alt_, max_return_alt_);

    auto sensor_qos = rclcpp::SensorDataQoS();
    main_odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      main_odometry_topic_, sensor_qos,
      std::bind(&ReturnHeightCoordinatorNode::main_odom_callback, this, std::placeholders::_1));
    target_odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      target_odometry_topic_, sensor_qos,
      std::bind(&ReturnHeightCoordinatorNode::target_odom_callback, this, std::placeholders::_1));
    main_state_sub_ = create_subscription<std_msgs::msg::String>(
      main_state_active_topic_, 10,
      std::bind(&ReturnHeightCoordinatorNode::main_state_callback, this, std::placeholders::_1));
    target_state_sub_ = create_subscription<std_msgs::msg::String>(
      target_state_active_topic_, 10,
      std::bind(&ReturnHeightCoordinatorNode::target_state_callback, this, std::placeholders::_1));
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
      reset_topic_, 10,
      std::bind(&ReturnHeightCoordinatorNode::reset_callback, this, std::placeholders::_1));
    target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic_, 10,
      std::bind(&ReturnHeightCoordinatorNode::target_callback, this, std::placeholders::_1));

    main_return_alt_pub_ = create_publisher<std_msgs::msg::Float32>(main_return_alt_topic_, 10);
    target_return_alt_pub_ = create_publisher<std_msgs::msg::Float32>(target_return_alt_topic_, 10);
    return_height_aligned_pub_ = create_publisher<std_msgs::msg::Bool>(return_height_aligned_topic_, 10);
    mission_start_ready_pub_ = create_publisher<std_msgs::msg::Bool>(mission_start_ready_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ReturnHeightCoordinatorNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "return_height_coordinator_node started: main_odom=%s target_odom=%s base_alt=%.2f",
      main_odometry_topic_.c_str(),
      target_odometry_topic_.c_str(),
      base_return_alt_);
  }

private:
  struct OdomState
  {
    double x{0.0};
    double y{0.0};
    double alt{0.0};
    double stamp_sec{-1.0};
    bool ready{false};
  };

  void main_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    update_odom(*msg, main_odom_);
  }

  void target_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    update_odom(*msg, target_odom_);
  }

  void update_odom(const px4_msgs::msg::VehicleOdometry &msg, OdomState &state)
  {
    if (std::isnan(msg.position[0]) || std::isnan(msg.position[1]) || std::isnan(msg.position[2])) {
      return;
    }
    state.x = msg.position[0];
    state.y = msg.position[1];
    state.alt = -msg.position[2];
    state.stamp_sec = now_sec();
    state.ready = true;
  }

  void main_state_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    main_state_ = msg->data;
    last_main_state_sec_ = now_sec();
  }

  void target_state_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    target_state_ = msg->data;
    last_target_state_sec_ = now_sec();
  }

  void reset_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      in_reset_cycle_ = true;
      stable_ticks_ = 0;
      odom_alignment_latched_ = false;
      mission_ready_latched_ = false;
      main_return_alt_ = base_return_alt_;
      target_return_alt_ = base_return_alt_;
    }
  }

  void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    target_msg_ = *msg;
    target_ready_ = true;
    last_target_sec_ = now_sec();
  }

  void timer_callback()
  {
    if (main_state_ == "Mission" && target_state_ == "Mission") {
      in_reset_cycle_ = false;
      odom_alignment_latched_ = false;
      mission_ready_latched_ = false;
      stable_ticks_ = 0;
      main_return_alt_ = base_return_alt_;
      target_return_alt_ = base_return_alt_;
    }

    const bool odom_ready = odom_fresh(main_odom_) && odom_fresh(target_odom_);
    const bool state_ready = state_fresh(last_main_state_sec_) && state_fresh(last_target_state_sec_);
    bool return_height_aligned = false;
    bool mission_start_ready = false;

    if (odom_ready && state_ready && (in_reset_cycle_ || main_state_ != "Mission" || target_state_ != "Mission")) {
      if (odom_alignment_latched_) {
        return_height_aligned = true;
      } else {
        update_return_targets_from_odom();
        const bool odom_aligned = return_targets_reached() && height_diff_aligned();
        if (odom_aligned) {
          ++stable_ticks_;
        } else {
          stable_ticks_ = 0;
          mission_ready_latched_ = false;
        }
        return_height_aligned = stable_ticks_ >= stable_ticks_required_;
        if (return_height_aligned) {
          odom_alignment_latched_ = true;
        }
      }

      if (return_height_aligned && main_state_ == "WaitMissionStart" && target_state_ == "WaitMissionStart") {
        mission_start_ready = update_perception_gate();
      }
    }

    if (mission_ready_latched_) {
      mission_start_ready = true;
    }

    publish_float(main_return_alt_pub_, static_cast<float>(main_return_alt_));
    publish_float(target_return_alt_pub_, static_cast<float>(target_return_alt_));
    publish_bool(return_height_aligned_pub_, return_height_aligned);
    publish_bool(mission_start_ready_pub_, mission_start_ready);
  }

  void update_return_targets_from_odom()
  {
    if (mission_ready_latched_) {
      return;
    }
    const double height_diff = main_odom_.alt - target_odom_.alt;
    const double adjustment = std::clamp(0.5 * height_diff, -max_alt_adjust_, max_alt_adjust_);
    main_return_alt_ = clamp_alt(base_return_alt_ - adjustment);
    target_return_alt_ = clamp_alt(base_return_alt_ + adjustment);
  }

  bool update_perception_gate()
  {
    const double target_z = target_z_in_control_frame();
    if (!std::isfinite(target_z)) {
      return false;
    }
    if (std::abs(target_z) <= perception_z_tolerance_) {
      mission_ready_latched_ = true;
      return true;
    }

    main_return_alt_ = clamp_alt(main_return_alt_ - perception_adjust_gain_ * target_z);
    return false;
  }

  double target_z_in_control_frame()
  {
    if (!target_ready_ || (now_sec() - last_target_sec_) > target_timeout_sec_) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    try {
      const auto transformed = tf_buffer_.transform(
        target_msg_,
        control_frame_,
        tf2::durationFromSec(tf_timeout_sec_));
      return transformed.point.z;
    } catch (const std::exception &ex) {
      RCLCPP_DEBUG(get_logger(), "target transform unavailable: %s", ex.what());
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  bool return_targets_reached() const
  {
    return std::abs(main_odom_.x) <= home_xy_tolerance_ &&
           std::abs(main_odom_.y) <= home_xy_tolerance_ &&
           std::abs(target_odom_.x) <= home_xy_tolerance_ &&
           std::abs(target_odom_.y) <= home_xy_tolerance_ &&
           std::abs(main_odom_.alt - main_return_alt_) <= return_alt_tolerance_ &&
           std::abs(target_odom_.alt - target_return_alt_) <= return_alt_tolerance_;
  }

  bool height_diff_aligned() const
  {
    return std::abs(main_odom_.alt - target_odom_.alt) <= height_diff_tolerance_;
  }

  bool odom_fresh(const OdomState &state) const
  {
    return state.ready && (now_sec() - state.stamp_sec) <= odom_timeout_sec_;
  }

  bool state_fresh(double stamp_sec) const
  {
    return stamp_sec >= 0.0 && (now_sec() - stamp_sec) <= state_timeout_sec_;
  }

  double clamp_alt(double value) const
  {
    return std::clamp(value, min_return_alt_, max_return_alt_);
  }

  void publish_float(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr &publisher,
    float value)
  {
    std_msgs::msg::Float32 msg{};
    msg.data = value;
    publisher->publish(msg);
  }

  void publish_bool(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &publisher,
    bool value)
  {
    std_msgs::msg::Bool msg{};
    msg.data = value;
    publisher->publish(msg);
  }

  double now_sec() const
  {
    return this->now().seconds();
  }

  std::string main_odometry_topic_;
  std::string target_odometry_topic_;
  std::string main_state_active_topic_;
  std::string target_state_active_topic_;
  std::string reset_topic_;
  std::string target_topic_;
  std::string control_frame_;
  std::string main_return_alt_topic_;
  std::string target_return_alt_topic_;
  std::string return_height_aligned_topic_;
  std::string mission_start_ready_topic_;

  double publish_rate_hz_{20.0};
  double base_return_alt_{5.0};
  double height_diff_tolerance_{0.25};
  double return_alt_tolerance_{0.35};
  double home_xy_tolerance_{0.5};
  double max_alt_adjust_{0.8};
  double min_return_alt_{4.0};
  double max_return_alt_{6.0};
  double perception_z_tolerance_{0.25};
  double perception_adjust_gain_{0.4};
  double target_timeout_sec_{0.5};
  double odom_timeout_sec_{0.5};
  double state_timeout_sec_{0.5};
  double tf_timeout_sec_{0.08};
  int stable_ticks_required_{5};

  OdomState main_odom_;
  OdomState target_odom_;
  std::string main_state_;
  std::string target_state_;
  double last_main_state_sec_{-1.0};
  double last_target_state_sec_{-1.0};
  geometry_msgs::msg::PointStamped target_msg_;
  bool target_ready_{false};
  double last_target_sec_{-1.0};
  bool in_reset_cycle_{false};
  bool odom_alignment_latched_{false};
  bool mission_ready_latched_{false};
  int stable_ticks_{0};
  double main_return_alt_{5.0};
  double target_return_alt_{5.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr main_odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr target_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr main_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr main_return_alt_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_return_alt_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr return_height_aligned_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_start_ready_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReturnHeightCoordinatorNode>());
  rclcpp::shutdown();
  return 0;
}
