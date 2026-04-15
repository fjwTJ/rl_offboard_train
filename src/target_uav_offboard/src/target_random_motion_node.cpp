#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class TargetRandomMotionNode : public rclcpp::Node
{
public:
  TargetRandomMotionNode()
  : Node("target_random_motion_node"),
    rng_(std::random_device{}())
  {
    // 随机运动节点输出的机体系速度指令话题。
    declare_parameter<std::string>("cmd_vel_topic", "/target_uav/cmd_vel_body");
    // 训练环境 reset 脉冲话题；收到 true 后重新采样目标运动。
    declare_parameter<std::string>("reset_topic", "/rl/reset");
    // 目标机状态机话题。
    declare_parameter<std::string>("state_active_topic", "/target_uav/state_active");
    // 目标机里程计话题，用来估计当前位置并做边界回中。
    declare_parameter<std::string>("odometry_topic", "/px4_2/fmu/out/vehicle_odometry");
    // 调试话题，发布当前随机运动模式名。
    declare_parameter<std::string>("motion_mode_topic", "/target_uav/motion_mode");
    // 随机动作生成与发布频率。
    declare_parameter<double>("publish_rate_hz", 20.0);
    // 单个随机运动模式的最短持续时间。
    declare_parameter<double>("mode_duration_min_sec", 1.0);
    // 单个随机运动模式的最长持续时间。
    declare_parameter<double>("mode_duration_max_sec", 3.0);
    // 前向速度上限；越大越难追踪。
    declare_parameter<double>("max_forward_speed", 1.2);
    // 横向速度上限；控制蛇形/横移幅度。
    declare_parameter<double>("max_lateral_speed", 0.6);
    // 垂向速度上限；建议前期训练保持较小。
    declare_parameter<double>("max_vertical_speed", 0.15);
    // 偏航角速度上限；越大目标朝向变化越剧烈。
    declare_parameter<double>("max_yaw_rate", 0.35);
    // 当前动作向目标动作靠拢的平滑系数，越大响应越快。
    declare_parameter<double>("smoothing_factor", 0.12);
    // reset 后延迟多久再锁定“home”位置，避免把起飞瞬间位置当成参考点。
    declare_parameter<double>("home_capture_delay_sec", 0.5);
    // 允许目标机围绕 home 活动的安全半径。
    declare_parameter<double>("safe_radius_m", 8.0);
    // 超出安全半径后，朝 home 回拉的增益。
    declare_parameter<double>("return_gain", 0.35);
    // 是否启用垂向随机运动；关闭时更适合前期训练。
    declare_parameter<bool>("enable_vertical_motion", false);
    // 允许飞行的最小高度（相对 local origin，单位 m）。
    declare_parameter<double>("min_flight_height_m", 2.0);
    // 允许飞行的最大高度（相对 local origin，单位 m）。
    declare_parameter<double>("max_flight_height_m", 10.0);
    // 超出高度边界后，Z 向回拉增益。
    declare_parameter<double>("altitude_return_gain", 0.8);

    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    reset_topic_ = get_parameter("reset_topic").as_string();
    state_active_topic_ = get_parameter("state_active_topic").as_string();
    odometry_topic_ = get_parameter("odometry_topic").as_string();
    motion_mode_topic_ = get_parameter("motion_mode_topic").as_string();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    mode_duration_min_sec_ = get_parameter("mode_duration_min_sec").as_double();
    mode_duration_max_sec_ = get_parameter("mode_duration_max_sec").as_double();
    max_forward_speed_ = get_parameter("max_forward_speed").as_double();
    max_lateral_speed_ = get_parameter("max_lateral_speed").as_double();
    max_vertical_speed_ = get_parameter("max_vertical_speed").as_double();
    max_yaw_rate_ = get_parameter("max_yaw_rate").as_double();
    smoothing_factor_ = get_parameter("smoothing_factor").as_double();
    home_capture_delay_sec_ = get_parameter("home_capture_delay_sec").as_double();
    safe_radius_m_ = get_parameter("safe_radius_m").as_double();
    return_gain_ = get_parameter("return_gain").as_double();
    enable_vertical_motion_ = get_parameter("enable_vertical_motion").as_bool();
    min_flight_height_m_ = get_parameter("min_flight_height_m").as_double();
    max_flight_height_m_ = get_parameter("max_flight_height_m").as_double();
    altitude_return_gain_ = get_parameter("altitude_return_gain").as_double();
    if (max_flight_height_m_ < min_flight_height_m_) {
      std::swap(min_flight_height_m_, max_flight_height_m_);
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>(motion_mode_topic_, 10);

    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
      reset_topic_, 10, std::bind(&TargetRandomMotionNode::reset_callback, this, std::placeholders::_1));
    state_active_sub_ = create_subscription<std_msgs::msg::String>(
      state_active_topic_, 10, std::bind(&TargetRandomMotionNode::state_active_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TargetRandomMotionNode::odometry_callback, this, std::placeholders::_1));

    const auto timer_period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
      std::bind(&TargetRandomMotionNode::timer_callback, this));

    schedule_next_mode(true);
    RCLCPP_INFO(
      get_logger(),
      "target_random_motion_node started: cmd_vel_topic=%s odometry_topic=%s state_active_topic=%s",
      cmd_vel_topic_.c_str(),
      odometry_topic_.c_str(),
      state_active_topic_.c_str());
  }

private:
  enum class MotionMode
  {
    Hover,
    Cruise,
    Strafe,
    ArcLeft,
    ArcRight
  };

  void reset_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }
    current_cmd_ = geometry_msgs::msg::Twist{};
    target_cmd_ = geometry_msgs::msg::Twist{};
    home_valid_ = false;
    home_capture_time_sec_ = -1.0;
    schedule_next_mode(true);
    publish_zero();
    RCLCPP_INFO(get_logger(), "target motion reset");
  }

  void state_active_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    state_active_ = msg->data;
  }

  void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    if (std::isnan(msg->position[0]) || std::isnan(msg->position[1]) || std::isnan(msg->position[2])) {
      return;
    }

    position_[0] = msg->position[0];
    position_[1] = msg->position[1];
    position_[2] = msg->position[2];
    odom_ready_ = true;

    if (!home_valid_) {
      if (home_capture_time_sec_ < 0.0) {
        home_capture_time_sec_ = now_sec();
      }
      if ((now_sec() - home_capture_time_sec_) >= home_capture_delay_sec_) {
        home_[0] = position_[0];
        home_[1] = position_[1];
        home_[2] = position_[2];
        home_valid_ = true;
        RCLCPP_INFO(get_logger(), "captured target motion home position: x=%.2f y=%.2f z=%.2f",
          home_[0], home_[1], home_[2]);
      }
    }
  }

  void timer_callback()
  {
    if (state_active_ != "Mission") {
      publish_zero();
      return;
    }

    if (now_sec() >= next_mode_switch_sec_) {
      schedule_next_mode(false);
    }

    apply_mode_target();
    apply_return_bias();
    apply_altitude_limits();
    smooth_command();
    cmd_pub_->publish(current_cmd_);
    publish_mode();
  }

  void schedule_next_mode(bool reset_phase)
  {
    const int mode_index = std::uniform_int_distribution<int>(0, 4)(rng_);
    current_mode_ = static_cast<MotionMode>(mode_index);
    next_mode_switch_sec_ = now_sec() + sample_uniform(mode_duration_min_sec_, mode_duration_max_sec_);

    target_cmd_ = geometry_msgs::msg::Twist{};
    if (reset_phase) {
      current_mode_ = MotionMode::Hover;
      next_mode_switch_sec_ = now_sec() + sample_uniform(0.8, 1.5);
    }
  }

  void apply_mode_target()
  {
    geometry_msgs::msg::Twist desired{};

    switch (current_mode_) {
      case MotionMode::Hover:
        break;

      case MotionMode::Cruise:
        desired.linear.x = sample_uniform(0.35, max_forward_speed_);
        desired.linear.y = sample_uniform(-0.35 * max_lateral_speed_, 0.35 * max_lateral_speed_);
        desired.angular.z = sample_uniform(-0.2 * max_yaw_rate_, 0.2 * max_yaw_rate_);
        break;

      case MotionMode::Strafe:
        desired.linear.x = sample_uniform(0.15, 0.55 * max_forward_speed_);
        desired.linear.y = sample_signed(max_lateral_speed_);
        desired.angular.z = sample_uniform(-0.15, 0.15);
        break;

      case MotionMode::ArcLeft:
        desired.linear.x = sample_uniform(0.35, max_forward_speed_);
        desired.linear.y = sample_uniform(0.0, 0.4 * max_lateral_speed_);
        desired.angular.z = sample_uniform(0.15, max_yaw_rate_);
        break;

      case MotionMode::ArcRight:
        desired.linear.x = sample_uniform(0.35, max_forward_speed_);
        desired.linear.y = sample_uniform(-0.4 * max_lateral_speed_, 0.0);
        desired.angular.z = sample_uniform(-max_yaw_rate_, -0.15);
        break;
    }

    if (enable_vertical_motion_) {
      desired.linear.z = sample_uniform(-max_vertical_speed_, max_vertical_speed_);
    }

    target_cmd_ = desired;
  }

  void apply_return_bias()
  {
    if (!odom_ready_ || !home_valid_) {
      return;
    }

    const double dx = home_[0] - position_[0];
    const double dy = home_[1] - position_[1];
    const double dist_xy = std::hypot(position_[0] - home_[0], position_[1] - home_[1]);
    if (dist_xy <= safe_radius_m_) {
      return;
    }

    const double norm = std::max(std::hypot(dx, dy), 1e-3);
    const double inward_x = dx / norm;
    const double inward_y = dy / norm;
    const double excess = dist_xy - safe_radius_m_;
    const double bias = std::clamp(excess * return_gain_, 0.0, max_forward_speed_);

    target_cmd_.linear.x += bias * inward_x;
    target_cmd_.linear.y += bias * inward_y;
    target_cmd_.angular.z += std::clamp(std::atan2(dy, dx) * 0.2, -max_yaw_rate_, max_yaw_rate_);
  }

  void apply_altitude_limits()
  {
    if (!odom_ready_) {
      return;
    }

    const double current_height_m = -position_[2];
    if (current_height_m < min_flight_height_m_) {
      const double climb_bias = std::clamp(
        (min_flight_height_m_ - current_height_m) * altitude_return_gain_,
        0.0,
        max_vertical_speed_);
      target_cmd_.linear.z = -climb_bias;
      return;
    }

    if (current_height_m > max_flight_height_m_) {
      const double descend_bias = std::clamp(
        (current_height_m - max_flight_height_m_) * altitude_return_gain_,
        0.0,
        max_vertical_speed_);
      target_cmd_.linear.z = descend_bias;
      return;
    }

    target_cmd_.linear.z = std::clamp(target_cmd_.linear.z, -max_vertical_speed_, max_vertical_speed_);
  }

  void smooth_command()
  {
    current_cmd_.linear.x = blend(current_cmd_.linear.x, target_cmd_.linear.x);
    current_cmd_.linear.y = blend(current_cmd_.linear.y, target_cmd_.linear.y);
    current_cmd_.linear.z = blend(current_cmd_.linear.z, target_cmd_.linear.z);
    current_cmd_.angular.z = blend(current_cmd_.angular.z, target_cmd_.angular.z);

    current_cmd_.linear.x = std::clamp(current_cmd_.linear.x, -max_forward_speed_, max_forward_speed_);
    current_cmd_.linear.y = std::clamp(current_cmd_.linear.y, -max_lateral_speed_, max_lateral_speed_);
    current_cmd_.linear.z = std::clamp(current_cmd_.linear.z, -max_vertical_speed_, max_vertical_speed_);
    current_cmd_.angular.z = std::clamp(current_cmd_.angular.z, -max_yaw_rate_, max_yaw_rate_);
  }

  void publish_zero()
  {
    current_cmd_ = geometry_msgs::msg::Twist{};
    cmd_pub_->publish(current_cmd_);
    publish_mode("inactive");
  }

  void publish_mode(const std::string &override_mode = "")
  {
    std_msgs::msg::String msg{};
    msg.data = override_mode.empty() ? motion_mode_name(current_mode_) : override_mode;
    mode_pub_->publish(msg);
  }

  double blend(double current, double target) const
  {
    const double alpha = std::clamp(smoothing_factor_, 0.0, 1.0);
    return current + alpha * (target - current);
  }

  double sample_uniform(double min_value, double max_value)
  {
    return std::uniform_real_distribution<double>(min_value, max_value)(rng_);
  }

  double sample_signed(double max_abs)
  {
    return sample_uniform(-max_abs, max_abs);
  }

  double now_sec()
  {
    return static_cast<double>(this->now().nanoseconds()) * 1e-9;
  }

  static std::string motion_mode_name(MotionMode mode)
  {
    switch (mode) {
      case MotionMode::Hover:
        return "hover";
      case MotionMode::Cruise:
        return "cruise";
      case MotionMode::Strafe:
        return "strafe";
      case MotionMode::ArcLeft:
        return "arc_left";
      case MotionMode::ArcRight:
        return "arc_right";
    }
    return "unknown";
  }

  std::string cmd_vel_topic_;
  std::string reset_topic_;
  std::string state_active_topic_;
  std::string odometry_topic_;
  std::string motion_mode_topic_;

  double publish_rate_hz_{20.0};
  double mode_duration_min_sec_{1.0};
  double mode_duration_max_sec_{3.0};
  double max_forward_speed_{1.2};
  double max_lateral_speed_{0.6};
  double max_vertical_speed_{0.15};
  double max_yaw_rate_{0.35};
  double smoothing_factor_{0.12};
  double home_capture_delay_sec_{0.5};
  double safe_radius_m_{8.0};
  double return_gain_{0.35};
  bool enable_vertical_motion_{false};
  double min_flight_height_m_{2.0};
  double max_flight_height_m_{4.0};
  double altitude_return_gain_{0.8};

  std::string state_active_{};
  bool odom_ready_{false};
  bool home_valid_{false};
  double home_capture_time_sec_{-1.0};
  double next_mode_switch_sec_{0.0};
  std::array<double, 3> position_{0.0, 0.0, 0.0};
  std::array<double, 3> home_{0.0, 0.0, 0.0};
  MotionMode current_mode_{MotionMode::Hover};
  geometry_msgs::msg::Twist current_cmd_{};
  geometry_msgs::msg::Twist target_cmd_{};
  std::mt19937 rng_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_active_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetRandomMotionNode>());
  rclcpp::shutdown();
  return 0;
}
