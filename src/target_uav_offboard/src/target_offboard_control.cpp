#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace
{
// Target UAV instance 2 uses MAV_SYS_ID=3; these identities are fixed in this project.
constexpr uint8_t kTargetSystemId = 3;
constexpr uint8_t kTargetComponentId = 1;
constexpr uint8_t kSourceSystemId = 1;
constexpr uint8_t kSourceComponentId = 1;
}

class TargetOffboardControl : public rclcpp::Node
{
public:
  TargetOffboardControl()
  : Node("target_offboard_control")
  {
    // PX4_2 的 ROS 2 话题命名空间，默认对应第二架 PX4 实例。
    declare_parameter<std::string>("px4_namespace", "/px4_2/fmu/");
    // 低层执行器订阅的机体系速度指令话题。
    declare_parameter<std::string>("cmd_vel_topic", "/target_uav/cmd_vel_body");
    // 对外发布“目标机”的状态话题。
    declare_parameter<std::string>("state_active_topic", "/target_uav/state_active");
    // 订阅“控制机”的状态话题。
    declare_parameter<std::string>("peer_state_active_topic", "/uav/state_active");
    // RL episode 软复位脉冲话题。
    declare_parameter<std::string>("reset_topic", "/rl/reset");
    declare_parameter<std::string>("return_alt_topic", "/rl/target_return_alt");
    declare_parameter<std::string>("return_height_aligned_topic", "/rl/return_height_aligned");
    // 起飞目标高度，单位 m。
    declare_parameter<double>("takeoff_height", 5.0);
    // 起飞后默认朝向，单位 rad。
    declare_parameter<double>("takeoff_yaw", 1.57);
    // Offboard 控制主循环频率。
    declare_parameter<double>("control_rate_hz", 10.0);
    // 外部速度指令超时时间；超时后回到位置保持。
    declare_parameter<double>("cmd_timeout_sec", 0.5);
    // 进入 OFFBOARD 之前连续发送 setpoint 的预热 tick 数。
    declare_parameter<int>("offboard_setpoint_warmup_ticks", 10);
    // 判定“起飞已到位”的高度比例阈值。
    declare_parameter<double>("takeoff_reached_ratio", 0.9);
    // 请求 OFFBOARD / ARM 后，若未成功，按该周期重发。
    declare_parameter<double>("command_retry_sec", 1.0);
    declare_parameter<double>("return_alt_timeout_sec", 0.5);
    declare_parameter<double>("return_alt_tolerance", 0.35);

    px4_namespace_ = get_parameter("px4_namespace").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    state_active_topic_ = get_parameter("state_active_topic").as_string();
    peer_state_active_topic_ = get_parameter("peer_state_active_topic").as_string();
    reset_topic_ = get_parameter("reset_topic").as_string();
    return_alt_topic_ = get_parameter("return_alt_topic").as_string();
    return_height_aligned_topic_ = get_parameter("return_height_aligned_topic").as_string();
    takeoff_height_ = static_cast<float>(get_parameter("takeoff_height").as_double());
    takeoff_yaw_ = static_cast<float>(get_parameter("takeoff_yaw").as_double());
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    cmd_timeout_sec_ = get_parameter("cmd_timeout_sec").as_double();
    offboard_setpoint_warmup_ticks_ = std::max<int>(1, get_parameter("offboard_setpoint_warmup_ticks").as_int());
    takeoff_reached_ratio_ = static_cast<float>(get_parameter("takeoff_reached_ratio").as_double());
    command_retry_sec_ = std::max(0.2, get_parameter("command_retry_sec").as_double());
    return_alt_timeout_sec_ = get_parameter("return_alt_timeout_sec").as_double();
    return_alt_tolerance_ = static_cast<float>(get_parameter("return_alt_tolerance").as_double());

    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(px4_namespace_ + "in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(px4_namespace_ + "in/trajectory_setpoint", 10);
    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(px4_namespace_ + "in/vehicle_command", 10);
    state_active_pub_ = create_publisher<std_msgs::msg::String>(state_active_topic_, 10);
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      px4_namespace_ + "out/vehicle_odometry",
      qos,
      std::bind(&TargetOffboardControl::odometry_callback, this, std::placeholders::_1));
    control_mode_sub_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
      px4_namespace_ + "out/vehicle_control_mode",
      qos,
      std::bind(&TargetOffboardControl::control_mode_callback, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      10,
      std::bind(&TargetOffboardControl::cmd_callback, this, std::placeholders::_1));

    peer_state_active_sub_ = create_subscription<std_msgs::msg::String>(
      peer_state_active_topic_,
      10,
      std::bind(&TargetOffboardControl::peer_state_callback, this, std::placeholders::_1));
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
      reset_topic_,
      10,
      std::bind(&TargetOffboardControl::reset_callback, this, std::placeholders::_1));
    return_alt_sub_ = create_subscription<std_msgs::msg::Float32>(
      return_alt_topic_,
      10,
      std::bind(&TargetOffboardControl::return_alt_callback, this, std::placeholders::_1));
    return_height_aligned_sub_ = create_subscription<std_msgs::msg::Bool>(
      return_height_aligned_topic_,
      10,
      std::bind(&TargetOffboardControl::return_height_aligned_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TargetOffboardControl::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "target_offboard_control started: px4_namespace=%s cmd_vel_topic=%s target_system_id=%u return_alt_topic=%s",
      px4_namespace_.c_str(),
      cmd_vel_topic_.c_str(),
      kTargetSystemId,
      return_alt_topic_.c_str());
  }

private:
  enum class State
  {
    Warmup,
    OffboardRequested,
    ArmRequested,
    Takeoff,
    WaitMissionStart,
    Mission,
    Return
  };

  void timer_callback()
  {
    publish_state_active();
    publish_offboard_control_mode();
    publish_active_setpoint();

    switch (state_) {
      case State::Warmup:
        if (!px4_input_ready()) {
          return;
        }
        ++warmup_ticks_;
        if (warmup_ticks_ >= offboard_setpoint_warmup_ticks_) {
          request_offboard_mode();
          state_ = State::OffboardRequested;
          RCLCPP_INFO(get_logger(), "Requested OFFBOARD mode");
        }
        break;

      case State::OffboardRequested:
        if (is_offboard_enabled()) {
          request_arm();
          state_ = State::ArmRequested;
          RCLCPP_INFO(get_logger(), "Requested arm");
        } else if (command_retry_elapsed()) {
          request_offboard_mode();
          RCLCPP_WARN(get_logger(), "OFFBOARD not active yet, retrying mode request");
        }
        break;

      case State::ArmRequested:
        if (is_armed()) {
          state_ = State::Takeoff;
          RCLCPP_INFO(get_logger(), "Entering takeoff");
        } else if (command_retry_elapsed()) {
          request_arm();
          RCLCPP_WARN(get_logger(), "Vehicle not armed yet, retrying arm request");
        }
        break;

      case State::Takeoff:
        if (odom_ready_ && (-position_[2]) >= takeoff_height_ * takeoff_reached_ratio_) {
          state_ = State::WaitMissionStart;
          RCLCPP_INFO(get_logger(), "Takeoff complete, wait mission start");
        }
        break;
      case State::WaitMissionStart:
        if (peer_state_active_ == "Mission")
        {
          state_ = State::Mission;
          RCLCPP_INFO(get_logger(), "Auto sync start triggered");
        }
        break;
      case State::Mission:
        if (reset_requested_) {
          cmd_body_ = geometry_msgs::msg::Twist{};
          last_cmd_time_sec_ = -1.0;
          reset_requested_ = false;
          RCLCPP_INFO(get_logger(), "RL reset requested, returning to home");
          state_ = State::Return;
        }
        break;
      case State::Return:
        if ((std::abs(position_[0]) < 0.5) &&
            (std::abs(position_[1]) < 0.5) &&
            (std::abs((-position_[2]) - active_return_alt()) < return_alt_tolerance_) &&
            return_height_aligned_ &&
            return_height_aligned_fresh()) {
          state_ = State::WaitMissionStart;
          RCLCPP_INFO(get_logger(), "Target UAV reached home, waiting for mission start");
        }
        break;
    }
  }

  void publish_state_active()
  {
    std_msgs::msg::String msg{};
    switch (state_) {
      case State::Warmup: msg.data = "Warmup"; break;
      case State::OffboardRequested: msg.data = "OffboardRequested"; break;
      case State::ArmRequested: msg.data = "ArmRequested"; break;
      case State::Takeoff: msg.data = "Takeoff"; break;
      case State::WaitMissionStart: msg.data = "WaitMissionStart"; break;
      case State::Mission: msg.data = "Mission"; break;
      case State::Return: msg.data = "Return"; break;
      default: msg.data = "Unknown"; break;
    }
    state_active_pub_->publish(msg);
  }

  void publish_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = (state_ != State::Mission) || !command_is_fresh();
    msg.velocity = (state_ == State::Mission) && command_is_fresh();
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = now_us();
    offboard_control_mode_pub_->publish(msg);
  }

  void publish_active_setpoint()
  {
    // 在 mission 状态下，如果收到新指令且里程计数据准备好，发布速度指令
    if (state_ == State::Mission && command_is_fresh() && odom_ready_) {
      publish_velocity_setpoint_body_to_ned();
      return;
    }
    // 在 mission 状态下，如果没有收到新指令，保持当前位置
    if (state_ == State::Mission && !command_is_fresh()) {
      control_x_ = position_[0];
      control_y_ = position_[1];
      control_z_ = position_[2];
      control_yaw_ = current_yaw_;
    }
    // 在非 mission 状态下，保持在起飞点附近
    if (state_ != State::Mission) {
      control_x_ = 0.0f;
      control_y_ = 0.0f;
      control_z_ = -active_return_alt();
      control_yaw_ = takeoff_yaw_;
    }
  
    const float control_x = odom_ready_ ? control_x_ : 0.0f;
    const float control_y = odom_ready_ ? control_y_ : 0.0f;
    const float control_z = odom_ready_ ? control_z_ : -active_return_alt();
    const float control_yaw = odom_ready_ ? control_yaw_ : takeoff_yaw_;
    publish_position_setpoint(control_x, control_y, control_z, control_yaw);
  }

  void publish_position_setpoint(float x, float y, float z, float yaw)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.velocity = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN()};
    msg.yaw = yaw;
    msg.timestamp = now_us();
    trajectory_setpoint_pub_->publish(msg);
  }

  void publish_velocity_setpoint_body_to_ned()
  {
    const float cy = std::cos(current_yaw_);
    const float sy = std::sin(current_yaw_);
    const float vx_ned = cy * cmd_body_.linear.x - sy * cmd_body_.linear.y;
    const float vy_ned = sy * cmd_body_.linear.x + cy * cmd_body_.linear.y;
    const float vz_ned = cmd_body_.linear.z;

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN()};
    msg.velocity = {vx_ned, vy_ned, vz_ned};
    msg.yaw = std::numeric_limits<float>::quiet_NaN();
    msg.yawspeed = static_cast<float>(cmd_body_.angular.z);
    msg.timestamp = now_us();
    trajectory_setpoint_pub_->publish(msg);
  }

  void request_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = kTargetSystemId;
    msg.target_component = kTargetComponentId;
    msg.source_system = kSourceSystemId;
    msg.source_component = kSourceComponentId;
    msg.from_external = true;
    msg.timestamp = now_us();
    vehicle_command_pub_->publish(msg);
    last_command_request_time_sec_ = now_sec();
  }

  void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    if (!std::isnan(msg->position[0]) && !std::isnan(msg->position[1]) && !std::isnan(msg->position[2])) {
      position_[0] = msg->position[0];
      position_[1] = msg->position[1];
      position_[2] = msg->position[2];
      odom_ready_ = true;
    }
    const auto &q = msg->q;
    if (!std::isnan(q[0]) && !std::isnan(q[1]) && !std::isnan(q[2]) && !std::isnan(q[3])) {
      current_yaw_ = quaternion_to_yaw(q[0], q[1], q[2], q[3]);
    }
  }

  void control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
  {
    vehicle_control_mode_ = *msg;
    control_mode_ready_ = true;
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_body_ = *msg;
    last_cmd_time_sec_ = now_sec();
  }

  void peer_state_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    peer_state_active_ = msg->data;
  }

  void reset_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && state_ == State::Mission) {
      reset_requested_ = true;
    }
  }

  void return_alt_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (std::isfinite(msg->data)) {
      return_alt_ = std::max(0.0f, msg->data);
      last_return_alt_time_sec_ = now_sec();
    }
  }

  void return_height_aligned_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    return_height_aligned_ = msg->data;
    last_return_height_aligned_time_sec_ = now_sec();
  }

  bool command_is_fresh()
  {
    if (last_cmd_time_sec_ < 0.0) {
      return false;
    }
    return (now_sec() - last_cmd_time_sec_) <= cmd_timeout_sec_;
  }

  bool return_alt_fresh()
  {
    return last_return_alt_time_sec_ >= 0.0 &&
           (now_sec() - last_return_alt_time_sec_) <= return_alt_timeout_sec_;
  }

  bool return_height_aligned_fresh()
  {
    return last_return_height_aligned_time_sec_ >= 0.0 &&
           (now_sec() - last_return_height_aligned_time_sec_) <= return_alt_timeout_sec_;
  }

  float active_return_alt()
  {
    return return_alt_fresh() ? return_alt_ : takeoff_height_;
  }

  bool px4_input_ready() const
  {
    return offboard_control_mode_pub_->get_subscription_count() > 0 &&
           trajectory_setpoint_pub_->get_subscription_count() > 0 &&
           vehicle_command_pub_->get_subscription_count() > 0 &&
           odom_ready_ &&
           control_mode_ready_;
  }

  bool is_offboard_enabled() const
  {
    return control_mode_ready_ && vehicle_control_mode_.flag_control_offboard_enabled;
  }

  bool is_armed() const
  {
    return control_mode_ready_ && vehicle_control_mode_.flag_armed;
  }

  bool command_retry_elapsed() const
  {
    if (last_command_request_time_sec_ < 0.0) {
      return true;
    }
    return (now_sec() - last_command_request_time_sec_) >= command_retry_sec_;
  }

  void request_offboard_mode()
  {
    request_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
  }

  void request_arm()
  {
    request_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
  }

  uint64_t now_us() const
  {
    return static_cast<uint64_t>(this->now().nanoseconds() / 1000);
  }

  double now_sec() const
  {
    return this->now().seconds();
  }

  static float quaternion_to_yaw(float w, float x, float y, float z)
  {
    const float siny_cosp = 2.0f * (w * z + x * y);
    const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  std::string px4_namespace_;
  std::string cmd_vel_topic_;
  std::string state_active_topic_;
  std::string peer_state_active_topic_;
  std::string reset_topic_;
  std::string return_alt_topic_;
  std::string return_height_aligned_topic_;
  float takeoff_height_{5.0f};
  float takeoff_yaw_{1.57f};
  double control_rate_hz_{10.0};
  double cmd_timeout_sec_{0.5};
  int offboard_setpoint_warmup_ticks_{10};
  float takeoff_reached_ratio_{0.9f};
  double command_retry_sec_{1.0};
  double return_alt_timeout_sec_{0.5};
  float return_alt_tolerance_{0.35f};

  State state_{State::Warmup};
  int warmup_ticks_{0};
  bool odom_ready_{false};
  bool control_mode_ready_{false};
  std::array<float, 3> position_{0.0f, 0.0f, 0.0f};
  float current_yaw_{0.0f};
  geometry_msgs::msg::Twist cmd_body_{};
  std::string peer_state_active_{"Unknown"};
  bool reset_requested_{false};
  double last_cmd_time_sec_{-1.0};
  double last_command_request_time_sec_{-1.0};
  float return_alt_{5.0f};
  bool return_height_aligned_{false};
  double last_return_alt_time_sec_{-1.0};
  double last_return_height_aligned_time_sec_{-1.0};
  float control_x_{0.0f};
  float control_y_{0.0f};
  float control_z_{-3.0f};
  float control_yaw_{1.57f};
  px4_msgs::msg::VehicleControlMode vehicle_control_mode_{};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_active_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr peer_state_active_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr return_alt_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr return_height_aligned_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetOffboardControl>());
  rclcpp::shutdown();
  return 0;
}
