/****************************************************************************
 *
 * Copyright 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source_ and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source_ code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples * 
 * @author Beniamino Pozzan <beniamino.pozzan@gmail.com>
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include "geometry_msgs/msg/twist.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <atomic>
#include <limits>
#include <cmath>

#include <chrono>
#include <deque>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl(std::string px4_namespace) :
		Node("offboard_control_srv"),
		state_{State::init},
		service_result_{0},
		service_done_{false},
		set_yaw_{1.57f},	// 设定初始yaw，单位弧度，这里是90度让无人机起飞后朝向东边
		init_altitude_{3},	//设定初始飞行高度m
		source_{"none"},
		num_of_steps_{0},
    	buffer_threshold_{10},	// 默认阈值1s
		vx_control_{0},
        vy_control_{0},
		vz_control_{0},
		offboard_control_mode_publisher_{this->create_publisher<OffboardControlMode>(px4_namespace+"in/offboard_control_mode", 10)},
		trajectory_setpoint_publisher_{this->create_publisher<TrajectorySetpoint>(px4_namespace+"in/trajectory_setpoint", 10)},
		vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace+"vehicle_command")}
	{
		RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example with PX4 services");
		RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			px4_namespace + "out/vehicle_odometry", qos,
			std::bind(&OffboardControl::odometry_callback, this, std::placeholders::_1));
		land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
			px4_namespace + "out/vehicle_land_detected", qos,
			std::bind(&OffboardControl::land_detected_callback, this, std::placeholders::_1));

		mission_sub_ = create_subscription<std_msgs::msg::Int8>(
    		"/mission_control", 10,
    		std::bind(&OffboardControl::mission_callback, this, std::placeholders::_1));

		control_val_sub_ = create_subscription<geometry_msgs::msg::Twist>(
			"/uav/cmd_vel_body", 10,
			std::bind(&OffboardControl::control_val_callback, this, std::placeholders::_1));

		target_lost_sub_ = create_subscription<std_msgs::msg::Bool>(
			"/perception/target_lost", 10,
			std::bind(&OffboardControl::target_lost_callback, this, std::placeholders::_1));
		mission_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("/uav/mission_active", 10);
		load_parameters();

		while (!vehicle_command_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for vehicle_command service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "vehicle_command service not available, waiting again...");
		}

		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
	}

	void switch_to_offboard_mode();
	void arm();
	void auto_land();

private:
	enum class State{
		init,
		offboard_requested,
		wait_for_stable_offboard_mode,
		arm_requested,
		arm_retry_wait,
		armed,
		wait_for_mission_start,
		mission,
		mission_paused,
		returned,
		land_requested,
		wait_for_stable_land,
		landing,
		complete
	} state_;
    
	uint8_t service_result_;
	bool service_done_;
    float vehicle_altitude_;            // 当前高度
    float vehicle_xdistance_;           // x方向移动距离
    float vehicle_ydistance_;           // y方向移动距离
	float vehicle_vertical_speed_;
    float current_yaw_;                 // 当前yaw
    float set_yaw_;                    	// 设定yaw
    uint8_t init_altitude_;             // 初始设置高度
    const char* source_;                // 高度数据来源
    uint8_t num_of_steps_;              // 计数器
    uint8_t buffer_threshold_;          // 计数阈值

	float vx_control_;                  // vx控制量
    float vy_control_;                  // vy控制量
	float vz_control_;                  // vz控制量
	float yaw_control_;				 	// yaw_rate控制量

	bool hold_active_{false};			// 保持位置标志
	float hold_x_{0.0f};
	float hold_y_{0.0f};
	float hold_z_{0.0f};
	float hold_yaw_{0.0f};

	bool odom_ready_{false};
	std::atomic_bool mission_enable_{false};
	std::atomic_bool mission_abort_{false};
	std::atomic_bool target_lost_{true};
	enum class ControlMode { Position, Velocity };
	ControlMode control_mode_{ControlMode::Position};

	bool auto_start_mission_{false};					//true：起飞到位后自动进 mission，不需要键盘 start；false：等待键盘 start 命令进入 mission
	bool auto_start_require_target_{true};				// true：自动进 mission 需要目标未丢失；false：自动进 mission 不考虑目标状态
	double auto_start_delay_sec_{0.5};					// 起飞到位后自动进 mission 的延时，单位秒
	double mission_target_lost_grace_sec_{1.5};
	double target_lost_since_sec_{-1.0};
	int arm_retry_count_{0};
	int arm_retry_max_{20};
	int arm_retry_backoff_steps_{20};
	double wait_for_mission_start_enter_time_{-1.0};	// 进入 wait_for_mission_start 状态的时间点，单位秒
	bool complete_logged_{false};
	double complete_enter_sec_{-1.0};
	double complete_exit_delay_sec_{0.5};
	bool land_detected_maybe_landed_{false};
	bool land_detected_landed_{false};
	bool land_detected_at_rest_{false};
	int landing_complete_hits_{0};
	int landing_complete_land_detect_count_{3};
	double landing_complete_vspeed_thresh_{0.18};
	double landing_complete_alt_window_sec_{1.0};
	double landing_complete_alt_range_thresh_{0.08};
	double landing_complete_fallback_delay_sec_{2.0};
	double landing_state_enter_sec_{-1.0};
	std::deque<std::pair<double, float>> altitude_history_;

    rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
	rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr mission_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_val_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_lost_sub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_active_pub_;

	void load_parameters();
	void publish_offboard_control_mode();
	void publish_position_setpoint(float x, float y, float z, float yaw);
	void publish_velocity_setpoint(float vx, float vy, float vz, float yaw_rate);
	void update_control_mode();
	void publish_active_setpoint();
	void enter_landing_state();
	bool landing_complete_ready(float *alt_range_out = nullptr);
	void switch_buffer(State next_state, const std::string& log_msg);
	void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
	void timer_callback(void);
	void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
	void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
	void mission_callback(const std_msgs::msg::Int8::SharedPtr msg);
	void control_val_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
	void target_lost_callback(const std_msgs::msg::Bool::SharedPtr msg);
};

void OffboardControl::load_parameters()
{
	this->declare_parameter<bool>("auto_start_mission", false);
	this->declare_parameter<bool>("auto_start_require_target", true);
	this->declare_parameter<double>("auto_start_delay_sec", 0.5);
	this->declare_parameter<double>("mission_target_lost_grace_sec", 1.5);
	this->declare_parameter<int>("arm_retry_max", 20);
	this->declare_parameter<int>("arm_retry_backoff_steps", 20);
	this->declare_parameter<int>("landing_complete_land_detect_count", 3);  // 判定落地完成前需要连续命中的次数
	this->declare_parameter<double>("landing_complete_vspeed_thresh", 0.18);  // odom 兜底判据允许的最大竖直速度绝对值
	this->declare_parameter<double>("landing_complete_alt_window_sec", 1.0);  // odom 兜底判据统计高度稳定性的时间窗口
	this->declare_parameter<double>("landing_complete_alt_range_thresh", 0.08);  // odom 兜底判据时间窗口内允许的最大高度波动
	this->declare_parameter<double>("landing_complete_fallback_delay_sec", 2.0);  // 进入 landing 后，启用 odom 兜底前至少等待多久

	auto_start_mission_ = this->get_parameter("auto_start_mission").as_bool();
	auto_start_require_target_ = this->get_parameter("auto_start_require_target").as_bool();
	auto_start_delay_sec_ = this->get_parameter("auto_start_delay_sec").as_double();
	mission_target_lost_grace_sec_ = this->get_parameter("mission_target_lost_grace_sec").as_double();
	arm_retry_max_ = std::max<int>(1, static_cast<int>(this->get_parameter("arm_retry_max").as_int()));
	arm_retry_backoff_steps_ =
		std::max<int>(1, static_cast<int>(this->get_parameter("arm_retry_backoff_steps").as_int()));
	landing_complete_land_detect_count_ =
		std::max<int>(1, static_cast<int>(this->get_parameter("landing_complete_land_detect_count").as_int()));  // 落地完成判据的去抖命中次数
	landing_complete_vspeed_thresh_ = this->get_parameter("landing_complete_vspeed_thresh").as_double();  // odom 兜底判据的竖直速度阈值
	landing_complete_alt_window_sec_ = this->get_parameter("landing_complete_alt_window_sec").as_double();  // odom 兜底判据的高度历史窗口时长
	landing_complete_alt_range_thresh_ = this->get_parameter("landing_complete_alt_range_thresh").as_double();  // odom 兜底判据的高度稳定范围
	landing_complete_fallback_delay_sec_ =
		this->get_parameter("landing_complete_fallback_delay_sec").as_double();  // 进入 landing 后启用 odom 兜底的最短等待时间

	RCLCPP_INFO(
		this->get_logger(),
		"auto_start_mission=%s auto_start_require_target=%s auto_start_delay_sec=%.2f "
		"mission_target_lost_grace_sec=%.2f "
		"arm_retry_max=%d arm_retry_backoff_steps=%d "
		"landing_complete_land_detect_count=%d landing_complete_vspeed_thresh=%.2f "
		"landing_complete_alt_window_sec=%.2f landing_complete_alt_range_thresh=%.2f "
		"landing_complete_fallback_delay_sec=%.2f",
		auto_start_mission_ ? "true" : "false",
		auto_start_require_target_ ? "true" : "false",
		auto_start_delay_sec_,
		mission_target_lost_grace_sec_,
		arm_retry_max_,
		arm_retry_backoff_steps_,
		landing_complete_land_detect_count_,
		landing_complete_vspeed_thresh_,
		landing_complete_alt_window_sec_,
		landing_complete_alt_range_thresh_,
		landing_complete_fallback_delay_sec_);
}

void OffboardControl::update_control_mode()
{
	if (state_ == State::mission) {
		control_mode_ = ControlMode::Velocity;
	} else {
		control_mode_ = ControlMode::Position;
	}
	if (control_mode_ == ControlMode::Velocity && !odom_ready_) {
		// 等待里程计就绪，避免未初始化yaw导致速度指令方向错误
		control_mode_ = ControlMode::Position;
	}
}

void OffboardControl::publish_active_setpoint()
{
	if (state_ == State::mission_paused ||
		state_ == State::land_requested ||
		state_ == State::wait_for_stable_land ||
		state_ == State::landing ||
		state_ == State::complete) {
		return;
	}

	if (control_mode_ == ControlMode::Position) {
		// 起飞阶段和返航阶段发布位置控制指令
		publish_position_setpoint(0.0, 0.0, -(float)init_altitude_, set_yaw_);
		return;
	}

	// 将机体系速度指令转换为NED，避免坐标系不一致
	const float cy = std::cos(current_yaw_);
	const float sy = std::sin(current_yaw_);
	const float vx_ned = cy * vx_control_ - sy * vy_control_;
	const float vy_ned = sy * vx_control_ + cy * vy_control_;
	const float vz_ned = vz_control_;
	publish_velocity_setpoint(vx_ned, vy_ned, vz_ned, yaw_control_);
}

void OffboardControl::enter_landing_state()
{
	landing_complete_hits_ = 0;
	landing_state_enter_sec_ = this->get_clock()->now().seconds();
	altitude_history_.clear();
	state_ = State::landing;
}

bool OffboardControl::landing_complete_ready(float *alt_range_out)
{
	const bool land_detect_ready =
		land_detected_landed_ || (land_detected_maybe_landed_ && land_detected_at_rest_);
	const bool alt_history_ready = !altitude_history_.empty();
	float min_alt = vehicle_altitude_;
	float max_alt = vehicle_altitude_;
	if (alt_history_ready) {
		for (const auto &sample : altitude_history_) {
			min_alt = std::min(min_alt, sample.second);
			max_alt = std::max(max_alt, sample.second);
		}
	}
	const float alt_range = max_alt - min_alt;
	const bool odom_stable =
		alt_history_ready &&
		std::abs(vehicle_vertical_speed_) <= landing_complete_vspeed_thresh_ &&
		alt_range <= landing_complete_alt_range_thresh_;
	const bool fallback_ready =
		landing_state_enter_sec_ > 0.0 &&
		(this->get_clock()->now().seconds() - landing_state_enter_sec_) >= landing_complete_fallback_delay_sec_ &&
		odom_stable;

	if (alt_range_out != nullptr) {
		*alt_range_out = alt_range;
	}

	if (land_detect_ready || fallback_ready) {
		landing_complete_hits_++;
	} else {
		landing_complete_hits_ = 0;
	}

	return landing_complete_hits_ >= landing_complete_land_detect_count_;
}

/**
 * @brief Send a command to switch to offboard mode
 */
void OffboardControl::switch_to_offboard_mode(){
	RCLCPP_INFO(this->get_logger(), "requesting switch to offboard mode");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	RCLCPP_INFO(this->get_logger(), "requesting arm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

/**
 * @brief Send a command to Auto_Land the vehicle
 */
void OffboardControl::auto_land()
{
	RCLCPP_INFO(this->get_logger(), "requesting auto_land");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 6);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only velocity and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = (control_mode_ == ControlMode::Position);
    msg.velocity = (control_mode_ == ControlMode::Velocity);
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory position setpoint
 */
void OffboardControl::publish_position_setpoint(float x, float y, float z, float yaw)
{
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.velocity = {std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN()};
    msg.yaw = yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory velocity setpoint
 */
void OffboardControl::publish_velocity_setpoint(float vx, float vy, float vz, float yaw_rate)
{
    TrajectorySetpoint msg{};
    msg.position = {std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN()};
    msg.velocity = {vx, vy, vz};
    // Use yawspeed when yaw is NaN; otherwise PX4 prioritizes yaw and ignores yawspeed.
    msg.yaw = std::numeric_limits<float>::quiet_NaN();
    msg.yawspeed = yaw_rate;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 * @param param3    Command parameter 3
 */
void OffboardControl::request_vehicle_command(uint16_t command, float param1, float param2, float param3)
{
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	request->request = msg;

	service_done_ = false;
	auto result = vehicle_command_client_->async_send_request(request, std::bind(&OffboardControl::response_callback, this,
                           std::placeholders::_1));
	RCLCPP_INFO(this->get_logger(), "Command send");
}

void OffboardControl::switch_buffer(State next_state, const std::string& log_msg){
	if (++num_of_steps_ > buffer_threshold_) {
        num_of_steps_ = 0;
        RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());
        state_ = next_state;
    }
}

void OffboardControl::timer_callback(void){
	if (state_ != State::wait_for_mission_start) {
		wait_for_mission_start_enter_time_ = -1.0;
	}

	std_msgs::msg::Bool mission_active_msg{};
	mission_active_msg.data = (state_ == State::mission);
	mission_active_pub_->publish(mission_active_msg);

	// offboard_control_mode needs to be paired with trajectory_setpoint
	// 例：任务段用速度，其它用位置保持
	update_control_mode();
	publish_offboard_control_mode();
	publish_active_setpoint();

	switch (state_){
    case State::init:
		switch_to_offboard_mode();
		state_ = State::offboard_requested;
		break;

    case State::offboard_requested:
		if(service_done_){
			if (service_result_ == 0){
			RCLCPP_INFO(this->get_logger(), "Entered offboard mode");
			state_ = State::wait_for_stable_offboard_mode;				
			}
			else{
				RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode, exiting");
				rclcpp::shutdown();
			}
		}
		break;

    case State::wait_for_stable_offboard_mode:
		if (++num_of_steps_ > 10){
			num_of_steps_ = 0;
			arm();
			state_ = State::arm_requested;
		}
		break;

    case State::arm_requested:
		if(service_done_){
			if (service_result_ == 0){
				arm_retry_count_ = 0;
                RCLCPP_INFO(this->get_logger(), "Vehicle armed and taking off");
				state_ = State::armed;
			}
			else{
				arm_retry_count_++;
				if (arm_retry_count_ <= arm_retry_max_) {
					num_of_steps_ = 0;
					RCLCPP_WARN(
						this->get_logger(),
						"Arm rejected (result=%u). Retrying %d/%d after %d ticks",
						service_result_,
						arm_retry_count_,
						arm_retry_max_,
						arm_retry_backoff_steps_);
					state_ = State::arm_retry_wait;
				} else {
					RCLCPP_ERROR(this->get_logger(), "Failed to arm after retries, exiting");
					rclcpp::shutdown();
				}
			}
		}
		break;

	case State::arm_retry_wait:
		if (++num_of_steps_ > arm_retry_backoff_steps_) {
			num_of_steps_ = 0;
			arm();
			state_ = State::arm_requested;
		}
		break;

	case State::armed:
		if(vehicle_altitude_ > init_altitude_ * 0.9)
			switch_buffer(State::wait_for_mission_start, "Reached target altitude, waiting for mission start");
		break;

    case State::wait_for_mission_start:
		if (wait_for_mission_start_enter_time_ < 0.0) {
			wait_for_mission_start_enter_time_ = this->get_clock()->now().seconds();
		}
		if (auto_start_mission_) {
			const bool target_ready = (!auto_start_require_target_) || (!target_lost_);
			const double wait_elapsed = this->get_clock()->now().seconds() - wait_for_mission_start_enter_time_;
			if (target_ready && wait_elapsed >= auto_start_delay_sec_) {
				mission_enable_ = true;
				mission_abort_ = false;
				target_lost_since_sec_ = -1.0;
				RCLCPP_INFO(get_logger(), "Auto mission start triggered");
				state_ = State::mission;
				break;
			}
		}
		if (mission_enable_ && !target_lost_) {
        	target_lost_since_sec_ = -1.0;
        	RCLCPP_INFO(get_logger(), "Mission start command received");
        	state_ = State::mission;
    	}
		break;

    case State::mission:
		if (target_lost_) {
			if (target_lost_since_sec_ < 0.0) {
				target_lost_since_sec_ = this->get_clock()->now().seconds();
			}
			const double lost_elapsed = this->get_clock()->now().seconds() - target_lost_since_sec_;
			if (lost_elapsed >= mission_target_lost_grace_sec_) {
				RCLCPP_WARN(
					get_logger(),
					"Target lost for %.2fs (>= %.2fs), holding position",
					lost_elapsed,
					mission_target_lost_grace_sec_);
				hold_active_ = false;
				state_ = State::mission_paused;
				break;
			}
		}
		target_lost_since_sec_ = -1.0;
		if (mission_abort_) {
        	RCLCPP_WARN(get_logger(), "Mission aborted by user");
        	state_ = State::mission_paused;
        	break;
    	}

    	if (!mission_enable_) {
        	RCLCPP_INFO(get_logger(), "Mission paused");
        	state_ = State::mission_paused;
        	break;
    	}	
		break;

	case State::mission_paused:
    	// 保持当前位置，不做任务动作
    	if (!hold_active_) {
			// NED: altitude 正值 -> z 取负
			hold_x_ = vehicle_xdistance_;
			hold_y_ = vehicle_ydistance_;
			hold_z_ = -vehicle_altitude_;
			hold_yaw_ = current_yaw_;
			hold_active_ = true;
			RCLCPP_INFO(get_logger(),
				"Hold locked: x=%.2f y=%.2f z=%.2f yaw=%.2f",
				hold_x_, hold_y_, hold_z_, hold_yaw_);
		}
		publish_position_setpoint(hold_x_, hold_y_, hold_z_, hold_yaw_);
		// 检测是否恢复任务或返航
    	if (mission_abort_) {
        	RCLCPP_WARN(get_logger(), "Abort -> landing");
        	state_ = State::returned;
    	} 
    	else if (mission_enable_ && !target_lost_) {
        	RCLCPP_INFO(get_logger(), "Mission resumed");
			hold_active_ = false;
        	state_ = State::mission;
    	}
    	break;

	case State::returned:
		if ((std::abs(vehicle_xdistance_) < 0.05) && 
            (std::abs(vehicle_ydistance_) < 0.05)){
			switch_buffer(State::land_requested, "Reached home, requesting land");
		}
		break; 
		
	case State::land_requested:
		auto_land();
		state_ = State::wait_for_stable_land;
		break;
            
	    case State::wait_for_stable_land:
		if(service_done_){
			if (service_result_ == 0){
				RCLCPP_INFO(this->get_logger(), "Land command accepted");
				enter_landing_state();
			} 
			else {
				RCLCPP_ERROR(this->get_logger(), "Land command failed");
			rclcpp::shutdown();
			}
		}
		break;

	case State::landing:
		{
			float alt_range = 0.0f;
			if (landing_complete_ready(&alt_range)) {
				RCLCPP_INFO(
					this->get_logger(),
					"Landing complete. Altitude: %.2fm Speed: %.2fm/s landed=%s maybe_landed=%s at_rest=%s alt_range=%.2fm",
					vehicle_altitude_, vehicle_vertical_speed_,
					land_detected_landed_ ? "true" : "false",
					land_detected_maybe_landed_ ? "true" : "false",
					land_detected_at_rest_ ? "true" : "false",
					alt_range);
				switch_buffer(State::complete, "Entered complete mode");
			}
		}
		break;

	case State::complete:
		if (!complete_logged_) {
			complete_enter_sec_ = this->get_clock()->now().seconds();
			RCLCPP_INFO(this->get_logger(), "Mission complete, exiting process soon");
			complete_logged_ = true;
		} else if (complete_enter_sec_ > 0.0 &&
				   (this->get_clock()->now().seconds() - complete_enter_sec_) >= complete_exit_delay_sec_) {
			RCLCPP_INFO(this->get_logger(), "Mission complete, shutting down offboard node");
			rclcpp::shutdown();
		}
		break;
	default:
		break;
	}
}

void OffboardControl::response_callback(
      rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    auto status = future.wait_for(1s);
	    if (status == std::future_status::ready) {
		  auto reply = future.get()->reply;
		  service_result_ = reply.result;
	      switch (service_result_)
			{
		case reply.VEHICLE_CMD_RESULT_ACCEPTED:
			RCLCPP_INFO(this->get_logger(), "command accepted");
			break;
		case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
			RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
			break;
		case reply.VEHICLE_CMD_RESULT_DENIED:
			RCLCPP_WARN(this->get_logger(), "command denied");
			break;
		case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
			RCLCPP_WARN(this->get_logger(), "command unsupported");
			break;
		case reply.VEHICLE_CMD_RESULT_FAILED:
			RCLCPP_WARN(this->get_logger(), "command failed");
			break;
		case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
			RCLCPP_WARN(this->get_logger(), "command in progress");
			break;
		case reply.VEHICLE_CMD_RESULT_CANCELLED:
			RCLCPP_WARN(this->get_logger(), "command cancelled");
			break;
		default:
			RCLCPP_WARN(this->get_logger(), "command reply unknown");
			break;
		}
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

void OffboardControl::odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    // 打印 debug 信息
    RCLCPP_DEBUG(this->get_logger(),
                 "odometry: x=%.3f y=%.3f z=%.3f vx=%.3f vy=%.3f vz=%.3f frame=%u",
                 msg->position[0], msg->position[1], msg->position[2],
                 msg->velocity[0], msg->velocity[1], msg->velocity[2],
                 msg->pose_frame);

	// 高度：NED 系，z 下为正 -> 转换成“高度为正”的常用表达
	if (!std::isnan(msg->position[2])) {
		vehicle_altitude_ = -msg->position[2];  // 起飞后变正值
		odom_ready_ = true;
		const double now_sec = this->get_clock()->now().seconds();
		altitude_history_.emplace_back(now_sec, vehicle_altitude_);
		while (!altitude_history_.empty() &&
			   (now_sec - altitude_history_.front().first) > landing_complete_alt_window_sec_) {
			altitude_history_.pop_front();
		}
	}

    // 平面位置
    if (!std::isnan(msg->position[0]) && !std::isnan(msg->position[1])) {
        vehicle_xdistance_ = msg->position[0];
        vehicle_ydistance_ = msg->position[1];
    }

    // 垂直速度（NED：下为正，这里取反让上升为正）
    if (!std::isnan(msg->velocity[2])) {
        vehicle_vertical_speed_ = -msg->velocity[2];
    }

    // yaw 从四元数解算（NED frame）
    if (!std::isnan(msg->q[0])) {
        float q0 = msg->q[0];
        float q1 = msg->q[1];
        float q2 = msg->q[2];
        float q3 = msg->q[3];
        // 参考公式：yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
        current_yaw_ = std::atan2(2.0f * (q0*q3 + q1*q2),
                                  1.0f - 2.0f * (q2*q2 + q3*q3));
    }

	source_ = "ODOMETRY";
}

void OffboardControl::land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
	land_detected_maybe_landed_ = msg->maybe_landed;
	land_detected_landed_ = msg->landed;
	land_detected_at_rest_ = msg->at_rest;
}

void OffboardControl::mission_callback(
    const std_msgs::msg::Int8::SharedPtr msg)
{
    switch (msg->data)
    {
    case 1:  // start
        mission_enable_ = true;
        mission_abort_ = false;
        break;

    case 2:  // pause
        mission_enable_ = false;
        break;

    case 3:  // abort
        mission_abort_ = true;
        mission_enable_ = false;
        break;

    case 4:  // land
        mission_abort_ = true;
        break;
    }
}

void OffboardControl::control_val_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	// 速度控制指令
	vx_control_ = msg->linear.x;
	vy_control_ = msg->linear.y;
	vz_control_ = msg->linear.z;
	yaw_control_ = msg->angular.z;
	//RCLCPP_INFO(this->get_logger(),"recv cmd: vx=%.2f vy=%.2f vz=%.2f yaw=%.2f",vx_control_, vy_control_, vz_control_, yaw_control_);
}

void OffboardControl::target_lost_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
	target_lost_ = msg->data;
	if (target_lost_) {
		if (target_lost_since_sec_ < 0.0) {
			target_lost_since_sec_ = this->get_clock()->now().seconds();
		}
	} else {
		target_lost_since_sec_ = -1.0;
	}
}

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>("/fmu/"));

	rclcpp::shutdown();
	return 0;
}
