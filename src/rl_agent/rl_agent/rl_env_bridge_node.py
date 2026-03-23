import math
import json

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from px4_msgs.msg import VehicleOdometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32, String
import tf2_ros
import tf2_geometry_msgs  # noqa: F401


class RLEnvBridgeNode(Node):
    """
    Convert ROS topics into RL-friendly streams:
    - /rl/obs    (Float32MultiArray)
    - /rl/reward (Float32)
    - /rl/done   (Bool)
    plus /rl/info as JSON string for debugging.
    """

    def __init__(self) -> None:
        super().__init__('rl_env_bridge_node')

        self.declare_parameter('target_topic', '/perception/target_xyz')
        self.declare_parameter('target_lost_topic', '/perception/target_lost')
        self.declare_parameter('odometry_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('action_topic', '/uav/cmd_vel_body')
        self.declare_parameter('mission_active_topic', '/uav/mission_active')
        self.declare_parameter('control_frame', 'base_link_frd')
        self.declare_parameter('tf_timeout_sec', 0.08)
        self.declare_parameter('mission_active_timeout_sec', 1.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('publish_info', True)
        self.declare_parameter('target_timeout_sec', 0.3)
        self.declare_parameter('episode_timeout_sec', 120.0)
        self.declare_parameter('lost_done_timeout_sec', 1.0)
        self.declare_parameter('max_target_distance_m', 15.0)
        self.declare_parameter('desired_distance_m', 3.0)

        self.declare_parameter('w_dist', 1.0)
        self.declare_parameter('w_lat', 0.4)
        self.declare_parameter('w_vert', 0.3)
        self.declare_parameter('w_yaw', 0.3)
        self.declare_parameter('w_act', 0.05)
        self.declare_parameter('lost_penalty', 2.0)
        self.declare_parameter('alive_bonus', 0.2)

        target_topic = str(self.get_parameter('target_topic').value)
        target_lost_topic = str(self.get_parameter('target_lost_topic').value)
        odom_topic = str(self.get_parameter('odometry_topic').value)
        action_topic = str(self.get_parameter('action_topic').value)
        mission_active_topic = str(self.get_parameter('mission_active_topic').value)

        self.control_frame = str(self.get_parameter('control_frame').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.mission_active_timeout_sec = float(self.get_parameter('mission_active_timeout_sec').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.publish_info = bool(self.get_parameter('publish_info').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.episode_timeout_sec = float(self.get_parameter('episode_timeout_sec').value)
        self.lost_done_timeout_sec = float(self.get_parameter('lost_done_timeout_sec').value)
        self.max_target_distance_m = float(self.get_parameter('max_target_distance_m').value)
        self.desired_distance_m = float(self.get_parameter('desired_distance_m').value)

        self.w_dist = float(self.get_parameter('w_dist').value)
        self.w_lat = float(self.get_parameter('w_lat').value)
        self.w_vert = float(self.get_parameter('w_vert').value)
        self.w_yaw = float(self.get_parameter('w_yaw').value)
        self.w_act = float(self.get_parameter('w_act').value)
        self.lost_penalty = float(self.get_parameter('lost_penalty').value)
        self.alive_bonus = float(self.get_parameter('alive_bonus').value)

        self.target_sub = self.create_subscription(PointStamped, target_topic, self.target_cb, 10)
        self.target_lost_sub = self.create_subscription(Bool, target_lost_topic, self.target_lost_cb, 10)
        self.odom_sub = self.create_subscription(
            VehicleOdometry, odom_topic, self.odom_cb, qos_profile_sensor_data
        )
        self.action_sub = self.create_subscription(Twist, action_topic, self.action_cb, 10)
        self.mission_active_sub = self.create_subscription(Bool, mission_active_topic, self.mission_active_cb, 10)
        self.reset_sub = self.create_subscription(Bool, '/rl/reset', self.reset_cb, 10)

        self.obs_pub = self.create_publisher(Float32MultiArray, '/rl/obs', 10)
        self.reward_pub = self.create_publisher(Float32, '/rl/reward', 10)
        self.done_pub = self.create_publisher(Bool, '/rl/done', 10)
        self.info_pub = self.create_publisher(String, '/rl/info', 10) if self.publish_info else None
        self.step_pub = self.create_publisher(Int32, '/rl/step_count', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_target_msg = None
        self.last_target_time = None
        self.target_lost = True
        self.lost_since = None

        self.odom = None
        self.last_action = Twist()
        self.mission_active = False
        self.last_mission_active_time = None
        self.prev_mission_active = False
        self.episode_started_in_mission = False
        self.mission_inactive_done_sent = False
        self.done_latched = False
        self.done_latched_reason = 'running'

        self.ep_start = self.now_sec()
        self.step_count = 0

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info('rl_env_bridge_node started. publishing /rl/obs /rl/reward /rl/done')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def target_cb(self, msg: PointStamped) -> None:
        self.last_target_msg = msg
        self.last_target_time = self.now_sec()
        self.target_lost = False
        self.lost_since = None

    def target_lost_cb(self, msg: Bool) -> None:
        lost = bool(msg.data)
        if lost and not self.target_lost:
            self.lost_since = self.now_sec()
        if not lost:
            self.lost_since = None
        self.target_lost = lost

    def odom_cb(self, msg: VehicleOdometry) -> None:
        self.odom = msg

    def action_cb(self, msg: Twist) -> None:
        self.last_action = msg

    def mission_active_cb(self, msg: Bool) -> None:
        self.mission_active = bool(msg.data)
        self.last_mission_active_time = self.now_sec()

    def mission_active_fresh(self) -> bool:
        if self.last_mission_active_time is None:
            return False
        return (self.now_sec() - self.last_mission_active_time) <= self.mission_active_timeout_sec

    def mission_active_effective(self) -> bool:
        return self.mission_active and self.mission_active_fresh()

    def reset_cb(self, msg: Bool) -> None:
        if msg.data:
            self.ep_start = self.now_sec()
            self.step_count = 0
            self.lost_since = None
            self.episode_started_in_mission = False
            self.mission_inactive_done_sent = False
            self.done_latched = False
            self.done_latched_reason = 'running'
            self.prev_mission_active = self.mission_active_effective()
            self.get_logger().info('Episode reset from /rl/reset')

    def target_fresh(self) -> bool:
        if self.last_target_time is None:
            return False
        return (self.now_sec() - self.last_target_time) <= self.target_timeout_sec

    def get_target_frd(self):
        if self.last_target_msg is None or not self.target_fresh():
            return None
        try:
            pt = self.tf_buffer.transform(
                self.last_target_msg,
                self.control_frame,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
            return pt
        except Exception:
            return None

    def timer_cb(self) -> None:
        pt = self.get_target_frd()
        tx, ty, tz = (float('nan'), float('nan'), float('nan'))
        dist_err = float('nan')
        yaw_err = float('nan')
        target_dist = float('nan')

        if pt is not None:
            tx = float(pt.point.x)
            ty = float(pt.point.y)
            tz = float(pt.point.z)
            dist_err = tx - self.desired_distance_m
            yaw_err = math.atan2(ty, max(tx, 1e-3))
            target_dist = math.sqrt(tx * tx + ty * ty + tz * tz)

        vx = vy = vz = yaw_rate = 0.0
        if self.odom is not None:
            vx = float(self.odom.velocity[0]) if not math.isnan(self.odom.velocity[0]) else 0.0
            vy = float(self.odom.velocity[1]) if not math.isnan(self.odom.velocity[1]) else 0.0
            vz = float(self.odom.velocity[2]) if not math.isnan(self.odom.velocity[2]) else 0.0
            yaw_rate = float(self.odom.angular_velocity[2]) if not math.isnan(self.odom.angular_velocity[2]) else 0.0

        act_vx = float(self.last_action.linear.x)
        act_vy = float(self.last_action.linear.y)
        act_vz = float(self.last_action.linear.z)
        act_yaw = float(self.last_action.angular.z)
        act_mag = abs(act_vx) + abs(act_vy) + abs(act_vz) + abs(act_yaw)

        mission_active_now = self.mission_active_effective()
        mission_entering = mission_active_now and (not self.prev_mission_active)
        mission_leaving = (not mission_active_now) and self.prev_mission_active
        self.prev_mission_active = mission_active_now

        if mission_entering:
            self.ep_start = self.now_sec()
            self.step_count = 0
            self.episode_started_in_mission = True
            self.mission_inactive_done_sent = False
            self.get_logger().info('Mission gate opened: RL stepping enabled')

        reward = 0.0
        if pt is None:
            reward -= self.lost_penalty
        else:
            reward -= self.w_dist * abs(dist_err)
            reward -= self.w_lat * abs(ty)
            reward -= self.w_vert * abs(tz)
            reward -= self.w_yaw * abs(yaw_err)
            reward -= self.w_act * act_mag
            reward += self.alive_bonus

        now = self.now_sec()
        episode_time = now - self.ep_start
        done = self.done_latched
        done_reason = self.done_latched_reason if self.done_latched else 'running'
        if not self.done_latched:
            if mission_active_now:
                if episode_time >= self.episode_timeout_sec:
                    done = True
                    done_reason = 'episode_timeout'
                elif self.target_lost and self.lost_since is not None and (now - self.lost_since) >= self.lost_done_timeout_sec:
                    done = True
                    done_reason = 'target_lost_timeout'
                elif (not math.isnan(target_dist)) and target_dist > self.max_target_distance_m:
                    done = True
                    done_reason = 'target_too_far'
            elif mission_leaving and self.episode_started_in_mission and not self.mission_inactive_done_sent:
                done = True
                done_reason = 'mission_inactive'
                self.mission_inactive_done_sent = True
                self.get_logger().info('Mission gate closed: publishing terminal transition')

            if done:
                self.done_latched = True
                self.done_latched_reason = done_reason

        should_publish_transition = mission_active_now or done
        if not should_publish_transition:
            self.reward_pub.publish(Float32(data=0.0))
            self.done_pub.publish(Bool(data=False))
            info = {
                'done_reason': 'waiting_for_mission',
                'episode_time_sec': round(episode_time, 3),
                'target_dist': None if math.isnan(target_dist) else round(target_dist, 4),
                'target_fresh': self.target_fresh(),
                'mission_active': False,
                'mission_active_fresh': self.mission_active_fresh(),
            }
            if self.info_pub is not None:
                self.info_pub.publish(String(data=json.dumps(info)))
            return

        self.step_count += 1

        obs = Float32MultiArray()
        # [target xyz frd, distance/yaw errors, vehicle vel xyz + yawrate, last action, lost flag]
        obs.data = [
            tx, ty, tz,
            dist_err, yaw_err,
            vx, vy, vz, yaw_rate,
            act_vx, act_vy, act_vz, act_yaw,
            1.0 if self.target_lost else 0.0,
        ]
        self.obs_pub.publish(obs)
        self.reward_pub.publish(Float32(data=float(reward)))
        self.done_pub.publish(Bool(data=done))
        self.step_pub.publish(Int32(data=self.step_count))

        info = {
            'done_reason': done_reason,
            'episode_time_sec': round(episode_time, 3),
            'target_dist': None if math.isnan(target_dist) else round(target_dist, 4),
            'target_fresh': self.target_fresh(),
            'mission_active': mission_active_now,
            'mission_active_fresh': self.mission_active_fresh(),
        }
        if self.info_pub is not None:
            self.info_pub.publish(String(data=json.dumps(info)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RLEnvBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
