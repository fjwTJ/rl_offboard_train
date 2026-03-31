import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs  # noqa: F401


def clamp(val: float, low: float, high: float) -> float:
    return max(min(val, high), low)


class HeuristicPolicyNode(Node):
    """
    Minimal runnable heuristic policy node.
    Current behavior is a safe hand-crafted placeholder that can later be
    replaced by a trained model inference node.
    """

    def __init__(self) -> None:
        super().__init__('heuristic_policy_node')

        self.declare_parameter('target_topic', '/perception/target_xyz')
        self.declare_parameter('target_lost_topic', '/perception/target_lost')
        self.declare_parameter('odometry_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('cmd_topic', '/uav/cmd_vel_rl')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('target_timeout_sec', 0.3)
        self.declare_parameter('control_frame', 'base_link_frd')
        self.declare_parameter('tf_timeout_sec', 0.08)

        # Placeholder policy gains. These approximate a stable chase policy.
        self.declare_parameter('desired_distance_m', 3.0)
        self.declare_parameter('kp_vx', 1.0)
        self.declare_parameter('kp_vy', 0.6)
        self.declare_parameter('kp_vz', 0.6)
        self.declare_parameter('kp_yaw', 1.5)
        self.declare_parameter('max_vx', 1.0)
        self.declare_parameter('max_vy', 0.4)
        self.declare_parameter('max_vz', 1.0)
        self.declare_parameter('max_yaw_rate', 1.6)

        target_topic = self.get_parameter('target_topic').value
        target_lost_topic = self.get_parameter('target_lost_topic').value
        odom_topic = self.get_parameter('odometry_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value

        self.desired_distance_m = float(self.get_parameter('desired_distance_m').value)
        self.kp_vx = float(self.get_parameter('kp_vx').value)
        self.kp_vy = float(self.get_parameter('kp_vy').value)
        self.kp_vz = float(self.get_parameter('kp_vz').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_vz = float(self.get_parameter('max_vz').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.control_frame = str(self.get_parameter('control_frame').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)

        self.target_sub = self.create_subscription(PointStamped, target_topic, self.target_cb, 10)
        self.target_lost_sub = self.create_subscription(Bool, target_lost_topic, self.target_lost_cb, 10)
        self.odom_sub = self.create_subscription(VehicleOdometry, odom_topic, self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_target_msg = None
        self.last_target_time = None
        self.target_lost = True
        self.last_yaw = 0.0

        period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info(f'heuristic_policy_node started. publish -> {cmd_topic}')

    def target_cb(self, msg: PointStamped) -> None:
        self.last_target_msg = msg
        self.last_target_time = self.get_clock().now()
        self.target_lost = False

    def target_lost_cb(self, msg: Bool) -> None:
        self.target_lost = bool(msg.data)

    def odom_cb(self, msg: VehicleOdometry) -> None:
        # Keep latest yaw for future model input usage.
        q0, q1, q2, q3 = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        self.last_yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

    def target_is_fresh(self) -> bool:
        if self.last_target_time is None:
            return False
        dt = (self.get_clock().now() - self.last_target_time).nanoseconds * 1e-9
        return dt <= self.target_timeout_sec

    def timer_cb(self) -> None:
        cmd = Twist()
        if self.target_lost or (not self.target_is_fresh()) or self.last_target_msg is None:
            self.cmd_pub.publish(cmd)
            return

        try:
            pt = self.tf_buffer.transform(
                self.last_target_msg,
                self.control_frame,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as exc:
            self.get_logger().warn(f'TF transform failed: {exc}')
            self.cmd_pub.publish(cmd)
            return

        # Use target position in FRD control frame.
        tx = pt.point.x
        ty = pt.point.y
        tz = pt.point.z

        dist_err = tx - self.desired_distance_m
        yaw_err = math.atan2(ty, max(tx, 1e-3))

        vx = clamp(self.kp_vx * dist_err, -self.max_vx, self.max_vx)
        vy = clamp(self.kp_vy * ty, -self.max_vy, self.max_vy)
        vz = clamp(self.kp_vz * tz, -self.max_vz, self.max_vz)
        yaw_rate = clamp(self.kp_yaw * yaw_err, -self.max_yaw_rate, self.max_yaw_rate)

        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = float(vz)
        cmd.angular.z = float(yaw_rate)
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeuristicPolicyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
