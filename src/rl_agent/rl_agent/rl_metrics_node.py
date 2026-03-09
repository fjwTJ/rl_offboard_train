import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
import tf2_ros
import tf2_geometry_msgs  # noqa: F401


class RLMetricsNode(Node):
    """Compute tracking metrics online (live or during rosbag replay)."""

    def __init__(self) -> None:
        super().__init__('rl_metrics_node')

        self.declare_parameter('target_topic', '/perception/target_xyz')
        self.declare_parameter('target_lost_topic', '/perception/target_lost')
        self.declare_parameter('cmd_topic', '/uav/cmd_vel_body')
        self.declare_parameter('active_source_topic', '/uav/cmd_active_source')
        self.declare_parameter('control_frame', 'base_link_frd')
        self.declare_parameter('tf_timeout_sec', 0.08)
        self.declare_parameter('summary_period_sec', 2.0)
        self.declare_parameter('save_json_path', '')
        self.declare_parameter('desired_distance_m', 3.0)

        target_topic = str(self.get_parameter('target_topic').value)
        target_lost_topic = str(self.get_parameter('target_lost_topic').value)
        cmd_topic = str(self.get_parameter('cmd_topic').value)
        active_source_topic = str(self.get_parameter('active_source_topic').value)
        self.control_frame = str(self.get_parameter('control_frame').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.summary_period_sec = float(self.get_parameter('summary_period_sec').value)
        self.save_json_path = str(self.get_parameter('save_json_path').value)
        self.desired_distance_m = float(self.get_parameter('desired_distance_m').value)

        self.target_sub = self.create_subscription(PointStamped, target_topic, self.target_cb, 10)
        self.target_lost_sub = self.create_subscription(Bool, target_lost_topic, self.target_lost_cb, 10)
        self.cmd_sub = self.create_subscription(Twist, cmd_topic, self.cmd_cb, 10)
        self.source_sub = self.create_subscription(String, active_source_topic, self.source_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.samples = 0
        self.lost_samples = 0
        self.dist_abs_sum = 0.0
        self.lat_abs_sum = 0.0
        self.yaw_abs_sum = 0.0
        self.cmd_delta_sum = 0.0
        self.prev_cmd = None
        self.active_source_counts = {}
        self.last_source = 'unknown'
        self.start_sec = self.now_sec()

        self.last_target = None
        self.target_lost = True

        self.timer = self.create_timer(max(self.summary_period_sec, 0.2), self.print_summary)
        self.get_logger().info('rl_metrics_node started')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def target_cb(self, msg: PointStamped) -> None:
        self.last_target = msg
        self.samples += 1
        self.active_source_counts[self.last_source] = self.active_source_counts.get(self.last_source, 0) + 1

        if self.target_lost:
            self.lost_samples += 1
            return

        try:
            pt = self.tf_buffer.transform(
                msg,
                self.control_frame,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception:
            return

        tx = float(pt.point.x)
        ty = float(pt.point.y)
        yaw_err = math.atan2(ty, max(tx, 1e-3))
        self.dist_abs_sum += abs(tx - self.desired_distance_m)
        self.lat_abs_sum += abs(ty)
        self.yaw_abs_sum += abs(yaw_err)

    def target_lost_cb(self, msg: Bool) -> None:
        self.target_lost = bool(msg.data)

    def cmd_cb(self, msg: Twist) -> None:
        cur = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.prev_cmd is not None:
            self.cmd_delta_sum += sum(abs(c - p) for c, p in zip(cur, self.prev_cmd))
        self.prev_cmd = cur

    def source_cb(self, msg: String) -> None:
        self.last_source = msg.data.strip()

    def make_summary(self):
        elapsed = max(self.now_sec() - self.start_sec, 1e-6)
        valid = max(self.samples - self.lost_samples, 1)
        summary = {
            'elapsed_sec': round(elapsed, 3),
            'samples': int(self.samples),
            'lost_ratio': float(self.lost_samples / max(self.samples, 1)),
            'mean_abs_dist_error_m': float(self.dist_abs_sum / valid),
            'mean_abs_lateral_error_m': float(self.lat_abs_sum / valid),
            'mean_abs_yaw_error_rad': float(self.yaw_abs_sum / valid),
            'mean_cmd_delta': float(self.cmd_delta_sum / max(self.samples - 1, 1)),
            'active_source_counts': self.active_source_counts,
        }
        return summary

    def print_summary(self) -> None:
        s = self.make_summary()
        self.get_logger().info(
            f"samples={s['samples']} lost={s['lost_ratio']:.3f} "
            f"dist={s['mean_abs_dist_error_m']:.3f} lat={s['mean_abs_lateral_error_m']:.3f} "
            f"yaw={s['mean_abs_yaw_error_rad']:.3f} dcmd={s['mean_cmd_delta']:.3f}"
        )

    def destroy_node(self):
        s = self.make_summary()
        self.get_logger().info('Final metrics: ' + json.dumps(s))
        if self.save_json_path:
            try:
                with open(self.save_json_path, 'w', encoding='utf-8') as f:
                    json.dump(s, f, ensure_ascii=False, indent=2)
                self.get_logger().info(f'Saved metrics json to: {self.save_json_path}')
            except Exception as exc:
                self.get_logger().error(f'Failed to save metrics json: {exc}')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RLMetricsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
