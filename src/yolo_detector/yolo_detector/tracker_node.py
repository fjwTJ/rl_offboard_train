import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import math

class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        self.sub = self.create_subscription(PointStamped, '/perception/target_xyz', self.callback, 10)
        self.pub = self.create_publisher(Twist, '/uav/cmd_vel_pid', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # PID 参数
        self.kx = 1.0  # 前后
        self.ky = 0.6  # 左右
        self.kz = 0.6  # 上下
        self.k_yaw = 1.5  # 偏航
        self.desired_dist = 3  # 目标距离 m
        self.yaw_enable_vy_rad = math.radians(45.0)
        # 低通滤波参数
        self.yaw_lpf_tau = 0.3
        self.vy_lpf_tau = 0.4
        self._last_time = None
        self._yaw_filt = 0.0
        self._vy_filt = 0.0

    def callback(self, msg):
        try:
            pt_frd = self.tf_buffer.transform(
                msg, "base_link_frd", timeout=Duration(seconds=0.1)
            )
        except Exception as exc:
            self.get_logger().warn(f"TF transform failed: {exc}")
            return

        X, Y, Z = pt_frd.point.x, pt_frd.point.y, pt_frd.point.z
        yaw_error = math.atan2(Y, X)
        # 前后误差：希望距离固定
        vx = self.kx * (X - self.desired_dist)

        # yaw 对齐后再逐步启用侧向速度
        beta = 1.0 - min(abs(yaw_error) / self.yaw_enable_vy_rad, 1.0)
        vy = beta * self.ky * Y

        vz = self.kz * Z
        yaw_rate = self.k_yaw * yaw_error

        # vy、yaw_rate低通滤波
        now = self.get_clock().now().nanoseconds
        if self._last_time is None:
            dt = 0.0
        else:
            dt = (now - self._last_time) * 1e-9
        self._last_time = now
        if dt > 0.0:
            yaw_alpha = dt / (self.yaw_lpf_tau + dt)
            vy_alpha = dt / (self.vy_lpf_tau + dt)
            self._yaw_filt += yaw_alpha * (yaw_rate - self._yaw_filt)
            self._vy_filt += vy_alpha * (vy - self._vy_filt)
        else:
            self._yaw_filt = yaw_rate
            self._vy_filt = vy

        yaw_rate = self._yaw_filt
        vy = self._vy_filt

        # 限幅
        vx = max(min(vx, 1.0), -1.0)
        vy = max(min(vy, 0.4), -0.4)
        vz = max(min(vz, 1.0), -1.0)
        yaw_rate = max(min(yaw_rate, 1.6), -1.6)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = yaw_rate
        self.pub.publish(cmd)

        self.get_logger().info(f"cmd: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw_rate={yaw_rate:.2f}")
        self.get_logger().info(f"FRD XYZ: ({X:.2f}, {Y:.2f}, {Z:.2f})")



def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
