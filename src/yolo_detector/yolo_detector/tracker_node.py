import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs
import math

class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        self.sub = self.create_subscription(PointStamped, '/perception/target_xyz', self.callback, 10)
        self.pub = self.create_publisher(Twist, '/uav/cmd_vel_body', 10)
        self.lost_pub = self.create_publisher(Bool, '/perception/target_lost', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.declare_parameter("use_sim_tf", True)
        self.use_sim_tf = self.get_parameter("use_sim_tf").get_parameter_value().bool_value
        self._publish_static_tf()
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
        self.target_timeout = Duration(seconds=0.3)
        self.last_target_time = None
        self.lost_target = True
        self.timer = self.create_timer(0.1, self.publish_target_status)

    def _publish_static_tf(self):
        # Toggle sim vs real via parameter: use_sim_tf (true=sim, false=real).
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link_frd"
        if self.use_sim_tf:
            # 仿真环境下的TF
            # Sim: base_link_frd -> camera_optical_frame
            # Camera position: +0.12m forward, 0.03m left, 0.242m up (FRD => left/up are negative).
            # Rotation: optical (X right, Y down, Z forward) relative to FRD (X forward, Y right, Z down).
            t.child_frame_id = "x500_depth_0/RealSenseD455/base_link/RealSenseD455/rgbd"
            t.transform.translation.x = 0.12
            t.transform.translation.y = -0.03
            t.transform.translation.z = -0.242
            t.transform.rotation.x = 0.5
            t.transform.rotation.y = 0.5
            t.transform.rotation.z = 0.5
            t.transform.rotation.w = 0.5
        else:
            # 实际环境下的TF
            # Real: base_link_frd -> camera_link
            # Camera position: +0.15m forward, 0.00m right, 0.10m down in FRD.
            # Rotation: camera_link aligned with base_link_frd.
            t.child_frame_id = "camera_link"
            t.transform.translation.x = 0.15
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.10
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(t)

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
        self.last_target_time = self.get_clock().now()

        self.get_logger().info(f"cmd: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw_rate={yaw_rate:.2f}")
        self.get_logger().info(f"FRD XYZ: ({X:.2f}, {Y:.2f}, {Z:.2f})")

    def publish_target_status(self):
        now = self.get_clock().now()
        have_target = (
            self.last_target_time is not None
            and (now - self.last_target_time) <= self.target_timeout
        )
        lost = not have_target
        if lost != self.lost_target:
            self.lost_target = lost
            state = "lost" if lost else "reacquired"
            self.get_logger().info(f"Target {state}.")

        self.lost_pub.publish(Bool(data=lost))



def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
