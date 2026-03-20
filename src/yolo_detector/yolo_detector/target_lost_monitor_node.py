import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class TargetLostMonitorNode(Node):
    """Publish /perception/target_lost from target_xyz freshness."""

    def __init__(self) -> None:
        super().__init__('target_lost_monitor_node')

        self.declare_parameter('target_topic', '/perception/target_xyz')
        self.declare_parameter('target_lost_topic', '/perception/target_lost')
        self.declare_parameter('target_timeout_sec', 1.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        target_topic = str(self.get_parameter('target_topic').value)
        target_lost_topic = str(self.get_parameter('target_lost_topic').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.sub = self.create_subscription(PointStamped, target_topic, self.target_cb, 10)
        self.pub = self.create_publisher(Bool, target_lost_topic, 10)

        self.last_target_time = None
        self.last_lost = True
        period = 1.0 / max(publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info(
            f'target_lost_monitor_node started. target={target_topic} lost_topic={target_lost_topic}'
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def target_cb(self, _msg: PointStamped) -> None:
        self.last_target_time = self.now_sec()

    def timer_cb(self) -> None:
        if self.last_target_time is None:
            lost = True
        else:
            lost = (self.now_sec() - self.last_target_time) > self.target_timeout_sec

        self.pub.publish(Bool(data=lost))
        if lost != self.last_lost:
            self.last_lost = lost
            self.get_logger().info('target_lost -> %s' % ('true' if lost else 'false'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetLostMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
