import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class CmdVelMuxNode(Node):
    """
    Select between PID and RL velocity commands, then publish one command stream
    to /uav/cmd_vel_body for PX4 offboard input.
    """

    def __init__(self) -> None:
        super().__init__('cmd_vel_mux_node')

        self.declare_parameter('pid_topic', '/uav/cmd_vel_pid')
        self.declare_parameter('rl_topic', '/uav/cmd_vel_rl')
        self.declare_parameter('output_topic', '/uav/cmd_vel_body')
        self.declare_parameter('target_lost_topic', '/perception/target_lost')
        self.declare_parameter('mode_topic', '/uav/cmd_source')
        self.declare_parameter('mode', 'pid')  # pid | rl
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.2)
        self.declare_parameter('fallback_to_pid', True)
        self.declare_parameter('hold_when_target_lost', True)

        pid_topic = self.get_parameter('pid_topic').value
        rl_topic = self.get_parameter('rl_topic').value
        output_topic = self.get_parameter('output_topic').value
        target_lost_topic = self.get_parameter('target_lost_topic').value
        mode_topic = self.get_parameter('mode_topic').value

        self.mode = str(self.get_parameter('mode').value).lower()
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.fallback_to_pid = bool(self.get_parameter('fallback_to_pid').value)
        self.hold_when_target_lost = bool(self.get_parameter('hold_when_target_lost').value)

        self.pid_sub = self.create_subscription(Twist, pid_topic, self.pid_cb, 10)
        self.rl_sub = self.create_subscription(Twist, rl_topic, self.rl_cb, 10)
        self.target_lost_sub = self.create_subscription(Bool, target_lost_topic, self.target_lost_cb, 10)
        self.mode_sub = self.create_subscription(String, mode_topic, self.mode_cb, 10)

        self.output_pub = self.create_publisher(Twist, output_topic, 10)
        self.active_source_pub = self.create_publisher(String, '/uav/cmd_active_source', 10)

        self.last_pid = Twist()
        self.last_rl = Twist()
        self.last_pid_time = None
        self.last_rl_time = None
        self.target_lost = True

        period = 1.0 / max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info(
            f'cmd_vel_mux_node started. pid={pid_topic}, rl={rl_topic}, out={output_topic}, mode={self.mode}'
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def is_fresh(self, t: float) -> bool:
        if t is None:
            return False
        return (self.now_sec() - t) <= self.cmd_timeout_sec

    def pid_cb(self, msg: Twist) -> None:
        self.last_pid = msg
        self.last_pid_time = self.now_sec()

    def rl_cb(self, msg: Twist) -> None:
        self.last_rl = msg
        self.last_rl_time = self.now_sec()

    def target_lost_cb(self, msg: Bool) -> None:
        self.target_lost = bool(msg.data)

    def mode_cb(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode in ('pid', 'rl'):
            self.mode = mode

    def timer_cb(self) -> None:
        out = Twist()
        source = 'zero'

        if self.hold_when_target_lost and self.target_lost:
            source = 'hold_lost'
        elif self.mode == 'rl':
            if self.is_fresh(self.last_rl_time):
                out = self.last_rl
                source = 'rl'
            elif self.fallback_to_pid and self.is_fresh(self.last_pid_time):
                out = self.last_pid
                source = 'pid_fallback'
        else:
            if self.is_fresh(self.last_pid_time):
                out = self.last_pid
                source = 'pid'
            elif self.is_fresh(self.last_rl_time):
                out = self.last_rl
                source = 'rl_fallback'

        self.output_pub.publish(out)
        self.active_source_pub.publish(String(data=source))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
