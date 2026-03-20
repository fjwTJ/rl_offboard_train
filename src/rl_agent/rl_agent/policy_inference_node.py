import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

try:
    from stable_baselines3 import PPO
except ImportError as exc:
    raise ImportError(
        'policy_inference_node requires stable-baselines3. Install it in the runtime environment before launching inference.'
    ) from exc

from rl_agent.rl_control_utils import normalized_action_to_twist, sanitize_observation, zero_twist


class PolicyInferenceNode(Node):
    def __init__(self) -> None:
        super().__init__('policy_inference_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('obs_topic', '/rl/obs')
        self.declare_parameter('done_topic', '/rl/done')
        self.declare_parameter('cmd_topic', '/uav/cmd_vel_rl')
        self.declare_parameter('obs_dim', 14)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('deterministic', True)
        self.declare_parameter('zero_on_done', True)
        self.declare_parameter('obs_fill_value', 0.0)
        self.declare_parameter('max_vx', 1.0)
        self.declare_parameter('max_vy', 0.4)
        self.declare_parameter('max_vz', 1.0)
        self.declare_parameter('max_yaw_rate', 1.6)

        model_path = str(self.get_parameter('model_path').value).strip()
        if not model_path:
            raise ValueError('parameter model_path must be set')

        self.obs_dim = int(self.get_parameter('obs_dim').value)
        self.deterministic = bool(self.get_parameter('deterministic').value)
        self.zero_on_done = bool(self.get_parameter('zero_on_done').value)
        self.obs_fill_value = float(self.get_parameter('obs_fill_value').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_vz = float(self.get_parameter('max_vz').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)

        obs_topic = str(self.get_parameter('obs_topic').value)
        done_topic = str(self.get_parameter('done_topic').value)
        cmd_topic = str(self.get_parameter('cmd_topic').value)

        self.model = PPO.load(model_path, device='auto')
        self.last_obs = None
        self.last_done = False

        self.create_subscription(Float32MultiArray, obs_topic, self.obs_cb, 10)
        self.create_subscription(Bool, done_topic, self.done_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        period = 1.0 / max(float(self.get_parameter('control_rate_hz').value), 1.0)
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info(f'policy_inference_node loaded model: {model_path}')

    def obs_cb(self, msg: Float32MultiArray) -> None:
        self.last_obs = sanitize_observation(msg.data, self.obs_dim, fill_value=self.obs_fill_value)

    def done_cb(self, msg: Bool) -> None:
        self.last_done = bool(msg.data)

    def timer_cb(self) -> None:
        if self.last_obs is None:
            self.cmd_pub.publish(zero_twist())
            return

        if self.zero_on_done and self.last_done:
            self.cmd_pub.publish(zero_twist())
            return

        action, _state = self.model.predict(self.last_obs, deterministic=self.deterministic)
        twist = normalized_action_to_twist(
            np.asarray(action, dtype=np.float32),
            max_vx=self.max_vx,
            max_vy=self.max_vy,
            max_vz=self.max_vz,
            max_yaw_rate=self.max_yaw_rate,
        )
        self.cmd_pub.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyInferenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(zero_twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
