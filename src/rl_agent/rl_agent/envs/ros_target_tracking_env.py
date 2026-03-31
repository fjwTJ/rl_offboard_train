import time
from typing import Any

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32, String
from geometry_msgs.msg import Twist

try:
    import gymnasium as gym
except ImportError as exc:
    raise ImportError(
        'ros_rl_env requires gymnasium. Install it in the runtime environment before launching training.'
    ) from exc

from rl_agent.utils.rl_control_utils import (
    normalized_action_to_twist,
    parse_info_json,
    sanitize_observation,
    zero_twist,
)


class _RosRlInterfaceNode(Node):
    def __init__(
        self,
        obs_topic: str,
        reward_topic: str,
        done_topic: str,
        info_topic: str,
        step_topic: str,
        mission_active_topic: str,
        action_topic: str,
    ) -> None:
        super().__init__('ros_rl_env_adapter')

        self.obs = None
        self.reward = 0.0
        self.done = False
        self.info_raw = ''
        self.step_count = None
        self.mission_active = False
        self.last_obs_time = None
        self.last_step_time = None

        self.create_subscription(Float32MultiArray, obs_topic, self._obs_cb, 10)
        self.create_subscription(Float32, reward_topic, self._reward_cb, 10)
        self.create_subscription(Bool, done_topic, self._done_cb, 10)
        self.create_subscription(String, info_topic, self._info_cb, 10)
        self.create_subscription(Int32, step_topic, self._step_cb, 10)
        self.create_subscription(Bool, mission_active_topic, self._mission_active_cb, 10)
        self.action_pub = self.create_publisher(Twist, action_topic, 10)

    def _obs_cb(self, msg: Float32MultiArray) -> None:
        self.obs = list(msg.data)
        self.last_obs_time = time.monotonic()

    def _reward_cb(self, msg: Float32) -> None:
        self.reward = float(msg.data)

    def _done_cb(self, msg: Bool) -> None:
        self.done = bool(msg.data)

    def _info_cb(self, msg: String) -> None:
        self.info_raw = msg.data

    def _step_cb(self, msg: Int32) -> None:
        self.step_count = int(msg.data)
        self.last_step_time = time.monotonic()

    def _mission_active_cb(self, msg: Bool) -> None:
        self.mission_active = bool(msg.data)

    def publish_action(self, twist: Twist) -> None:
        self.action_pub.publish(twist)


class RosTargetTrackingEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(
        self,
        obs_topic: str = '/rl/obs',
        reward_topic: str = '/rl/reward',
        done_topic: str = '/rl/done',
        info_topic: str = '/rl/info',
        step_topic: str = '/rl/step_count',
        mission_active_topic: str = '/uav/mission_active',
        action_topic: str = '/uav/cmd_vel_rl',
        obs_dim: int = 14,
        max_vx: float = 1.0,
        max_vy: float = 0.4,
        max_vz: float = 1.0,
        max_yaw_rate: float = 1.6,
        step_timeout_sec: float = 5.0,
        reset_timeout_sec: float = 90.0,
        step_retry_sleep_sec: float = 0.2,
        reset_retry_sleep_sec: float = 1.0,
        max_step_timeout_retries: int = 20,
        max_reset_timeout_retries: int = 10,
        mission_inactive_step_grace_sec: float = 0.5,
        post_step_settle_sec: float = 0.03,
        spin_timeout_sec: float = 0.01,
        obs_fill_value: float = 0.0,
        zero_action_on_reset: bool = True,
        zero_action_on_close: bool = True,
    ) -> None:
        super().__init__()

        self.obs_dim = int(obs_dim)
        self.max_vx = float(max_vx)
        self.max_vy = float(max_vy)
        self.max_vz = float(max_vz)
        self.max_yaw_rate = float(max_yaw_rate)
        self.step_timeout_sec = float(step_timeout_sec)
        self.reset_timeout_sec = float(reset_timeout_sec)
        self.step_retry_sleep_sec = float(step_retry_sleep_sec)
        self.reset_retry_sleep_sec = float(reset_retry_sleep_sec)
        self.max_step_timeout_retries = int(max_step_timeout_retries)
        self.max_reset_timeout_retries = int(max_reset_timeout_retries)
        self.mission_inactive_step_grace_sec = float(mission_inactive_step_grace_sec)
        self.post_step_settle_sec = float(post_step_settle_sec)
        self.spin_timeout_sec = float(spin_timeout_sec)
        self.obs_fill_value = float(obs_fill_value)
        self.zero_action_on_reset = bool(zero_action_on_reset)
        self.zero_action_on_close = bool(zero_action_on_close)

        self._owns_rclpy = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True

        self.node = _RosRlInterfaceNode(
            obs_topic=obs_topic,
            reward_topic=reward_topic,
            done_topic=done_topic,
            info_topic=info_topic,
            step_topic=step_topic,
            mission_active_topic=mission_active_topic,
            action_topic=action_topic,
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(4,),
            dtype=np.float32,
        )
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.obs_dim,),
            dtype=np.float32,
        )

    def _spin_once(self, timeout: float | None = None) -> None:
        self.executor.spin_once(timeout_sec=self.spin_timeout_sec if timeout is None else timeout)

    def _spin_until(self, predicate, timeout_sec: float, fail_msg: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self._spin_once()
            if predicate():
                return
        raise TimeoutError(fail_msg)

    def _current_obs(self) -> np.ndarray:
        return sanitize_observation(self.node.obs, expected_dim=self.obs_dim, fill_value=self.obs_fill_value)

    def _current_info(self) -> dict[str, Any]:
        info = parse_info_json(self.node.info_raw)
        if self.node.step_count is not None:
            info.setdefault('step_count', self.node.step_count)
        info.setdefault('mission_active', self.node.mission_active)
        return info

    def _publish_zero_action(self) -> None:
        self.node.publish_action(zero_twist())

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        timeout_count = 0
        while True:
            ref_step = self.node.step_count
            ref_done = self.node.done

            if self.zero_action_on_reset:
                self._publish_zero_action()

            def reset_ready() -> bool:
                if (not self.node.mission_active) or self.node.obs is None or self.node.done:
                    return False
                if self.node.step_count is None:
                    return True
                if ref_step is None:
                    return True
                if self.node.step_count < ref_step:
                    return True
                if ref_done and self.node.step_count != ref_step:
                    return True
                return not ref_done

            try:
                self._spin_until(
                    reset_ready,
                    timeout_sec=self.reset_timeout_sec,
                    fail_msg='timeout waiting for mission_active=true and a fresh reset observation from /rl/obs',
                )
                if timeout_count > 0:
                    self.node.get_logger().info(
                        f'reset recovered after {timeout_count} timeout retries; mission_active={self.node.mission_active}'
                    )
                return self._current_obs(), self._current_info()
            except TimeoutError:
                timeout_count += 1
                if self.max_reset_timeout_retries > 0 and timeout_count >= self.max_reset_timeout_retries:
                    raise TimeoutError(
                        'reset exceeded retry budget while waiting for mission_active=true and fresh observation; '
                        f'mission_active={self.node.mission_active}, done={self.node.done}, '
                        f'step_count={self.node.step_count}, obs_ready={self.node.obs is not None}'
                    )
                self.node.get_logger().warn(
                    'reset timeout while waiting for next mission episode; '
                    f'retrying in {self.reset_retry_sleep_sec:.1f}s '
                    f'(retries={timeout_count}, mission_active={self.node.mission_active}, done={self.node.done})'
                )
                sleep_deadline = time.monotonic() + self.reset_retry_sleep_sec
                while time.monotonic() < sleep_deadline:
                    self._spin_once()

    def step(self, action):
        if not self.node.mission_active:
            self._spin_until(
                lambda: self.node.mission_active or self.node.done,
                timeout_sec=self.reset_timeout_sec,
                fail_msg='timeout waiting for /uav/mission_active or terminal transition before publishing action',
            )

        if self.node.done and not self.node.mission_active:
            obs = self._current_obs()
            reward = float(self.node.reward)
            terminated = True
            truncated = False
            info = self._current_info()
            return obs, reward, terminated, truncated, info

        ref_step = self.node.step_count
        twist = normalized_action_to_twist(
            action,
            max_vx=self.max_vx,
            max_vy=self.max_vy,
            max_vz=self.max_vz,
            max_yaw_rate=self.max_yaw_rate,
        )
        self.node.publish_action(twist)

        def next_step_ready() -> bool:
            if self.node.step_count is None:
                return self.node.obs is not None and ref_step is None
            if ref_step is None:
                return True
            return self.node.step_count > ref_step or self.node.step_count < ref_step

        step_timeout_count = 0
        inactive_since = None
        while True:
            try:
                deadline = time.monotonic() + self.step_timeout_sec
                while time.monotonic() < deadline:
                    self._spin_once()
                    if next_step_ready():
                        inactive_since = None
                        raise StopIteration
                    if self.node.done and not self.node.mission_active:
                        obs = self._current_obs()
                        reward = float(self.node.reward)
                        terminated = True
                        truncated = False
                        info = self._current_info()
                        info.setdefault('step_timeout_retries', step_timeout_count)
                        return obs, reward, terminated, truncated, info
                    if not self.node.mission_active:
                        if inactive_since is None:
                            inactive_since = time.monotonic()
                        elif (time.monotonic() - inactive_since) >= self.mission_inactive_step_grace_sec:
                            obs = self._current_obs()
                            reward = float(self.node.reward)
                            terminated = True
                            truncated = False
                            info = self._current_info()
                            info.setdefault('done_reason', 'mission_inactive_env_fallback')
                            info.setdefault('step_timeout_retries', step_timeout_count)
                            return obs, reward, terminated, truncated, info
                    else:
                        inactive_since = None
                raise TimeoutError('timeout waiting for next /rl/step_count after publishing action')
            except StopIteration:
                break
            except TimeoutError:
                step_timeout_count += 1
                if self.node.done and not self.node.mission_active:
                    obs = self._current_obs()
                    reward = float(self.node.reward)
                    terminated = True
                    truncated = False
                    info = self._current_info()
                    info.setdefault('step_timeout_retries', step_timeout_count)
                    return obs, reward, terminated, truncated, info
                if self.max_step_timeout_retries > 0 and step_timeout_count >= self.max_step_timeout_retries:
                    raise TimeoutError(
                        'step exceeded retry budget while waiting for next transition; '
                        f'mission_active={self.node.mission_active}, done={self.node.done}, '
                        f'step_count={self.node.step_count}, obs_ready={self.node.obs is not None}'
                    )
                self.node.get_logger().warn(
                    'step timeout while waiting for next transition; '
                    f'retrying in {self.step_retry_sleep_sec:.1f}s '
                    f'(retries={step_timeout_count}, mission_active={self.node.mission_active}, '
                    f'done={self.node.done}, step_count={self.node.step_count})'
                )
                sleep_deadline = time.monotonic() + self.step_retry_sleep_sec
                while time.monotonic() < sleep_deadline:
                    self._spin_once()

        settle_deadline = time.monotonic() + self.post_step_settle_sec
        while time.monotonic() < settle_deadline:
            self._spin_once()

        obs = self._current_obs()
        reward = float(self.node.reward)
        terminated = bool(self.node.done)
        truncated = False
        info = self._current_info()
        return obs, reward, terminated, truncated, info

    def close(self) -> None:
        if getattr(self, 'node', None) is not None:
            if self.zero_action_on_close:
                self._publish_zero_action()
                self._spin_once(timeout=0.01)
            self.executor.remove_node(self.node)
            self.node.destroy_node()
            self.node = None
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
