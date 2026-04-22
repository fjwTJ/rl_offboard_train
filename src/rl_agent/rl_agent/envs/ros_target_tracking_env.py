import math
import time
from typing import Any

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from geometry_msgs.msg import PointStamped, Twist
from px4_msgs.msg import VehicleOdometry
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

try:
    import gymnasium as gym
except ImportError as exc:
    raise ImportError(
        'ros_rl_env requires gymnasium. Install it in the runtime environment before launching training.'
    ) from exc

from rl_agent.utils.rl_control_utils import normalized_action_to_twist, sanitize_observation, zero_twist


class _RosDataNode(Node):
    def __init__(
        self,
        target_topic: str,
        target_lost_topic: str,
        odom_topic: str,
        mission_active_topic: str,
        action_topic: str,
        control_frame: str,
        tf_timeout_sec: float,
    ) -> None:
        super().__init__('ros_rl_env_adapter')

        self.control_frame = control_frame
        self.tf_timeout_sec = float(tf_timeout_sec)
        self.target_msg = None
        self.last_target_time = None
        self.target_lost = True
        self.lost_since = None
        self.odom = None
        self.mission_active = False
        self.last_mission_active_time = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PointStamped, target_topic, self._target_cb, 10)
        self.create_subscription(Bool, target_lost_topic, self._target_lost_cb, 10)
        self.create_subscription(VehicleOdometry, odom_topic, self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, mission_active_topic, self._mission_active_cb, 10)
        self.action_pub = self.create_publisher(Twist, action_topic, 10)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _target_cb(self, msg: PointStamped) -> None:
        self.target_msg = msg
        self.last_target_time = self.now_sec()
        self.target_lost = False
        self.lost_since = None

    def _target_lost_cb(self, msg: Bool) -> None:
        lost = bool(msg.data)
        if lost and not self.target_lost:
            self.lost_since = self.now_sec()
        if not lost:
            self.lost_since = None
        self.target_lost = lost

    def _odom_cb(self, msg: VehicleOdometry) -> None:
        self.odom = msg

    def _mission_active_cb(self, msg: Bool) -> None:
        self.mission_active = bool(msg.data)
        self.last_mission_active_time = self.now_sec()

    def mission_active_fresh(self, timeout_sec: float) -> bool:
        if self.last_mission_active_time is None:
            return False
        return (self.now_sec() - self.last_mission_active_time) <= timeout_sec

    def target_fresh(self, timeout_sec: float) -> bool:
        if self.last_target_time is None:
            return False
        return (self.now_sec() - self.last_target_time) <= timeout_sec

    def get_target_frd(self) -> PointStamped | None:
        if self.target_msg is None:
            return None
        try:
            return self.tf_buffer.transform(
                self.target_msg,
                self.control_frame,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except Exception:
            return None

    def publish_action(self, twist: Twist) -> None:
        self.action_pub.publish(twist)


class RosTargetTrackingEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(
        self,
        target_topic: str = '/perception/target_xyz',
        target_lost_topic: str = '/perception/target_lost',
        odom_topic: str = '/fmu/out/vehicle_odometry',
        mission_active_topic: str = '/uav/mission_active',
        action_topic: str = '/uav/cmd_vel_body',
        control_frame: str = 'base_link_frd',
        obs_dim: int = 14,
        max_vx: float = 1.0,
        max_vy: float = 0.4,
        max_vz: float = 1.0,
        max_yaw_rate: float = 1.6,
        tf_timeout_sec: float = 0.08,
        mission_active_timeout_sec: float = 1.0,
        target_timeout_sec: float = 0.3,
        episode_timeout_sec: float = 120.0,
        lost_done_timeout_sec: float = 1.0,
        max_target_distance_m: float = 15.0,
        desired_distance_m: float = 3.0,
        w_dist: float = 1.0,
        w_lat: float = 0.4,
        w_vert: float = 0.3,
        w_yaw: float = 0.3,
        w_act: float = 0.05,
        lost_penalty: float = 2.0,
        alive_bonus: float = 0.2,
        step_timeout_sec: float = 5.0,
        reset_timeout_sec: float = 90.0,
        step_retry_sleep_sec: float = 0.2,
        reset_retry_sleep_sec: float = 1.0,
        max_step_timeout_retries: int = 20,
        max_reset_timeout_retries: int = 10,
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
        self.mission_active_timeout_sec = float(mission_active_timeout_sec)
        self.target_timeout_sec = float(target_timeout_sec)
        self.episode_timeout_sec = float(episode_timeout_sec)
        self.lost_done_timeout_sec = float(lost_done_timeout_sec)
        self.max_target_distance_m = float(max_target_distance_m)
        self.desired_distance_m = float(desired_distance_m)
        self.w_dist = float(w_dist)
        self.w_lat = float(w_lat)
        self.w_vert = float(w_vert)
        self.w_yaw = float(w_yaw)
        self.w_act = float(w_act)
        self.lost_penalty = float(lost_penalty)
        self.alive_bonus = float(alive_bonus)
        self.step_timeout_sec = float(step_timeout_sec)
        self.reset_timeout_sec = float(reset_timeout_sec)
        self.step_retry_sleep_sec = float(step_retry_sleep_sec)
        self.reset_retry_sleep_sec = float(reset_retry_sleep_sec)
        self.max_step_timeout_retries = int(max_step_timeout_retries)
        self.max_reset_timeout_retries = int(max_reset_timeout_retries)
        self.post_step_settle_sec = float(post_step_settle_sec)
        self.spin_timeout_sec = float(spin_timeout_sec)
        self.obs_fill_value = float(obs_fill_value)
        self.zero_action_on_reset = bool(zero_action_on_reset)
        self.zero_action_on_close = bool(zero_action_on_close)

        self._owns_rclpy = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True

        self.node = _RosDataNode(
            target_topic=target_topic,
            target_lost_topic=target_lost_topic,
            odom_topic=odom_topic,
            mission_active_topic=mission_active_topic,
            action_topic=action_topic,
            control_frame=control_frame,
            tf_timeout_sec=tf_timeout_sec,
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.obs_dim,),
            dtype=np.float32,
        )

        self.last_action = zero_twist()
        self.episode_start_time = None

    def _spin_once(self, timeout: float | None = None) -> None:
        self.executor.spin_once(timeout_sec=self.spin_timeout_sec if timeout is None else timeout)

    def _spin_until(self, predicate, timeout_sec: float, fail_msg: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self._spin_once()
            if predicate():
                return
        raise TimeoutError(fail_msg)

    def _mission_active_effective(self) -> bool:
        return self.node.mission_active and self.node.mission_active_fresh(self.mission_active_timeout_sec)

    def _episode_time_sec(self) -> float:
        if self.episode_start_time is None:
            return 0.0
        return max(0.0, self.node.now_sec() - self.episode_start_time)

    def _extract_state(self) -> dict[str, Any]:
        pt = self.node.get_target_frd()
        tx = ty = tz = float('nan')
        dist_err = float('nan')
        yaw_err = float('nan')
        target_dist = float('nan')
        if pt is not None and self.node.target_fresh(self.target_timeout_sec):
            tx = float(pt.point.x)
            ty = float(pt.point.y)
            tz = float(pt.point.z)
            dist_err = tx - self.desired_distance_m
            yaw_err = math.atan2(ty, max(tx, 1e-3))
            target_dist = math.sqrt(tx * tx + ty * ty + tz * tz)

        vx = vy = vz = yaw_rate = 0.0
        if self.node.odom is not None:
            vx = float(self.node.odom.velocity[0]) if not math.isnan(self.node.odom.velocity[0]) else 0.0
            vy = float(self.node.odom.velocity[1]) if not math.isnan(self.node.odom.velocity[1]) else 0.0
            vz = float(self.node.odom.velocity[2]) if not math.isnan(self.node.odom.velocity[2]) else 0.0
            yaw_rate = (
                float(self.node.odom.angular_velocity[2])
                if not math.isnan(self.node.odom.angular_velocity[2])
                else 0.0
            )

        act_vx = float(self.last_action.linear.x)
        act_vy = float(self.last_action.linear.y)
        act_vz = float(self.last_action.linear.z)
        act_yaw = float(self.last_action.angular.z)

        return {
            'tx': tx,
            'ty': ty,
            'tz': tz,
            'dist_err': dist_err,
            'yaw_err': yaw_err,
            'target_dist': target_dist,
            'vx': vx,
            'vy': vy,
            'vz': vz,
            'yaw_rate': yaw_rate,
            'act_vx': act_vx,
            'act_vy': act_vy,
            'act_vz': act_vz,
            'act_yaw': act_yaw,
            'target_lost': bool(self.node.target_lost),
            'target_fresh': self.node.target_fresh(self.target_timeout_sec),
            'mission_active': self._mission_active_effective(),
            'odom_ready': self.node.odom is not None,
        }

    def _current_obs(self) -> np.ndarray:
        state = self._extract_state()
        obs = [
            state['tx'],
            state['ty'],
            state['tz'],
            state['dist_err'],
            state['yaw_err'],
            state['vx'],
            state['vy'],
            state['vz'],
            state['yaw_rate'],
            state['act_vx'],
            state['act_vy'],
            state['act_vz'],
            state['act_yaw'],
            1.0 if state['target_lost'] else 0.0,
        ]
        return sanitize_observation(obs, expected_dim=self.obs_dim, fill_value=self.obs_fill_value)

    def _current_reward(self) -> float:
        state = self._extract_state()
        if not state['target_fresh']:
            return -self.lost_penalty

        act_mag = abs(state['act_vx']) + abs(state['act_vy']) + abs(state['act_vz']) + abs(state['act_yaw'])
        reward = 0.0
        reward -= self.w_dist * abs(state['dist_err'])
        reward -= self.w_lat * abs(state['ty'])
        reward -= self.w_vert * abs(state['tz'])
        reward -= self.w_yaw * abs(state['yaw_err'])
        reward -= self.w_act * act_mag
        reward += self.alive_bonus
        return float(reward)

    def _current_done(self) -> tuple[bool, str]:
        state = self._extract_state()
        if not state['mission_active']:
            return True, 'mission_inactive'
        if self.episode_start_time is not None and self._episode_time_sec() >= self.episode_timeout_sec:
            return True, 'episode_timeout'
        if (
            state['target_lost']
            and self.node.lost_since is not None
            and (self.node.now_sec() - self.node.lost_since) >= self.lost_done_timeout_sec
        ):
            return True, 'target_lost_timeout'
        if (not math.isnan(state['target_dist'])) and state['target_dist'] > self.max_target_distance_m:
            return True, 'target_too_far'
        return False, 'running'

    def _current_info(self) -> dict[str, Any]:
        state = self._extract_state()
        terminated, done_reason = self._current_done()
        return {
            'done_reason': done_reason,
            'episode_time_sec': round(self._episode_time_sec(), 3),
            'target_dist': None if math.isnan(state['target_dist']) else round(state['target_dist'], 4),
            'target_fresh': state['target_fresh'],
            'mission_active': state['mission_active'],
            'target_lost': state['target_lost'],
            'terminated': terminated,
        }

    def _publish_zero_action(self) -> None:
        self.last_action = zero_twist()
        self.node.publish_action(self.last_action)

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        timeout_count = 0
        while True:
            if self.zero_action_on_reset:
                self._publish_zero_action()

            try:
                self._spin_until(
                    lambda: self._mission_active_effective() and self.node.odom is not None,
                    timeout_sec=self.reset_timeout_sec,
                    fail_msg='timeout waiting for mission_active=true and fresh odometry',
                )
                self.episode_start_time = self.node.now_sec()
                return self._current_obs(), self._current_info()
            except TimeoutError:
                timeout_count += 1
                if self.max_reset_timeout_retries > 0 and timeout_count >= self.max_reset_timeout_retries:
                    raise TimeoutError(
                        'reset exceeded retry budget while waiting for mission_active=true and fresh odometry; '
                        f'mission_active={self.node.mission_active}, odom_ready={self.node.odom is not None}'
                    )
                self.node.get_logger().warn(
                    'reset timeout while waiting for next mission episode; '
                    f'retrying in {self.reset_retry_sleep_sec:.1f}s '
                    f'(retries={timeout_count}, mission_active={self.node.mission_active})'
                )
                sleep_deadline = time.monotonic() + self.reset_retry_sleep_sec
                while time.monotonic() < sleep_deadline:
                    self._spin_once()

    def step(self, action):
        if not self._mission_active_effective():
            self._spin_until(
                self._mission_active_effective,
                timeout_sec=self.reset_timeout_sec,
                fail_msg='timeout waiting for /uav/mission_active before publishing action',
            )

        twist = normalized_action_to_twist(
            action,
            max_vx=self.max_vx,
            max_vy=self.max_vy,
            max_vz=self.max_vz,
            max_yaw_rate=self.max_yaw_rate,
        )
        self.last_action = twist
        self.node.publish_action(twist)

        step_timeout_count = 0
        while True:
            try:
                deadline = time.monotonic() + self.step_timeout_sec
                while time.monotonic() < deadline:
                    self._spin_once()
                    terminated, _done_reason = self._current_done()
                    if terminated or self.node.odom is not None:
                        settle_deadline = time.monotonic() + self.post_step_settle_sec
                        while time.monotonic() < settle_deadline:
                            self._spin_once()
                        obs = self._current_obs()
                        reward = self._current_reward()
                        terminated, _done_reason = self._current_done()
                        info = self._current_info()
                        info.setdefault('step_timeout_retries', step_timeout_count)
                        return obs, reward, terminated, False, info
                raise TimeoutError('timeout waiting for a fresh environment sample after publishing action')
            except TimeoutError:
                step_timeout_count += 1
                if self.max_step_timeout_retries > 0 and step_timeout_count >= self.max_step_timeout_retries:
                    raise TimeoutError(
                        'step exceeded retry budget while waiting for next transition; '
                        f'mission_active={self.node.mission_active}, odom_ready={self.node.odom is not None}'
                    )
                self.node.get_logger().warn(
                    'step timeout while waiting for next transition; '
                    f'retrying in {self.step_retry_sleep_sec:.1f}s '
                    f'(retries={step_timeout_count}, mission_active={self.node.mission_active})'
                )
                sleep_deadline = time.monotonic() + self.step_retry_sleep_sec
                while time.monotonic() < sleep_deadline:
                    self._spin_once()

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
