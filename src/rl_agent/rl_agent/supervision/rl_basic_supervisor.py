import os
import shlex
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RlBasicSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('rl_basic_supervisor')
        self.declare_parameter('basic_launch_args', '')
        self.declare_parameter('reset_timeout_event_topic', '/rl/reset_timeout')
        self.declare_parameter('restart_cooldown_sec', 10.0)
        self.declare_parameter('shutdown_timeout_sec', 8.0)
        self.declare_parameter('kill_timeout_sec', 4.0)

        self.basic_launch_args = self.get_parameter('basic_launch_args').value
        self.restart_cooldown_sec = float(self.get_parameter('restart_cooldown_sec').value)
        self.shutdown_timeout_sec = float(self.get_parameter('shutdown_timeout_sec').value)
        self.kill_timeout_sec = float(self.get_parameter('kill_timeout_sec').value)
        reset_timeout_event_topic = self.get_parameter('reset_timeout_event_topic').value

        self.process: subprocess.Popen | None = None
        self.last_restart_time = 0.0
        self.exited_reported = False

        self.create_subscription(String, reset_timeout_event_topic, self._reset_timeout_cb, 10)
        self.create_timer(1.0, self._poll_process)

        self._start_basic()

    def _build_command(self) -> list[str]:
        args = shlex.split(str(self.basic_launch_args))
        return ['ros2', 'launch', 'rl_agent', 'rl_basic.launch.py', *args]

    def _start_basic(self) -> None:
        cmd = self._build_command()
        self.get_logger().info('Starting rl_basic.launch.py: ' + shlex.join(cmd))
        self.process = subprocess.Popen(cmd, start_new_session=True)
        self.exited_reported = False

    def _poll_process(self) -> None:
        if self.process is None:
            return
        return_code = self.process.poll()
        if return_code is None or self.exited_reported:
            return
        self.exited_reported = True
        self.get_logger().error(
            f'rl_basic.launch.py exited with code {return_code}; waiting for reset timeout before restarting'
        )

    def _reset_timeout_cb(self, msg: String) -> None:
        now = time.monotonic()
        if now - self.last_restart_time < self.restart_cooldown_sec:
            self.get_logger().warn(
                f'Ignoring reset timeout during restart cooldown: {msg.data}'
            )
            return
        self.last_restart_time = now
        self.get_logger().warn(f'Reset timeout received; restarting rl_basic.launch.py: {msg.data}')
        self._restart_basic()

    def _restart_basic(self) -> None:
        self._stop_basic()
        self._start_basic()

    def _stop_basic(self) -> None:
        if self.process is None:
            return
        if self.process.poll() is not None:
            self.process = None
            return

        pid = self.process.pid
        self.get_logger().info(f'Stopping rl_basic.launch.py process group pid={pid}')
        self._signal_process_group(signal.SIGINT)
        if self._wait_for_exit(self.shutdown_timeout_sec):
            self.process = None
            return

        self.get_logger().warn('rl_basic.launch.py did not exit after SIGINT; sending SIGTERM')
        self._signal_process_group(signal.SIGTERM)
        if self._wait_for_exit(self.kill_timeout_sec):
            self.process = None
            return

        self.get_logger().error('rl_basic.launch.py did not exit after SIGTERM; sending SIGKILL')
        self._signal_process_group(signal.SIGKILL)
        self._wait_for_exit(self.kill_timeout_sec)
        self.process = None

    def _signal_process_group(self, sig: signal.Signals) -> None:
        if self.process is None:
            return
        try:
            os.killpg(self.process.pid, sig)
        except ProcessLookupError:
            pass

    def _wait_for_exit(self, timeout_sec: float) -> bool:
        if self.process is None:
            return True
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                return True
            time.sleep(0.1)
        return self.process.poll() is not None

    def destroy_node(self) -> bool:
        self._stop_basic()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RlBasicSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
