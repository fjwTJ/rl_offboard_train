import os
import rclpy
import shlex
import signal
import subprocess
import time
from rclpy.node import Node
from std_msgs.msg import Bool


class EpisodeManagerNode(Node):
    """Episode flow: done -> stop offboard/perception -> restart PX4/Gazebo -> start perception/offboard -> cooldown."""

    def __init__(self) -> None:
        super().__init__('episode_manager_node')

        self.declare_parameter('done_topic', '/rl/done')
        self.declare_parameter('reset_topic', '/rl/reset')
        self.declare_parameter('mission_active_topic', '/uav/mission_active')
        self.declare_parameter('px4_dir', '')
        self.declare_parameter('px4_run_cmd', 'env HEADLESS=1 make px4_sitl_default gz_x500_depth')
        self.declare_parameter('px4_startup_wait_sec', 8.0)
        self.declare_parameter('yolo_run_cmd', 'ros2 run yolo_detector yolo_node')
        self.declare_parameter('target_depth_run_cmd', 'ros2 run yolo_detector target_depth_node')
        self.declare_parameter('target_lost_monitor_run_cmd', 'ros2 run yolo_detector target_lost_monitor_node')
        self.declare_parameter('perception_startup_grace_sec', 1.0)
        self.declare_parameter('offboard_run_cmd', 'ros2 run px4_ros_com offboard_control_srv')
        self.declare_parameter('stop_timeout_sec', 5.0)
        self.declare_parameter('offboard_startup_grace_sec', 1.0)
        self.declare_parameter('mission_start_timeout_sec', 20.0)
        self.declare_parameter('cooldown_sec', 0.5)
        self.declare_parameter('reset_pulse_sec', 0.2)
        self.declare_parameter('tick_hz', 20.0)
        self.declare_parameter('cleanup_orphans_before_px4_start', True)
        self.declare_parameter('cleanup_orphans_on_shutdown', True)
        self.declare_parameter('orphan_cleanup_grace_sec', 1.0)

        done_topic = str(self.get_parameter('done_topic').value)
        reset_topic = str(self.get_parameter('reset_topic').value)
        mission_active_topic = str(self.get_parameter('mission_active_topic').value)
        self.px4_dir = str(self.get_parameter('px4_dir').value)
        self.px4_run_cmd = shlex.split(str(self.get_parameter('px4_run_cmd').value))
        self.px4_startup_wait_sec = float(self.get_parameter('px4_startup_wait_sec').value)
        self.yolo_run_cmd = shlex.split(str(self.get_parameter('yolo_run_cmd').value))
        self.target_depth_run_cmd = shlex.split(str(self.get_parameter('target_depth_run_cmd').value))
        self.target_lost_monitor_run_cmd = shlex.split(str(self.get_parameter('target_lost_monitor_run_cmd').value))
        self.perception_startup_grace_sec = float(self.get_parameter('perception_startup_grace_sec').value)
        self.offboard_run_cmd = shlex.split(str(self.get_parameter('offboard_run_cmd').value))
        self.stop_timeout_sec = float(self.get_parameter('stop_timeout_sec').value)
        self.offboard_startup_grace_sec = float(self.get_parameter('offboard_startup_grace_sec').value)
        self.mission_start_timeout_sec = float(self.get_parameter('mission_start_timeout_sec').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)
        self.reset_pulse_sec = float(self.get_parameter('reset_pulse_sec').value)
        self.cleanup_orphans_before_px4_start = bool(
            self.get_parameter('cleanup_orphans_before_px4_start').value
        )
        self.cleanup_orphans_on_shutdown = bool(self.get_parameter('cleanup_orphans_on_shutdown').value)
        self.orphan_cleanup_grace_sec = float(self.get_parameter('orphan_cleanup_grace_sec').value)

        self.done_sub = self.create_subscription(Bool, done_topic, self.done_cb, 10)
        self.mission_active_sub = self.create_subscription(Bool, mission_active_topic, self.mission_active_cb, 10)
        self.reset_pub = self.create_publisher(Bool, reset_topic, 10)

        self.last_done = False
        self.mission_active = False
        self.phase = 'idle'
        self.phase_start_sec = self.now_sec()
        self.px4_process = None
        self.yolo_process = None
        self.target_depth_process = None
        self.target_lost_monitor_process = None
        self.offboard_process = None
        self.reset_release_sec = None

        period = 1.0 / max(float(self.get_parameter('tick_hz').value), 1.0)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(
            'episode_manager_node(process-driven) started. '
            f'px4_startup_wait_sec={self.px4_startup_wait_sec:.1f} '
            f'perception_startup_grace_sec={self.perception_startup_grace_sec:.1f} '
            f'offboard_startup_grace_sec={self.offboard_startup_grace_sec:.1f}'
        )
        self.start_px4_process()
        self.set_phase('wait_px4_start')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self) -> float:
        return self.now_sec() - self.phase_start_sec

    def set_phase(self, phase: str) -> None:
        self.phase = phase
        self.phase_start_sec = self.now_sec()
        self.get_logger().info(f'episode phase -> {phase}')

    def done_cb(self, msg: Bool) -> None:
        done = bool(msg.data)
        if done and not self.last_done and self.phase == 'idle':
            self.stop_offboard_process()
        self.last_done = done

    def mission_active_cb(self, msg: Bool) -> None:
        self.mission_active = bool(msg.data)

    def start_reset_pulse(self) -> None:
        self.reset_pub.publish(Bool(data=True))
        self.reset_release_sec = self.now_sec() + max(self.reset_pulse_sec, 0.0)
        self.get_logger().info('published /rl/reset=true')

    def update_reset_pulse(self) -> None:
        if self.reset_release_sec is None:
            return
        if self.now_sec() >= self.reset_release_sec:
            self.reset_pub.publish(Bool(data=False))
            self.reset_release_sec = None
            self.get_logger().info('published /rl/reset=false')

    def terminate_process(self, proc: subprocess.Popen | None, name: str) -> subprocess.Popen | None:
        if proc is None:
            return None
        if proc.poll() is None:
            self.get_logger().info(f'stopping {name} process')
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=self.stop_timeout_sec)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f'{name} exit timeout, killing process')
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=1.0)
        return None

    def _pkill_pattern(self, pattern: str, sig: signal.Signals) -> None:
        result = subprocess.run(
            ['pkill', f'-{sig.value}', '-f', pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if result.returncode == 0:
            self.get_logger().warn(f'orphan cleanup sent SIG{sig.value} to pattern: {pattern}')

    def cleanup_orphan_processes(self) -> None:
        px4_bin_pattern = '/home/fjw/PX4-Autopilot/build/px4_sitl_default/bin/px4'
        px4_build_pattern = 'cmake --build /home/fjw/PX4-Autopilot/build/px4_sitl_default -- gz_x500_depth'
        if self.px4_dir:
            px4_bin_pattern = os.path.join(self.px4_dir, 'build/px4_sitl_default/bin/px4')
            px4_build_pattern = f'cmake --build {os.path.join(self.px4_dir, "build/px4_sitl_default")} -- gz_x500_depth'
        patterns = [
            px4_bin_pattern,
            'gz sim --verbose=1 -r -s',
            'make px4_sitl_default gz_x500_depth',
            px4_build_pattern,
        ]
        self.get_logger().warn('running orphan cleanup for PX4/Gazebo leftovers')
        for pattern in patterns:
            self._pkill_pattern(pattern, signal.SIGTERM)
        time.sleep(max(self.orphan_cleanup_grace_sec, 0.0))
        for pattern in patterns:
            self._pkill_pattern(pattern, signal.SIGKILL)

    def start_px4_process(self) -> None:
        if self.px4_process is not None and self.px4_process.poll() is None:
            return
        if self.cleanup_orphans_before_px4_start:
            self.cleanup_orphan_processes()
        kwargs = {}
        if self.px4_dir:
            kwargs['cwd'] = self.px4_dir
        self.get_logger().info(f'starting px4 sitl process: {" ".join(self.px4_run_cmd)}')
        self.px4_process = subprocess.Popen(self.px4_run_cmd, start_new_session=True, **kwargs)

    def start_process(self, attr_name: str, cmd: list[str], name: str) -> None:
        proc = getattr(self, attr_name)
        if proc is not None and proc.poll() is None:
            return
        self.get_logger().info(f'starting {name} process: {" ".join(cmd)}')
        setattr(self, attr_name, subprocess.Popen(cmd, start_new_session=True))

    def stop_process(self, attr_name: str, next_step) -> None:
        proc = getattr(self, attr_name)
        if proc is None or proc.poll() is not None:
            setattr(self, attr_name, None)
            next_step()
            return
        os.killpg(proc.pid, signal.SIGTERM)
        next_step()

    def stop_px4_process(self) -> None:
        if self.px4_process is None or self.px4_process.poll() is not None:
            self.px4_process = None
            self.start_px4_process()
            self.set_phase('wait_px4_start')
            return
        os.killpg(self.px4_process.pid, signal.SIGTERM)
        self.set_phase('wait_px4_exit')

    def start_perception_processes(self) -> None:
        self.start_process('yolo_process', self.yolo_run_cmd, 'yolo')
        self.start_process('target_depth_process', self.target_depth_run_cmd, 'target_depth')
        self.start_process('target_lost_monitor_process', self.target_lost_monitor_run_cmd, 'target_lost_monitor')

    def stop_perception_processes(self) -> None:
        self.stop_process('yolo_process', self._after_yolo_stop)

    def _after_yolo_stop(self) -> None:
        self.set_phase('wait_yolo_exit')

    def _after_target_depth_stop(self) -> None:
        self.set_phase('wait_target_depth_exit')

    def _after_target_lost_monitor_stop(self) -> None:
        self.set_phase('wait_target_lost_monitor_exit')

    def start_offboard_process(self) -> None:
        if self.offboard_process is not None and self.offboard_process.poll() is None:
            return
        self.get_logger().info(f'starting offboard process: {" ".join(self.offboard_run_cmd)}')
        self.offboard_process = subprocess.Popen(self.offboard_run_cmd, start_new_session=True)

    def stop_offboard_process(self) -> None:
        if self.offboard_process is None or self.offboard_process.poll() is not None:
            self.offboard_process = None
            self.stop_perception_processes()
            return
        os.killpg(self.offboard_process.pid, signal.SIGTERM)
        self.set_phase('wait_offboard_exit')

    def cleanup_offboard_process(self) -> None:
        self.offboard_process = self.terminate_process(self.offboard_process, 'offboard')

    def cleanup_perception_processes(self) -> None:
        self.yolo_process = self.terminate_process(self.yolo_process, 'yolo')
        self.target_depth_process = self.terminate_process(self.target_depth_process, 'target_depth')
        self.target_lost_monitor_process = self.terminate_process(
            self.target_lost_monitor_process, 'target_lost_monitor'
        )

    def cleanup_px4_process(self) -> None:
        self.px4_process = self.terminate_process(self.px4_process, 'px4 sitl')

    def tick(self) -> None:
        self.update_reset_pulse()

        if self.phase == 'idle':
            return

        if self.phase == 'wait_offboard_exit':
            if self.offboard_process is None or self.offboard_process.poll() is not None:
                self.offboard_process = None
                self.stop_perception_processes()
                return
            if self.elapsed() >= self.stop_timeout_sec:
                os.killpg(self.offboard_process.pid, signal.SIGKILL)
            return

        if self.phase == 'wait_yolo_exit':
            if self.yolo_process is None or self.yolo_process.poll() is not None:
                self.yolo_process = None
                self.stop_process('target_depth_process', self._after_target_depth_stop)
                return
            if self.elapsed() >= self.stop_timeout_sec:
                os.killpg(self.yolo_process.pid, signal.SIGKILL)
            return

        if self.phase == 'wait_target_depth_exit':
            if self.target_depth_process is None or self.target_depth_process.poll() is not None:
                self.target_depth_process = None
                self.stop_process('target_lost_monitor_process', self._after_target_lost_monitor_stop)
                return
            if self.elapsed() >= self.stop_timeout_sec:
                os.killpg(self.target_depth_process.pid, signal.SIGKILL)
            return

        if self.phase == 'wait_target_lost_monitor_exit':
            if self.target_lost_monitor_process is None or self.target_lost_monitor_process.poll() is not None:
                self.target_lost_monitor_process = None
                self.stop_px4_process()
                return
            if self.elapsed() >= self.stop_timeout_sec:
                os.killpg(self.target_lost_monitor_process.pid, signal.SIGKILL)
            return

        if self.phase == 'wait_px4_exit':
            if self.px4_process is None or self.px4_process.poll() is not None:
                self.px4_process = None
                self.start_px4_process()
                self.set_phase('wait_px4_start')
                return
            if self.elapsed() >= self.stop_timeout_sec:
                os.killpg(self.px4_process.pid, signal.SIGKILL)
            return

        if self.phase == 'wait_px4_start':
            if self.elapsed() >= self.px4_startup_wait_sec:
                self.start_perception_processes()
                self.set_phase('wait_perception_start')
            return

        if self.phase == 'wait_perception_start':
            restart = False
            if self.yolo_process is not None and self.yolo_process.poll() is not None:
                self.get_logger().warn('yolo process exited during startup, restarting')
                self.yolo_process = None
                restart = True
            if self.target_depth_process is not None and self.target_depth_process.poll() is not None:
                self.get_logger().warn('target_depth process exited during startup, restarting')
                self.target_depth_process = None
                restart = True
            if self.target_lost_monitor_process is not None and self.target_lost_monitor_process.poll() is not None:
                self.get_logger().warn('target_lost_monitor process exited during startup, restarting')
                self.target_lost_monitor_process = None
                restart = True
            if restart:
                self.start_perception_processes()
                self.set_phase('wait_perception_start')
                return
            if (
                self.yolo_process is not None
                and self.target_depth_process is not None
                and self.target_lost_monitor_process is not None
                and self.elapsed() >= self.perception_startup_grace_sec
            ):
                self.start_offboard_process()
                self.set_phase('wait_offboard_start')
            return

        if self.phase == 'wait_offboard_start':
            if self.offboard_process is not None and self.offboard_process.poll() is not None:
                self.get_logger().warn('offboard process exited during startup, restarting')
                self.offboard_process = None
                self.start_offboard_process()
                self.set_phase('wait_offboard_start')
                return
            if self.offboard_process is not None and self.elapsed() >= self.offboard_startup_grace_sec:
                self.set_phase('wait_mission_active')
            return

        if self.phase == 'wait_mission_active':
            if self.offboard_process is None or self.offboard_process.poll() is not None:
                self.get_logger().warn('offboard process exited before mission became active, restarting episode')
                self.offboard_process = None
                self.stop_perception_processes()
                return
            if self.mission_active:
                self.start_reset_pulse()
                self.set_phase('cooldown')
                return
            if self.elapsed() >= self.mission_start_timeout_sec:
                self.get_logger().warn(
                    'mission did not become active within %.1fs, restarting episode' % self.mission_start_timeout_sec
                )
                self.stop_offboard_process()
                return
            return

        if self.phase == 'cooldown':
            if self.elapsed() >= self.cooldown_sec:
                self.set_phase('idle')
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EpisodeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cleanup_offboard_process()
    node.cleanup_perception_processes()
    node.cleanup_px4_process()
    if node.cleanup_orphans_on_shutdown:
        node.cleanup_orphan_processes()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
