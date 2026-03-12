import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from std_msgs.msg import Bool, Empty, Int8


class EpisodeManagerNode(Node):
    """Strict reset flow: done -> abort -> wait complete -> world reset -> set_pose -> hard_reset -> wait init."""

    OFFBOARD_STATE_INIT = 0
    OFFBOARD_STATE_COMPLETE = 12

    def __init__(self) -> None:
        super().__init__('episode_manager_node')

        self.declare_parameter('done_topic', '/rl/done')
        self.declare_parameter('mission_topic', '/mission_control')
        self.declare_parameter('offboard_state_topic', '/offboard/state')
        self.declare_parameter('hard_reset_topic', '/offboard/hard_reset')
        self.declare_parameter('world_name', 'default')

        self.declare_parameter('enable_world_reset', True)
        self.declare_parameter('world_reset_mode', 'model_only')
        self.declare_parameter('enable_set_pose', True)
        self.declare_parameter('model_name', 'x500_depth_0')
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_z', 0.0)
        self.declare_parameter('init_qx', 0.0)
        self.declare_parameter('init_qy', 0.0)
        self.declare_parameter('init_qz', 0.0)
        self.declare_parameter('init_qw', 1.0)

        self.declare_parameter('wait_complete_timeout_sec', 30.0)
        self.declare_parameter('reset_timeout_sec', 2.0)
        self.declare_parameter('set_pose_timeout_sec', 2.0)
        self.declare_parameter('wait_init_timeout_sec', 12.0)
        self.declare_parameter('cooldown_sec', 0.5)
        self.declare_parameter('tick_hz', 20.0)

        done_topic = str(self.get_parameter('done_topic').value)
        mission_topic = str(self.get_parameter('mission_topic').value)
        offboard_state_topic = str(self.get_parameter('offboard_state_topic').value)
        hard_reset_topic = str(self.get_parameter('hard_reset_topic').value)
        world_name = str(self.get_parameter('world_name').value)

        self.enable_world_reset = bool(self.get_parameter('enable_world_reset').value)
        self.world_reset_mode = str(self.get_parameter('world_reset_mode').value)
        self.enable_set_pose = bool(self.get_parameter('enable_set_pose').value)
        self.model_name = str(self.get_parameter('model_name').value)

        self.wait_complete_timeout_sec = float(self.get_parameter('wait_complete_timeout_sec').value)
        self.reset_timeout_sec = float(self.get_parameter('reset_timeout_sec').value)
        self.set_pose_timeout_sec = float(self.get_parameter('set_pose_timeout_sec').value)
        self.wait_init_timeout_sec = float(self.get_parameter('wait_init_timeout_sec').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)

        self.init_x = float(self.get_parameter('init_x').value)
        self.init_y = float(self.get_parameter('init_y').value)
        self.init_z = float(self.get_parameter('init_z').value)
        self.init_qx = float(self.get_parameter('init_qx').value)
        self.init_qy = float(self.get_parameter('init_qy').value)
        self.init_qz = float(self.get_parameter('init_qz').value)
        self.init_qw = float(self.get_parameter('init_qw').value)

        self.done_sub = self.create_subscription(Bool, done_topic, self.done_cb, 10)
        self.state_sub = self.create_subscription(Int8, offboard_state_topic, self.state_cb, 10)
        self.mission_pub = self.create_publisher(Int8, mission_topic, 10)
        self.hard_reset_pub = self.create_publisher(Empty, hard_reset_topic, 10)

        control_srv = f'/world/{world_name}/control'
        set_pose_srv = f'/world/{world_name}/set_pose'
        self.world_client = self.create_client(ControlWorld, control_srv)
        self.set_pose_client = self.create_client(SetEntityPose, set_pose_srv)
        self.world_future = None
        self.pose_future = None

        self.last_done = False
        self.offboard_state = self.OFFBOARD_STATE_INIT
        self.phase = 'idle'
        self.phase_start_sec = self.now_sec()
        self.hard_reset_sent = False

        period = 1.0 / max(float(self.get_parameter('tick_hz').value), 1.0)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(
            'episode_manager_node(state-driven) started. '
            f'world_reset={self.enable_world_reset} mode={self.world_reset_mode} set_pose={self.enable_set_pose}'
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self) -> float:
        return self.now_sec() - self.phase_start_sec

    def set_phase(self, phase: str) -> None:
        self.phase = phase
        self.phase_start_sec = self.now_sec()
        if phase == 'hard_reset':
            self.hard_reset_sent = False
        self.get_logger().info(f'episode phase -> {phase}')

    def done_cb(self, msg: Bool) -> None:
        done = bool(msg.data)
        if done and not self.last_done and self.phase == 'idle':
            self.mission_pub.publish(Int8(data=3))
            self.set_phase('wait_complete')
        self.last_done = done

    def state_cb(self, msg: Int8) -> None:
        self.offboard_state = int(msg.data)

    def request_world_reset(self) -> None:
        if not self.enable_world_reset:
            self.set_phase('set_pose')
            return
        if not self.world_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('World control service unavailable, skip world reset')
            self.set_phase('set_pose')
            return
        req = ControlWorld.Request()
        mode = self.world_reset_mode.lower().strip()
        if mode == 'all':
            req.world_control.reset.all = True
        elif mode == 'time_only':
            req.world_control.reset.time_only = True
        else:
            req.world_control.reset.model_only = True
        self.world_future = self.world_client.call_async(req)
        self.set_phase('wait_world_reset')

    def request_set_pose(self) -> None:
        if not self.enable_set_pose:
            self.set_phase('hard_reset')
            return
        if not self.set_pose_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('Set pose service unavailable, skip set pose')
            self.set_phase('hard_reset')
            return
        req = SetEntityPose.Request()
        req.entity.name = self.model_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = self.init_x
        req.pose.position.y = self.init_y
        req.pose.position.z = self.init_z
        req.pose.orientation.x = self.init_qx
        req.pose.orientation.y = self.init_qy
        req.pose.orientation.z = self.init_qz
        req.pose.orientation.w = self.init_qw
        self.pose_future = self.set_pose_client.call_async(req)
        self.set_phase('wait_set_pose')

    def tick(self) -> None:
        if self.phase == 'idle':
            return

        if self.phase == 'wait_complete':
            if self.offboard_state == self.OFFBOARD_STATE_COMPLETE:
                self.request_world_reset()
                return
            if self.elapsed() >= self.wait_complete_timeout_sec:
                self.get_logger().warn('wait complete timeout, resend abort and keep waiting')
                self.mission_pub.publish(Int8(data=3))
                self.phase_start_sec = self.now_sec()
            return

        if self.phase == 'wait_world_reset':
            if self.world_future is not None and self.world_future.done():
                self.request_set_pose()
                return
            if self.elapsed() >= self.reset_timeout_sec:
                self.get_logger().warn('world reset timeout, continue')
                self.request_set_pose()
            return

        if self.phase == 'set_pose':
            self.request_set_pose()
            return

        if self.phase == 'wait_set_pose':
            if self.pose_future is not None and self.pose_future.done():
                self.set_phase('hard_reset')
                return
            if self.elapsed() >= self.set_pose_timeout_sec:
                self.get_logger().warn('set pose timeout, continue')
                self.set_phase('hard_reset')
            return

        if self.phase == 'hard_reset':
            if not self.hard_reset_sent:
                self.hard_reset_pub.publish(Empty())
                self.hard_reset_sent = True
                self.set_phase('wait_init')
            return

        if self.phase == 'wait_init':
            if self.offboard_state == self.OFFBOARD_STATE_INIT:
                self.set_phase('cooldown')
                return
            if self.elapsed() >= self.wait_init_timeout_sec:
                self.get_logger().warn('wait init timeout, force cooldown')
                self.set_phase('cooldown')
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
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
