import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


class CameraTfPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_tf_publisher_node')
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.declare_parameter('use_sim_tf', True)
        self.use_sim_tf = self.get_parameter('use_sim_tf').get_parameter_value().bool_value

        self.declare_parameter('parent_frame', 'base_link_frd')
        self.parent_frame = str(self.get_parameter('parent_frame').value)

        # 默认子坐标系使用当前仿真相机链路。
        self.declare_parameter(
            'sim_child_frame',
            'x500_depth_0/RealSenseD455/base_link/RealSenseD455/rgbd',
        )
        self.declare_parameter('real_child_frame', 'camera_link')
        self.sim_child_frame = str(self.get_parameter('sim_child_frame').value)
        self.real_child_frame = str(self.get_parameter('real_child_frame').value)

        self.publish_static_tf()
        self.get_logger().info(
            f'Published static TF: {self.parent_frame} -> '
            f'{self.sim_child_frame if self.use_sim_tf else self.real_child_frame}'
        )

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame

        if self.use_sim_tf:
            # 仿真环境: base_link_frd -> 相机等效光学坐标系。
            t.child_frame_id = self.sim_child_frame
            t.transform.translation.x = 0.12
            t.transform.translation.y = -0.03
            t.transform.translation.z = -0.242
            t.transform.rotation.x = 0.5
            t.transform.rotation.y = 0.5
            t.transform.rotation.z = 0.5
            t.transform.rotation.w = 0.5
        else:
            # 实机环境: base_link_frd -> camera_link。
            t.child_frame_id = self.real_child_frame
            t.transform.translation.x = 0.15
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.10
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CameraTfPublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
