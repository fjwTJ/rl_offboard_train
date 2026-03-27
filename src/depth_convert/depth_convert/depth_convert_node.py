import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthConvert(Node):
    def __init__(self):
        super().__init__('depth_convert')

        self.declare_parameter('input_topic', '/realsense/rgbd/depth_image')
        self.declare_parameter('output_topic', '/realsense/rgbd/depth_image_16uc1')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(Image, input_topic, self.cb, 10)
        self.pub = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(
            f'Converting depth image topic {input_topic} -> {output_topic}'
        )

    def cb(self, msg: Image):
        if msg.encoding and msg.encoding != '32FC1':
            self.get_logger().warn(
                f'Expected 32FC1 depth image, got {msg.encoding}. Skipping message.'
            )
            return

        depth_array = np.frombuffer(msg.data, dtype=np.float32)
        expected_size = msg.height * msg.width
        if depth_array.size != expected_size:
            self.get_logger().warn(
                f'Unexpected image buffer size {depth_array.size}, expected {expected_size}.'
            )
            return

        depth_array = depth_array.reshape(msg.height, msg.width)
        depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=0.0, neginf=0.0)
        depth_array = np.clip(depth_array, 0.0, np.iinfo(np.uint16).max / 1000.0)
        depth_mm = (depth_array * 1000.0).astype(np.uint16)

        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = '16UC1'
        out.is_bigendian = msg.is_bigendian
        out.step = msg.width * 2
        out.data = depth_mm.tobytes()

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthConvert()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
