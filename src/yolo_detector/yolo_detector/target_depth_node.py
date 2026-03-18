import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer


class TargetDepthNode(Node):
    def __init__(self):
        super().__init__('target_depth_node')
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None

        # 相机内参
        self.sub_info = self.create_subscription(
            CameraInfo, '/realsense/rgbd/camera_info', self.info_callback, 10)

        # 深度图与检测框近似时间同步，减少错配噪声。
        self.sub_depth = Subscriber(self, Image, '/realsense/rgbd/depth_image_16uc1')
        self.sub_det = Subscriber(self, Detection2DArray, '/detector/boxes')
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_depth, self.sub_det],
            queue_size=20,
            slop=0.06,
        )
        self.sync.registerCallback(self.synced_callback)

        # 目标三维坐标（相机坐标系）
        self.pub_target = self.create_publisher(PointStamped, '/perception/target_xyz', 10)

    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def synced_callback(self, depth_msg, det_msg):
        if self.fx is None:
            self.get_logger().warn("camera intrinsics not ready")
            return

        if len(det_msg.detections) == 0:
            return

        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # 选取置信度最高的目标框。
        best = max(det_msg.detections, key=lambda d: d.results[0].hypothesis.score)
        cx = int(best.bbox.center.position.x)
        cy = int(best.bbox.center.position.y)

        # 取中心 3x3 区域中值深度，抑制离群值。
        h, w = depth_image.shape
        x0, y0 = max(0, cx - 1), max(0, cy - 1)
        x1, y1 = min(w - 1, cx + 1), min(h - 1, cy + 1)
        roi = depth_image[y0:y1 + 1, x0:x1 + 1].astype(np.float32)
        roi = roi[np.isfinite(roi)]
        if roi.size == 0:
            return
        Z = np.median(roi) / 1000.0

        # 像素坐标转相机坐标。
        X = (cx - self.cx) * Z / self.fx
        Y = (cy - self.cy) * Z / self.fy

        pt = PointStamped()
        pt.header = det_msg.header
        if not pt.header.frame_id:
            pt.header.frame_id = "camera_optical_frame"
        pt.point.x = X
        pt.point.y = Y
        pt.point.z = Z
        self.pub_target.publish(pt)

        #self.get_logger().info(f"Target XYZ: ({X:.2f}, {Y:.2f}, {Z:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = TargetDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
