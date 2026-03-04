import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
import cv2

class TargetDepthNode(Node):
    def __init__(self):
        super().__init__('target_depth_node')
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None

        # 订阅深度图
        self.sub_depth = self.create_subscription(
            Image, '/realsense/rgbd/depth_image_16uc1', self.depth_callback, 10)
        # 订阅相机内参
        self.sub_info = self.create_subscription(
            CameraInfo, '/realsense/rgbd/camera_info', self.info_callback, 10)
        # 订阅YOLO检测结果
        self.sub_det = self.create_subscription(
            Detection2DArray, '/detector/boxes', self.det_callback, 10)

        # 发布3D目标位置
        self.pub_target = self.create_publisher(PointStamped, '/perception/target_xyz', 10)
        self.depth_image = None

    def info_callback(self, msg):
        # 获取相机内参
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def det_callback(self, msg):
        if self.depth_image is None:
            self.get_logger().warn("depth_image is None")
            return

        if self.fx is None:
            self.get_logger().warn("camera intrinsics not ready")
            return

        if len(msg.detections) == 0:
            return

        # 选置信度最高的框
        best = max(msg.detections, key=lambda d: d.results[0].hypothesis.score)
        cx = int(best.bbox.center.position.x)
        cy = int(best.bbox.center.position.y)

        # 取中心3x3区域平均深度
        h, w = self.depth_image.shape
        x0, y0 = max(0, cx-1), max(0, cy-1)
        x1, y1 = min(w-1, cx+1), min(h-1, cy+1)
        roi = self.depth_image[y0:y1+1, x0:x1+1].astype(np.float32)
        roi = roi[np.isfinite(roi)]
        if roi.size == 0:
            return
        Z = np.median(roi) / 1000.0  # mm -> m

        # 像素坐标转相机坐标
        X = (cx - self.cx) * Z / self.fx
        Y = (cy - self.cy) * Z / self.fy

        # 发布
        pt = PointStamped()
        pt.header = msg.header
        if not pt.header.frame_id:
            pt.header.frame_id = "camera_optical_frame"
        pt.point.x = X
        pt.point.y = Y
        pt.point.z = Z
        self.pub_target.publish(pt)

        self.get_logger().info(f"Target XYZ: ({X:.2f}, {Y:.2f}, {Z:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = TargetDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
