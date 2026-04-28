import rclpy
from rclpy.executors import ExternalShutdownException
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
        self.declare_parameter('camera_info_topic', '/realsense/rgbd/camera_info')
        self.declare_parameter('depth_topic', '/realsense/rgbd/depth_image_16uc1')
        self.declare_parameter('detection_topic', '/detector/boxes')
        self.declare_parameter('center_roi_scale', 0.35)
        self.declare_parameter('min_center_roi_size', 5)
        self.declare_parameter('foreground_percentile', 25.0)
        self.declare_parameter('depth_tolerance_mm', 250.0)

        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        detection_topic = str(self.get_parameter('detection_topic').value)
        # 中心 ROI 相对检测框宽高的缩放比例。
        # 例如 0.35 表示只使用检测框中心 35% 的区域做深度估计。
        self.center_roi_scale = float(self.get_parameter('center_roi_scale').value)
        # 中心 ROI 的最小边长，防止检测框很小时 ROI 小到没有统计意义。
        self.min_center_roi_size = int(self.get_parameter('min_center_roi_size').value)
        # 用于估计“前景深度种子”的分位数。
        # 值越小，越偏向选择更近的像素；值越大，越容易把背景也算进去。
        self.foreground_percentile = float(self.get_parameter('foreground_percentile').value)
        # 前景像素筛选的深度容差，单位是毫米。
        # 只有深度小于“前景种子 + 容差”的像素才会被视为目标候选。
        self.depth_tolerance_mm = float(self.get_parameter('depth_tolerance_mm').value)

        # 相机内参
        self.sub_info = self.create_subscription(
            CameraInfo, camera_info_topic, self.info_callback, 10)

        # 深度图与检测框近似时间同步，减少错配噪声。
        self.sub_depth = Subscriber(self, Image, depth_topic)
        self.sub_det = Subscriber(self, Detection2DArray, detection_topic)
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

    def estimate_target_pixel_and_depth(self, depth_image, det):
        h, w = depth_image.shape
        bbox_cx = float(det.bbox.center.position.x)
        bbox_cy = float(det.bbox.center.position.y)
        bbox_w = max(1, int(round(det.bbox.size_x)))
        bbox_h = max(1, int(round(det.bbox.size_y)))

        # 不直接用整个检测框，而是只取框中心的一小块区域。
        # 对无人机这类小目标，检测框里常常大部分都是背景，
        # 缩小 ROI 可以减少背景深度对统计结果的干扰。
        roi_w = max(self.min_center_roi_size, int(round(bbox_w * self.center_roi_scale)))
        roi_h = max(self.min_center_roi_size, int(round(bbox_h * self.center_roi_scale)))

        cx = int(round(bbox_cx))
        cy = int(round(bbox_cy))
        x0 = max(0, cx - roi_w // 2)
        y0 = max(0, cy - roi_h // 2)
        x1 = min(w, x0 + roi_w)
        y1 = min(h, y0 + roi_h)
        roi = depth_image[y0:y1, x0:x1].astype(np.float32)

        valid_mask = np.isfinite(roi) & (roi > 0.0)
        if not np.any(valid_mask):
            return None

        valid_depths = roi[valid_mask]
        # 用较小分位数作为“前景深度种子”。
        # 直觉上，如果目标在背景前方，那么 ROI 里更小的深度值
        # 更可能来自目标本身，而不是远处背景。
        seed_depth = np.percentile(valid_depths, self.foreground_percentile)

        # 只保留“接近前景深度种子”的像素，认为它们更可能属于目标。
        # depth_tolerance_mm 控制保留范围，值越小越严格，越大越宽松。
        fg_mask = valid_mask & (roi <= seed_depth + self.depth_tolerance_mm)
        if not np.any(fg_mask):
            # 如果筛完一个像素都没有，退化为使用 ROI 内所有有效深度，
            # 避免因为参数过严导致整帧没有输出。
            fg_mask = valid_mask

        ys, xs = np.nonzero(fg_mask)
        fg_depths = roi[fg_mask]

        # 用前景像素的质心替代检测框中心。
        # 这样可以避免“框中心落在背景上”时，X/Y 方向也一起偏掉。
        px = int(round(x0 + float(np.mean(xs))))
        py = int(round(y0 + float(np.mean(ys))))
        # Z 仍然使用前景像素的中值，目的是抑制少量离群点。
        z_m = float(np.median(fg_depths)) / 1000.0
        return px, py, z_m

    def synced_callback(self, depth_msg, det_msg):
        if self.fx is None:
            self.get_logger().warn("camera intrinsics not ready")
            return

        if len(det_msg.detections) == 0:
            return

        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # 选取置信度最高的目标框。
        best = max(det_msg.detections, key=lambda d: d.results[0].hypothesis.score)
        # 在检测框中心区域内，估计更可能属于目标的像素位置和深度。
        estimate = self.estimate_target_pixel_and_depth(depth_image, best)
        if estimate is None:
            return
        px, py, Z = estimate

        # 将“前景像素点 + 深度”反投影到相机坐标系。
        X = (px - self.cx) * Z / self.fx
        Y = (py - self.cy) * Z / self.fy

        pt = PointStamped()
        pt.header = det_msg.header
        if not pt.header.frame_id:
            pt.header.frame_id = "camera_optical_frame"
        pt.point.x = X
        pt.point.y = Y
        pt.point.z = Z
        self.pub_target.publish(pt)

        # self.get_logger().info(
        #     f"Target XYZ: ({X:.2f}, {Y:.2f}, {Z:.2f}), pixel=({px}, {py})"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = TargetDepthNode()
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
