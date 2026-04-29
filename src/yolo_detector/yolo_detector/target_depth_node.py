import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer


def detection_score(det):
    if not det.results:
        return 0.0
    return float(det.results[0].hypothesis.score)


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
        self.declare_parameter('depth_value_type', 'ray_range')
        self.declare_parameter('min_depth_m', 0.1)
        self.declare_parameter('max_depth_m', 30.0)
        self.declare_parameter('min_foreground_pixels', 4)
        self.declare_parameter('max_depth_jump_m', 1.5)
        self.declare_parameter('depth_filter_alpha', 0.45)

        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        detection_topic = str(self.get_parameter('detection_topic').value)
        # 中心 ROI 相对检测框宽高的缩放比例。
        # 例如 0.35 表示只使用检测框中心 35% 的区域做深度估计。
        self.center_roi_scale = float(self.get_parameter('center_roi_scale').value)
        # 中心 ROI 的最小边长，防止检测框很小时 ROI 小到没有统计意义。
        self.min_center_roi_size = int(self.get_parameter('min_center_roi_size').value)
        # 用于估计“前景深度种子”的分位数。
        # 值越小，越偏向选择更近的像素。
        # 值越大，越容易把背景也算进去。
        self.foreground_percentile = float(self.get_parameter('foreground_percentile').value)
        # 前景像素筛选的深度容差，单位是毫米。
        # 只有深度小于“前景种子 + 容差”的像素才会被视为目标候选。
        self.depth_tolerance_mm = float(self.get_parameter('depth_tolerance_mm').value)
        self.depth_value_type = str(self.get_parameter('depth_value_type').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.min_foreground_pixels = int(self.get_parameter('min_foreground_pixels').value)
        self.max_depth_jump_m = float(self.get_parameter('max_depth_jump_m').value)
        self.depth_filter_alpha = float(self.get_parameter('depth_filter_alpha').value)
        self._filtered_estimate = None

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

    def depth_image_to_meters(self, depth_image):
        if depth_image.dtype == np.uint16:
            depth_m = depth_image.astype(np.float32) / 1000.0
        else:
            depth_m = depth_image.astype(np.float32)

        depth_m = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
        if self.depth_value_type == 'z_depth':
            return depth_m
        if self.depth_value_type != 'ray_range':
            self.get_logger().warn(
                f"Unknown depth_value_type={self.depth_value_type}, using ray_range")

        h, w = depth_m.shape
        xs = (np.arange(w, dtype=np.float32) - float(self.cx)) / float(self.fx)
        ys = (np.arange(h, dtype=np.float32) - float(self.cy)) / float(self.fy)
        scale = np.sqrt(1.0 + xs[np.newaxis, :] ** 2 + ys[:, np.newaxis] ** 2)
        return depth_m / scale

    def estimate_target_pixel_and_depth(self, depth_z_m, det):
        h, w = depth_z_m.shape
        bbox_cx = float(det.bbox.center.position.x)
        bbox_cy = float(det.bbox.center.position.y)
        bbox_w = max(1, int(round(det.bbox.size_x)))
        bbox_h = max(1, int(round(det.bbox.size_y)))

        x0 = max(0, int(np.floor(bbox_cx - bbox_w * 0.5)))
        y0 = max(0, int(np.floor(bbox_cy - bbox_h * 0.5)))
        x1 = min(w, int(np.ceil(bbox_cx + bbox_w * 0.5)))
        y1 = min(h, int(np.ceil(bbox_cy + bbox_h * 0.5)))
        if x1 <= x0 or y1 <= y0:
            return None

        roi = depth_z_m[y0:y1, x0:x1].astype(np.float32)

        valid_mask = (
            np.isfinite(roi)
            & (roi >= self.min_depth_m)
            & (roi <= self.max_depth_m)
        )
        if not np.any(valid_mask):
            return None

        valid_depths = roi[valid_mask]
        tolerance_m = self.depth_tolerance_mm / 1000.0
        nearest_depth = float(np.min(valid_depths))
        percentile_depth = float(np.percentile(valid_depths, self.foreground_percentile))
        seed_depth = min(percentile_depth, nearest_depth + tolerance_m)
        fg_mask = valid_mask & (roi <= seed_depth + tolerance_m)
        if int(np.count_nonzero(fg_mask)) < self.min_foreground_pixels:
            return None

        component_mask = self.select_foreground_component(fg_mask, bbox_cx - x0, bbox_cy - y0)
        if component_mask is None:
            return None

        ys, xs = np.nonzero(component_mask)
        fg_depths = roi[component_mask]
        if fg_depths.size < self.min_foreground_pixels:
            return None

        px = int(round(x0 + float(np.mean(xs))))
        py = int(round(y0 + float(np.mean(ys))))
        z_m = float(np.median(fg_depths))
        return px, py, z_m

    def select_foreground_component(self, fg_mask, center_x, center_y):
        visited = np.zeros(fg_mask.shape, dtype=bool)
        best_pixels = None
        best_score = None
        height, width = fg_mask.shape

        for start_y, start_x in np.argwhere(fg_mask):
            if visited[start_y, start_x]:
                continue
            stack = [(int(start_y), int(start_x))]
            visited[start_y, start_x] = True
            pixels = []

            while stack:
                y, x = stack.pop()
                pixels.append((y, x))
                for ny in range(max(0, y - 1), min(height, y + 2)):
                    for nx in range(max(0, x - 1), min(width, x + 2)):
                        if visited[ny, nx] or not fg_mask[ny, nx]:
                            continue
                        visited[ny, nx] = True
                        stack.append((ny, nx))

            count = len(pixels)
            if count < self.min_foreground_pixels:
                continue
            arr = np.asarray(pixels, dtype=np.float32)
            cy = float(np.mean(arr[:, 0]))
            cx = float(np.mean(arr[:, 1]))
            distance = np.hypot(cx - center_x, cy - center_y)
            score = distance - 0.25 * np.sqrt(float(count))
            if best_score is None or score < best_score:
                best_score = score
                best_pixels = pixels

        if best_pixels is None:
            return None

        component = np.zeros(fg_mask.shape, dtype=bool)
        for y, x in best_pixels:
            component[y, x] = True
        return component

    def apply_temporal_filter(self, estimate):
        if estimate is None:
            return None
        if self._filtered_estimate is None:
            self._filtered_estimate = tuple(float(v) for v in estimate)
            return estimate

        last_px, last_py, last_z = self._filtered_estimate
        px, py, z = (float(v) for v in estimate)
        if z > last_z + self.max_depth_jump_m:
            return None

        alpha = min(max(self.depth_filter_alpha, 0.0), 1.0)
        filtered = (
            alpha * px + (1.0 - alpha) * last_px,
            alpha * py + (1.0 - alpha) * last_py,
            alpha * z + (1.0 - alpha) * last_z,
        )
        self._filtered_estimate = filtered
        return int(round(filtered[0])), int(round(filtered[1])), float(filtered[2])

    def synced_callback(self, depth_msg, det_msg):
        if self.fx is None:
            self.get_logger().warn("camera intrinsics not ready")
            return

        if len(det_msg.detections) == 0:
            return

        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth_z_m = self.depth_image_to_meters(depth_image)

        # 选取置信度最高的目标框。
        best = max(det_msg.detections, key=detection_score)
        # 在检测框区域内提取前景深度簇，估计目标像素位置和深度。
        estimate = self.estimate_target_pixel_and_depth(depth_z_m, best)
        estimate = self.apply_temporal_filter(estimate)
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
