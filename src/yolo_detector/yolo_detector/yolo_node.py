import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        default_model_path = str(Path(__file__).resolve().parents[1] / 'best.pt')
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('image_topic', '/realsense/rgbd/image')
        self.declare_parameter('enable_visualization', False)
        self.model_path = str(self.get_parameter('model_path').value)
        image_topic = str(self.get_parameter('image_topic').value)
        self.enable_visualization = bool(self.get_parameter('enable_visualization').value)
        self.window_name = 'YOLO Detection'
        self.window_created = False
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # 加载模型
        self.model = YOLO(self.model_path)

        self.bridge = CvBridge()

        # 订阅相机图像（注意话题名称）
        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)

        # 发布检测结果
        self.pub = self.create_publisher(Detection2DArray, '/detector/boxes', 10)

    def _on_set_parameters(self, params):
        for param in params:
            if param.name == 'enable_visualization':
                self.enable_visualization = bool(param.value)
                if not self.enable_visualization:
                    self._destroy_window()
        return SetParametersResult(successful=True)

    def _destroy_window(self):
        if self.window_created:
            cv2.destroyWindow(self.window_name)
            self.window_created = False

    def image_callback(self, msg):
        # 记录开始时间
        t0 = time.time()

        # 转成 OpenCV 图像
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # YOLO 推理
        #results = self.model.predict(img, verbose=False, classes=[2])
        results = self.model.predict(img, verbose=False)

        # ========= FPS =========
        dt = time.time() - t0
        fps = 1.0 / dt if dt > 0 else 0.0
        #self.get_logger().info(f"YOLO FPS: {fps:.2f}")

        # ========= 发布检测结果 =========
        detections = Detection2DArray()
        detections.header = msg.header

        for box in results[0].boxes:
            det = Detection2D()

            cx, cy, w, h = box.xywh[0].tolist()
            score = float(box.conf[0])
            cls_id = int(box.cls[0])

            det.bbox.center.position.x = float(cx)
            det.bbox.center.position.y = float(cy)
            det.bbox.center.theta = 0.0

            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = str(cls_id)
            hypo.hypothesis.score = score
            det.results.append(hypo)

            detections.detections.append(det)

        self.pub.publish(detections)

        # ========= 可视化窗口 =========
        if self.enable_visualization:
            annotated = results[0].plot()
            if not self.window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                self.window_created = True
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)
        else:
            self._destroy_window()


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._destroy_window()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
