import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # 加载模型
        self.model = YOLO("/home/fjw/rl_offboard/src/yolo_detector/best.pt")
        #self.get_logger().info(f"YOLO model loaded: {model_path}")

        self.bridge = CvBridge()

        # 订阅相机图像（注意话题名称）
        self.sub = self.create_subscription(
            Image,
            '/realsense/rgbd/image',  
            self.image_callback,
            10)

        # 发布检测结果
        self.pub = self.create_publisher(Detection2DArray, '/detector/boxes', 10)

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
        self.get_logger().info(f"YOLO FPS: {fps:.2f}")

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
        annotated = results[0].plot()
        cv2.imshow("YOLO Detection", annotated)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

