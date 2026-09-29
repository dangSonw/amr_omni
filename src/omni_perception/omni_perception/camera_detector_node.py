"""
Camera detector node for AMR Omni.
Subscribes to camera image, runs pluggable detector, and publishes vision_msgs Detection2DArray.
"""

import time
from typing import Optional
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .detectors import DETECTOR_REGISTRY


class CameraDetectorNode(Node):
    """ROS 2 Node for camera object detection with multi-model backend."""

    def __init__(self) -> None:
        super().__init__('camera_detector_node')

        self.declare_parameter('model', 'yolov8n')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('max_fps', 15.0)

        model_key = str(self.get_parameter('model').value)
        self._confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self._device = str(self.get_parameter('device').value)
        self._publish_debug = bool(self.get_parameter('publish_debug_image').value)
        self._max_fps = float(self.get_parameter('max_fps').value)
        self._min_interval = 1.0 / self._max_fps if self._max_fps > 0 else 0.0

        if model_key not in DETECTOR_REGISTRY:
            avail = ', '.join(sorted(DETECTOR_REGISTRY.keys()))
            raise ValueError(f"Model '{model_key}' not found. Available models: {avail}")

        detector_cls = DETECTOR_REGISTRY[model_key]
        self._detector = detector_cls()
        self._detector.load_model(model_key, self._device)
        self.get_logger().info(f"Loaded detector: {self._detector.get_model_info()}")

        self._bridge = CvBridge()
        self._last_process_time = 0.0

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        default_qos = QoSPresetProfiles.SYSTEM_DEFAULT.value

        self._sub_image = self.create_subscription(
            Image, '/camera/image', self._on_image, sensor_qos
        )
        self._pub_detections = self.create_publisher(
            Detection2DArray, '/perception/detections', default_qos
        )
        self._pub_debug = self.create_publisher(
            Image, '/perception/debug_image', sensor_qos
        )

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_process_time < self._min_interval:
            return
        self._last_process_time = now

        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f"Failed to convert ROS Image to cv2: {exc}")
            return

        raw_detections = self._detector.detect(cv_img)

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        debug_img = cv_img.copy() if self._publish_debug else None

        for det in raw_detections:
            conf = det['confidence']
            if conf < self._confidence_threshold:
                continue

            x1, y1, x2, y2 = det['bbox']
            w = float(x2 - x1)
            h = float(y2 - y1)
            cx = float(x1 + w / 2.0)
            cy = float(y1 + h / 2.0)
            class_name = det['class_name']

            d2d = Detection2D()
            d2d.header = msg.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_name
            hyp.hypothesis.score = conf
            d2d.results.append(hyp)

            d2d.bbox.center.position.x = cx
            d2d.bbox.center.position.y = cy
            d2d.bbox.size_x = w
            d2d.bbox.size_y = h
            detection_array.detections.append(d2d)

            if debug_img is not None:
                pt1 = (int(x1), int(y1))
                pt2 = (int(x2), int(y2))
                cv2.rectangle(debug_img, pt1, pt2, (0, 255, 0), 2)
                label = f"{class_name} {conf:.2f}"
                cv2.putText(
                    debug_img, label, (int(x1), max(int(y1) - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA
                )

        self._pub_detections.publish(detection_array)

        if debug_img is not None:
            try:
                debug_msg = self._bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                debug_msg.header = msg.header
                self._pub_debug.publish(debug_msg)
            except Exception as exc:
                self.get_logger().warning(f"Failed to publish debug image: {exc}")


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = CameraDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
