import sys
import unittest
from unittest.mock import MagicMock, patch
import numpy as np

from omni_perception.detectors.base_detector import BaseDetector
from omni_perception.detectors import DETECTOR_REGISTRY
from omni_perception.detectors.yolo_detector import YoloDetector


class TestDetectorPipeline(unittest.TestCase):
    def test_ros_agnostic_shared_core(self):
        """Detectors module must NOT import rclpy or rospy."""
        self.assertNotIn('rclpy', sys.modules, "detectors module must not import rclpy")
        self.assertNotIn('rospy', sys.modules, "detectors module must not import rospy")

    def test_base_detector_cannot_be_instantiated(self):
        """BaseDetector is abstract and cannot be directly instantiated."""
        with self.assertRaises(TypeError):
            BaseDetector()

    def test_detector_registry_and_custom_detector(self):
        """Extensible registry allowing registration of new detectors."""
        class DummyDetector(BaseDetector):
            def __init__(self):
                self.is_loaded = False

            def load_model(self, model_key: str, device: str = "cpu") -> None:
                self.is_loaded = True
                self.model_key = model_key

            def detect(self, cv_image):
                return [
                    {
                        'bbox': [0.0, 0.0, 50.0, 50.0],
                        'class_name': "custom_object",
                        'confidence': 0.95,
                    }
                ]

            def get_model_info(self):
                return {'name': 'dummy', 'backend': 'custom'}

        DETECTOR_REGISTRY['dummy_test'] = DummyDetector

        self.assertIn('dummy_test', DETECTOR_REGISTRY)
        detector_cls = DETECTOR_REGISTRY['dummy_test']
        self.assertEqual(detector_cls, DummyDetector)

        instance = detector_cls()
        instance.load_model('dummy_model')
        self.assertTrue(instance.is_loaded)
        results = instance.detect(np.zeros((100, 100, 3), dtype=np.uint8))
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]['class_name'], "custom_object")
        self.assertEqual(results[0]['confidence'], 0.95)
        self.assertEqual(results[0]['bbox'], [0.0, 0.0, 50.0, 50.0])

    def test_yolo_detector_unloaded_raises(self):
        detector = YoloDetector()
        with self.assertRaises(RuntimeError):
            detector.detect(np.zeros((100, 100, 3), dtype=np.uint8))

    def test_yolo_detector_mocked_inference(self):
        """YoloDetector inference with mocked YOLO ultralytics."""
        mock_box1 = MagicMock()
        mock_box1.cls = MagicMock(item=lambda: 0)
        mock_box1.conf = MagicMock(item=lambda: 0.85)
        mock_box1.xyxy = [np.array([10.0, 20.0, 100.0, 200.0])]

        mock_result = MagicMock()
        mock_result.boxes = [mock_box1]
        mock_result.names = {0: "person", 2: "car"}

        with patch.dict('sys.modules', {'ultralytics': MagicMock()}):
            import ultralytics
            mock_model_instance = MagicMock()
            mock_model_instance.return_value = [mock_result]
            ultralytics.YOLO = MagicMock(return_value=mock_model_instance)

            detector = YoloDetector()
            detector.load_model('yolov8n', device='cpu')
            test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            detections = detector.detect(test_frame)

            self.assertEqual(len(detections), 1)
            self.assertEqual(detections[0]['class_name'], "person")
            self.assertAlmostEqual(detections[0]['confidence'], 0.85)
            self.assertEqual(detections[0]['bbox'], [10.0, 20.0, 100.0, 200.0])

            info = detector.get_model_info()
            self.assertEqual(info['name'], 'yolov8n')
            self.assertEqual(info['backend'], 'ultralytics')


if __name__ == '__main__':
    unittest.main()
