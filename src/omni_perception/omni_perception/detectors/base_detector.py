"""
Abstract base detector for AMR Omni perception pipeline.
ROS-agnostic shared core module.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, List


class BaseDetector(ABC):
    """
    Abstract interface for object detectors.
    Pure Python, ROS-agnostic.
    """

    @abstractmethod
    def load_model(self, model_key: str, device: str = "cpu") -> None:
        """
        Load model weights and configure inference device.
        Called once during detector initialization.
        """
        pass

    @abstractmethod
    def detect(self, cv_image: Any) -> List[Dict[str, Any]]:
        """
        Perform detection on a BGR numpy image.

        Returns:
            list of dict: [
                {
                    'bbox': [x1, y1, x2, y2],
                    'class_name': str,
                    'confidence': float,
                },
                ...
            ]
        """
        pass

    @abstractmethod
    def get_model_info(self) -> Dict[str, Any]:
        """
        Return metadata dictionary about current detector.
        Example: {'name': 'yolov8n', 'backend': 'ultralytics', 'input_size': 640, 'classes': 80}
        """
        pass
