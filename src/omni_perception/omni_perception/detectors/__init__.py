"""
Detector module registry for AMR Omni perception.
"""

from typing import Dict, Type
from .base_detector import BaseDetector
from .yolo_detector import YoloDetector

DETECTOR_REGISTRY: Dict[str, Type[BaseDetector]] = {
    'yolov8n': YoloDetector,
    'yolov11n': YoloDetector,
    'yolov8n-seg': YoloDetector,
}

__all__ = [
    'BaseDetector',
    'YoloDetector',
    'DETECTOR_REGISTRY',
]
