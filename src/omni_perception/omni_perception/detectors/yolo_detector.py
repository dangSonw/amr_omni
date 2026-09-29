"""
YOLO detector implementation wrapping Ultralytics.
ROS-agnostic shared core module.
"""

from typing import Any, Dict, List
from .base_detector import BaseDetector


class YoloDetector(BaseDetector):
    """
    Detector implementation using Ultralytics YOLO models (v8, v11, seg).
    """

    MODEL_MAP: Dict[str, str] = {
        'yolov8n': 'yolov8n.pt',
        'yolov11n': 'yolo11n.pt',
        'yolov8n-seg': 'yolov8n-seg.pt',
    }

    def __init__(self) -> None:
        self._model = None
        self._device = "cpu"
        self._key = "yolov8n"

    def load_model(self, model_key: str, device: str = "cpu") -> None:
        """
        Load Ultralytics YOLO weights.
        """
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise ImportError(
                "ultralytics is required for YoloDetector. Install via 'pip install ultralytics'."
            ) from exc

        self._key = model_key
        self._device = device
        weights = self.MODEL_MAP.get(model_key, model_key)
        self._model = YOLO(weights)

    def detect(self, cv_image: Any) -> List[Dict[str, Any]]:
        """
        Run inference on image and return list of detected bounding boxes.
        """
        if self._model is None:
            raise RuntimeError("YOLO model not loaded. Call load_model() first.")

        results = self._model(cv_image, device=self._device, verbose=False)
        detections: List[Dict[str, Any]] = []

        for r in results:
            names = r.names
            if r.boxes is None:
                continue
            for b in r.boxes:
                coords = b.xyxy[0].tolist() if hasattr(b.xyxy[0], 'tolist') else [float(x) for x in b.xyxy[0]]
                cls_raw = b.cls.item() if hasattr(b.cls, 'item') else (b.cls[0] if hasattr(b.cls, '__getitem__') else b.cls)
                cls_id = int(cls_raw)
                conf_raw = b.conf.item() if hasattr(b.conf, 'item') else (b.conf[0] if hasattr(b.conf, '__getitem__') else b.conf)
                conf = float(conf_raw)

                class_name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(names[cls_id])
                detections.append({
                    'bbox': [float(x) for x in coords],
                    'class_name': class_name,
                    'confidence': conf,
                })

        return detections

    def get_model_info(self) -> Dict[str, Any]:
        """
        Return model description dictionary.
        """
        return {
            'name': self._key,
            'backend': 'ultralytics',
            'input_size': 640,
            'classes': 80,
        }
