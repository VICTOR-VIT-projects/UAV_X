"""Onboard perception: YOLOv8n person/vehicle detection on the UAV cameras.

The model is real (Ultralytics YOLOv8n, COCO weights) and runs on the actual
rendered camera frames. If ultralytics isn't installed the rest of the
simulation still runs, just without detection boxes.
"""

import os

KEEP = {"person", "car", "truck", "bus", "motorcycle", "bicycle"}


class Detector:
    def __init__(self, root, log=print):
        self.model = None
        self.names = {}
        self.error = None
        candidates = [os.path.join(root, "models", "yolov8n.pt"), "yolov8n.pt"]
        try:
            from ultralytics import YOLO
            path = next((p for p in candidates if os.path.exists(p)), "yolov8n.pt")
            self.model = YOLO(path)
            self.names = self.model.names
            import numpy as np
            self.model(np.zeros((480, 640, 3), dtype=np.uint8), imgsz=640, verbose=False)
            log(f"vision: YOLOv8n loaded from {path}")
        except Exception as e:                      # noqa: BLE001
            self.error = f"{type(e).__name__}: {e}"
            log(f"vision: detection disabled ({self.error})")

    @property
    def ok(self):
        return self.model is not None

    def detect(self, rgb, conf=0.3, iou=0.7):
        """rgb: HxWx3 uint8. Returns [(label, conf, x1, y1, x2, y2), ...].
        iou: non-maximum-suppression overlap threshold (ultralytics default
        0.7); higher keeps separate boxes for people standing close together."""
        if not self.model:
            return []
        r = self.model(rgb[:, :, ::-1], imgsz=640, conf=conf, iou=iou, verbose=False)[0]
        out = []
        for c, s, xyxy in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist(), r.boxes.xyxy.tolist()):
            label = self.names[int(c)]
            if label in KEEP:
                out.append((label, s, *xyxy))
        return out
