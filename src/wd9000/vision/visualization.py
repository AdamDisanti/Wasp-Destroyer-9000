"""Draw detections onto frames."""

import cv2
import numpy as np

from wd9000.vision.detector import Detection

BOX_COLOR = (0, 200, 255)  # BGR amber
TEXT_COLOR = (0, 0, 0)


def draw_detections(image: np.ndarray, detections: list[Detection]) -> np.ndarray:
    """Return a copy of image with boxes and confidence labels drawn on it."""
    out = image.copy()
    thickness = max(2, round(min(out.shape[:2]) / 300))
    font_scale = thickness * 0.3
    for d in detections:
        x1, y1, x2, y2 = d.corners
        cv2.rectangle(out, (x1, y1), (x2, y2), BOX_COLOR, thickness)
        text = f"{d.label} {d.confidence:.0%}"
        (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        ty = max(y1, th + baseline)
        cv2.rectangle(out, (x1, ty - th - baseline), (x1 + tw, ty), BOX_COLOR, -1)
        cv2.putText(out, text, (x1, ty - baseline), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, TEXT_COLOR, thickness, cv2.LINE_AA)
    return out
