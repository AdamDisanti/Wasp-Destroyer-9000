import numpy as np
import pytest

from wd9000.vision.detector import Detection, MockDetector, make_detector, parse_predictions
from wd9000.vision.visualization import draw_detections

# Trimmed real response from wasps-nest/1 (cloud backend).
SAMPLE_RESPONSE = {
    "image": {"width": 1280, "height": 960},
    "predictions": [
        {"x": 719.5, "y": 357.5, "width": 871.0, "height": 651.0,
         "confidence": 0.595, "class": "nests", "class_id": 0},
        {"x": 619.0, "y": 541.5, "width": 72.0, "height": 89.0,
         "confidence": 0.835, "class": "wasps", "class_id": 1},
        {"x": 672.0, "y": 279.0, "width": 126.0, "height": 154.0,
         "confidence": 0.443, "class": "wasps", "class_id": 1},
    ],
}


def test_parse_predictions_sorted_by_confidence():
    detections = parse_predictions(SAMPLE_RESPONSE)
    assert [d.confidence for d in detections] == [0.835, 0.595, 0.443]
    assert detections[1].label == "nests"


def test_parse_predictions_filters_low_confidence():
    detections = parse_predictions(SAMPLE_RESPONSE, min_confidence=0.5)
    assert len(detections) == 2
    assert all(d.confidence >= 0.5 for d in detections)


def test_parse_predictions_empty():
    assert parse_predictions({"predictions": []}) == []
    assert parse_predictions({}) == []


def test_corners_from_center():
    d = Detection("nests", 0.9, x=100, y=50, width=40, height=20)
    assert d.corners == (80, 40, 120, 60)


def test_mock_detector_stays_inside_frame():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    detector = MockDetector(seed=0)
    for _ in range(50):
        for d in detector.detect(image):
            x1, y1, x2, y2 = d.corners
            assert 0 <= x1 < x2 <= 640
            assert 0 <= y1 < y2 <= 480


def test_make_detector_mock_needs_no_key():
    settings = {"inference": {"backend": "mock"}}
    assert isinstance(make_detector(settings, api_key=None), MockDetector)


def test_make_detector_cloud_requires_key():
    settings = {"inference": {"backend": "cloud", "model_id": "wasps-nest/1",
                              "cloud_url": "https://example.invalid"}}
    with pytest.raises(RuntimeError, match="ROBOFLOW_API_KEY"):
        make_detector(settings, api_key=None)


def test_make_detector_rejects_unknown_backend():
    with pytest.raises(ValueError):
        make_detector({"inference": {"backend": "gpu-magic"}}, api_key="x")


def test_draw_detections_does_not_modify_input():
    image = np.zeros((200, 200, 3), dtype=np.uint8)
    out = draw_detections(image, [Detection("nests", 0.8, 100, 100, 50, 50)])
    assert image.sum() == 0
    assert out.sum() > 0
