"""Run the wasp-nest model on one image and save an annotated copy.

Usage:
    python scripts/infer_image.py samples/paper_wasp_nest.jpg
    python scripts/infer_image.py my.jpg --backend mock --show
"""

import argparse
import json
import logging
import sys
import time
from pathlib import Path

import cv2

from wd9000.config import PROJECT_ROOT, get_api_key, load_settings
from wd9000.vision.detector import make_detector
from wd9000.vision.visualization import draw_detections

log = logging.getLogger("infer_image")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image", type=Path)
    parser.add_argument("--backend", choices=["cloud", "local", "mock"], help="override settings.yaml")
    parser.add_argument("--out", type=Path, help="annotated output path (default: output/<name>_detected.jpg)")
    parser.add_argument("--show", action="store_true", help="also open the result in a window")
    parser.add_argument("--raw", action="store_true", help="print the raw Roboflow JSON response")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    image = cv2.imread(str(args.image))
    if image is None:
        log.error("Could not read image: %s", args.image)
        return 1

    settings = load_settings()
    if args.backend:
        settings["inference"]["backend"] = args.backend
    backend = settings["inference"]["backend"]

    try:
        detector = make_detector(settings, get_api_key())
        start = time.perf_counter()
        detections = detector.detect(image)
        elapsed_ms = (time.perf_counter() - start) * 1000
    except Exception as e:
        log.error("Inference failed (%s backend): %s", backend, e)
        return 1

    log.info("%s backend: %d detection(s) in %.0f ms", backend, len(detections), elapsed_ms)
    for d in detections:
        log.info("  %-12s %5.1f%%  center=(%.0f, %.0f)  size=%.0fx%.0f",
                 d.label, d.confidence * 100, d.x, d.y, d.width, d.height)
    if args.raw and hasattr(detector, "last_result"):
        print(json.dumps(detector.last_result, indent=2))

    out = args.out or PROJECT_ROOT / "output" / f"{args.image.stem}_detected.jpg"
    out.parent.mkdir(parents=True, exist_ok=True)
    annotated = draw_detections(image, detections)
    cv2.imwrite(str(out), annotated)
    log.info("Saved %s", out)

    if args.show:
        cv2.imshow("WD9000 detection", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
