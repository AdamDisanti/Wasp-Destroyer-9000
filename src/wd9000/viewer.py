"""Live camera viewer with optional wasp-nest detection overlay.

Usage:
    python -m wd9000.viewer                                   # settings.yaml defaults
    python -m wd9000.viewer --source samples/paper_wasp_nest.jpg --backend mock
    python -m wd9000.viewer --source 0 --no-detect            # laptop webcam

Keys: q/Esc quit, d toggle detection, s save snapshot to output/
"""

import argparse
import logging
import sys
import time

import cv2
import numpy as np

from wd9000.camera.stream import FrameSource
from wd9000.config import PROJECT_ROOT, get_api_key, load_settings
from wd9000.vision.detector import make_detector
from wd9000.vision.visualization import draw_detections
from wd9000.vision.worker import InferenceWorker

log = logging.getLogger("viewer")

WINDOW = "WD9000"
OK_COLOR = (80, 220, 80)
WARN_COLOR = (0, 165, 255)
ERR_COLOR = (60, 60, 255)


def draw_hud(image: np.ndarray, lines: list[tuple[str, tuple[int, int, int]]]) -> None:
    """Draw status lines top-left on a translucent background, in place."""
    scale = max(0.45, image.shape[1] / 1400)
    line_h = int(26 * scale / 0.6)
    width = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] for t, _ in lines) + 16
    height = line_h * len(lines) + 8
    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (width, height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, image, 0.45, 0, dst=image)
    for i, (text, color) in enumerate(lines):
        cv2.putText(image, text, (8, line_h * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def placeholder(text: str) -> np.ndarray:
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(image, text, (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
    return image


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", help="stream URL, video/image file, or webcam index (overrides settings.yaml)")
    parser.add_argument("--backend", choices=["cloud", "local", "mock"], help="inference backend (overrides settings.yaml)")
    parser.add_argument("--no-detect", action="store_true", help="video only, no inference")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    settings = load_settings()
    cam_cfg = settings["camera"]
    view_cfg = settings.get("viewer", {})
    if args.backend:
        settings["inference"]["backend"] = args.backend
    backend = settings["inference"]["backend"]

    try:
        camera = FrameSource(
            args.source or cam_cfg["source"],
            timeout_s=cam_cfg.get("timeout_s", 5),
            reconnect_delay_s=cam_cfg.get("reconnect_delay_s", 1),
            loop_files=cam_cfg.get("loop_video_files", True),
        ).start()
    except FileNotFoundError as e:
        log.error("%s", e)
        return 1

    worker = None
    if not args.no_detect:
        try:
            detector = make_detector(settings, get_api_key())
        except Exception as e:
            log.error("Could not start %s inference backend: %s", backend, e)
            camera.stop()
            return 1
        worker = InferenceWorker(detector, camera.latest, view_cfg.get("inference_interval_s", 0)).start()

    stale_s = view_cfg.get("detection_stale_s", 2.0)
    snapshot_dir = PROJECT_ROOT / "output"
    log.info("Camera source: %s | inference: %s", camera.source, "off" if worker is None else backend)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    try:
        while True:
            frame = camera.latest()
            image = frame.image.copy() if frame else placeholder(f"Connecting to {camera.source} ...")

            cam_color = OK_COLOR if camera.status == "streaming" else WARN_COLOR
            lines = [(f"CAM {camera.status}  {camera.fps:4.1f} fps", cam_color)]
            if frame:
                age = time.monotonic() - frame.timestamp
                if age > 1.0:
                    lines.append((f"last frame {age:.1f}s ago", WARN_COLOR))
            if camera.last_error:
                lines.append((camera.last_error[:70], ERR_COLOR))

            if worker is None:
                lines.append(("DET off", WARN_COLOR))
            elif not worker.enabled:
                lines.append((f"DET {backend} paused", WARN_COLOR))
            else:
                result = worker.latest()
                fresh = result is not None and time.monotonic() - result.completed_at < stale_s
                if result is None:
                    lines.append((f"DET {backend} waiting for first result", WARN_COLOR))
                else:
                    lines.append((f"DET {backend}  {worker.fps:4.1f} fps  latency {result.latency_s * 1000:.0f} ms",
                                  OK_COLOR if fresh else WARN_COLOR))
                if fresh and frame:
                    image = draw_detections(image, result.detections)
                    top = ", ".join(f"{d.label} {d.confidence:.0%}" for d in result.detections[:3])
                    lines.append((f"{len(result.detections)} detection(s)  {top}", OK_COLOR))
                if worker.last_error:
                    lines.append((worker.last_error[:70], ERR_COLOR))
            lines.append(("q quit  d detect  s snapshot", (200, 200, 200)))

            draw_hud(image, lines)
            cv2.imshow(WINDOW, image)

            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("d") and worker is not None:
                worker.enabled = not worker.enabled
            if key == ord("s"):
                snapshot_dir.mkdir(exist_ok=True)
                path = snapshot_dir / f"snapshot_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
                cv2.imwrite(str(path), image)
                log.info("Saved %s", path)
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if worker:
            worker.stop()
        camera.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
