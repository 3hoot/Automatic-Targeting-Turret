import logging
import time

import cv2

from .common import finish, put_latest, report, start_worker
from .config import AppConfig, CameraConfig

log = logging.getLogger("camera")
MAX_READ_FAILURES = 5


def open_camera(cfg: CameraConfig) -> cv2.VideoCapture | None:
    cap = cv2.VideoCapture(cfg.index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))  # USB cameras need MJPG for 30 fps
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
    cap.set(cv2.CAP_PROP_FPS, cfg.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep the driver queue short: low latency
    return cap


def run_camera(cfg: AppConfig, frames_ui: object,
               status: object, stop: object) -> None:
    start_worker()
    cam = cfg.camera
    cap = None
    failures = 0
    frame_id = 0
    frame_count = 0 # used for FPS 
    window_start = time.monotonic()
    try:
        while not stop.is_set():  # type: ignore[attr-defined]
            if cap is None:
                cap = open_camera(cam)
                if cap is None:
                    report(status, "camera", state="disconnected", fps=0.0)
                    stop.wait(cam.retry_s)  # type: ignore[attr-defined]
                    continue
                failures = 0
                log.info("camera connected")

            ok, frame = cap.read()
            if not ok:
                failures += 1
                time.sleep(0.01)
                if failures >= MAX_READ_FAILURES:
                    log.warning("camera lost, will retry")
                    cap.release()
                    cap = None
                    report(status, "camera", state="disconnected", fps=0.0)
                continue
            failures = 0

            if frame.shape[1] != cam.width or frame.shape[0] != cam.height:
                frame = cv2.resize(frame, (cam.width, cam.height))
            frame = cv2.rotate(frame, cv2.ROTATE_180) # Fix upside down mounting
            now = time.monotonic()
            frame_id += 1
            put_latest(frames_ui, (frame_id, now, frame))

            frame_count += 1
            if now - window_start >= 1.0:
                report(status, "camera", state="connected", fps=frame_count / (now - window_start))
                frame_count, window_start = 0, now
    finally:
        if cap is not None:
            cap.release()
        finish(frames_ui, status)
