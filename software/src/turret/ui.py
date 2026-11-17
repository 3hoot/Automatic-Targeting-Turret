import asyncio
import logging
import queue
import threading
import time
from collections import deque
from typing import Any

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, Query
from fastapi.responses import HTMLResponse, StreamingResponse

from .common import finish, put_latest, start_worker
from .config import AppConfig

log = logging.getLogger("ui")

ARROW_SECONDS = 0.25      # the velocity arrow shows where the ball will be in 0.25 s
TRACK_STALE_S = 1.0


class UiData:
    def __init__(self, log_lines: int, image_width: int, image_height: int) -> None:
        self.lock = threading.Lock()
        self.status: dict[str, dict] = {}          # source -> its latest report
        self.stamp: dict[str, float] = {}          # source -> time of that report
        self.esp_lines: deque = deque(maxlen=log_lines)
        self.jpeg: bytes | None = None
        self.jpeg_seq = 0
        self.image_width = image_width
        self.image_height = image_height
        self.point: tuple[int, int] | None = None
        self.calibration_grid: tuple[int, int] | None = None
        self.calibration_distance: float = 1.0

    def apply(self, source: str, fields: dict) -> None:
        with self.lock:
            if source == "esp_log":
                self.esp_lines.append(fields)
            else:
                self.status[source] = fields
                self.stamp[source] = time.time()

    def snapshot(self) -> dict:
        now = time.time()
        with self.lock:
            result = {s: {**f, "age_s": round(now - self.stamp[s], 1)}
                      for s, f in self.status.items()}
            if self.point is not None:
                result["selection"] = {"x": self.point[0], "y": self.point[1]}
            return result

    def recent_lines(self, n: int) -> list:
        with self.lock:
            return list(self.esp_lines)[-max(1, n):]

    def set_jpeg(self, jpeg: bytes) -> None:
        with self.lock:
            self.jpeg = jpeg
            self.jpeg_seq += 1

    def get_jpeg(self) -> tuple[int, bytes | None]:
        with self.lock:
            return self.jpeg_seq, self.jpeg

    def set_point(self, x: int, y: int) -> None:
        with self.lock:
            self.point = (x, y)

    def set_calibration_grid(self, spacing: int, width: int) -> None:
        with self.lock:
            self.calibration_grid = (spacing, width)

    def clear_calibration_grid(self) -> None:
        with self.lock:
            self.calibration_grid = None

    def set_calibration_distance(self, distance_m: float) -> None:
        with self.lock:
            self.calibration_distance = distance_m

    def overlay_inputs(self) -> tuple[dict | None, list[str], tuple[int, int] | None]:
        with self.lock:

            control = self.status.get("control")
            cam = self.status.get("camera", {})
            comms = self.status.get("comms", {}).get("state", "-")
            uart0 = self.status.get("uart0", {}).get("state", "-")
            distance = self.calibration_distance

            armed = bool(control and control.get("armed"))
            hud = [
                f"{'ARMED' if armed else 'SAFE'}   ",
                f"cam {cam.get('fps', 0.0):.0f} fps",
                f"comms {comms}   uart0 {uart0}    ",
                f"target distance {distance} m     "
            ]
            esp_error = self.status.get("esp_error")
            if esp_error and esp_error.get("active"):
                hud.append(f"ESP ERROR axis {esp_error['axis_id']}: run `turret reset-error`")
            return control, hud, self.calibration_grid


def draw_overlay(
    frame: np.ndarray,
    control: dict | None,
    hud: list[str],
    calibration_grid: tuple[int, int] | None = None,
) -> None:
    h, w = frame.shape[:2]
    cv2.drawMarker(frame, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 20, 1)   # barrel direction

    if calibration_grid is not None:
        spacing, line_width = calibration_grid
        for x in range(spacing, w, spacing):
            cv2.line(frame, (x, 0), (x, h - 1), (255, 255, 0), line_width)
        for y in range(spacing, h, spacing):
            cv2.line(frame, (0, y), (w - 1, y), (255, 255, 0), line_width)

    for i, line in enumerate(hud):
        cv2.putText(frame, line, (8, 18 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(frame, line, (8, 18 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


def event_loop(status_in: Any, data: UiData, stop: Any) -> None:
    while not stop.is_set():
        try:
            source, fields = status_in.get(timeout=0.2)
        except queue.Empty:
            continue
        data.apply(source, fields)


def render_loop(cfg: AppConfig, frames_in: Any, data: UiData, stop: Any) -> None:
    params = [cv2.IMWRITE_JPEG_QUALITY, cfg.ui.jpeg_quality]
    last_publish = 0.0
    while not stop.is_set():
        try:
            _, _, frame = frames_in.get(timeout=0.25)
        except queue.Empty:
            if time.monotonic() - last_publish > 1.0:       # camera silent: say so in the stream
                frame = np.zeros((cfg.camera.height, cfg.camera.width, 3), dtype=np.uint8)
                cv2.putText(frame, "NO CAMERA SIGNAL", (cfg.camera.width // 2 - 140, cfg.camera.height // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            else:
                continue
        else:
            if time.monotonic() - last_publish < 1.0 / cfg.ui.stream_fps:
                continue                                    # skip frames to save CPU
        control, hud, calibration_grid = data.overlay_inputs()
        draw_overlay(frame, control, hud, calibration_grid)
        ok, jpeg = cv2.imencode(".jpg", frame, params)
        if ok:
            data.set_jpeg(jpeg.tobytes())
            last_publish = time.monotonic()


async def mjpeg(data: UiData):
    last = -1
    while True:
        seq, jpeg = data.get_jpeg()
        if jpeg is None or seq == last:
            await asyncio.sleep(0.01)
            continue
        last = seq
        yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
               + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")


def make_app(data: UiData, to_control: Any, cfg: AppConfig) -> FastAPI:
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)

    @app.get("/", response_class=HTMLResponse)
    def home() -> str:
        return """<!doctype html>
                <html lang="en">
                <head>
                <meta charset="utf-8">
                <meta name="viewport" content="width=device-width, initial-scale=1">
                <title>Turret camera</title>
                <style>
                    body { margin: 1rem; background: #171717; color: #eee; font: 16px sans-serif; }
                    main { width: min(100%, 960px); margin: auto; }
                    #view { position: relative; display: inline-block; max-width: 100%; }
                    #stream { display: block; max-width: 100%; height: auto; cursor: crosshair; }
                    #marker { display: none; position: absolute; width: 14px; height: 14px;
                    border: 2px solid #ff3b30; border-radius: 50%; transform: translate(-50%, -50%);
                    pointer-events: none; }
                    #message { min-height: 1.5em; }
                </style>
                </head>
                <body>
                <main>
                    <h1>Turret camera</h1>
                    <p>Click the camera image to send its pixel coordinates.</p>
                    <div id="view">
                    <img id="stream" src="/stream.mjpg" alt="Live camera stream">
                    <span id="marker"></span>
                    </div>
                    <p id="message" aria-live="polite">Waiting for a camera frame…</p>
                </main>
                <script>
                    const image = document.getElementById("stream");
                    const marker = document.getElementById("marker");
                    const message = document.getElementById("message");
                    image.addEventListener("click", async (event) => {
                    if (!image.naturalWidth || !image.naturalHeight) return;
                    const bounds = image.getBoundingClientRect();
                    const x = Math.min(image.naturalWidth - 1,
                        Math.max(0, Math.floor((event.clientX - bounds.left) * image.naturalWidth / bounds.width)));
                    const y = Math.min(image.naturalHeight - 1,
                        Math.max(0, Math.floor((event.clientY - bounds.top) * image.naturalHeight / bounds.height)));
                    message.textContent = `Sending x=${x}, y=${y}…`;
                    try {
                        const response = await fetch(`/api/point?x=${x}&y=${y}`, { method: "POST" });
                        if (!response.ok) throw new Error(`HTTP ${response.status}`);
                        const point = await response.json();
                        marker.style.left = `${(point.x + 0.5) / image.naturalWidth * 100}%`;
                        marker.style.top = `${(point.y + 0.5) / image.naturalHeight * 100}%`;
                        marker.style.display = "block";
                        message.textContent = `Sent x=${point.x}, y=${point.y}`;
                    } catch (error) {
                        message.textContent = `Could not send point: ${error.message}`;
                    }
                    });
                </script>
                </body>
                </html>"""

    @app.get("/api/status")
    def get_status() -> dict:
        return data.snapshot()

    @app.post("/api/point")
    def point(
        x: int = Query(..., ge=0, le=data.image_width - 1),
        y: int = Query(..., ge=0, le=data.image_height - 1),
    ) -> dict:
        data.set_point(x, y)

        return {"ok": True, "x": x, "y": y}

    @app.post("/api/move")
    def move(
        pan: float = Query(..., ge=cfg.control.pan_min_deg, le=cfg.control.pan_max_deg),
        tilt: float = Query(..., ge=cfg.control.tilt_min_deg, le=cfg.control.tilt_max_deg),
    ) -> dict:
        put_latest(to_control, ("move", pan, tilt))
        return {"ok": True, "pan": pan, "tilt": tilt}

    @app.post("/api/grid")
    def grid(
        enabled: bool = True,
        spacing: int = Query(20, ge=1, le=10000),
        width: int = Query(2, ge=1, le=20),
    ) -> dict:
        if enabled:
            data.set_calibration_grid(spacing, width)
        else:
            data.clear_calibration_grid()
        return {"ok": True, "enabled": enabled, "spacing": spacing, "width": width}

    @app.post("/api/calibrate")
    def calibrate(distance_m: float = Query(1, ge=0.1, le=100)) -> dict:
        data.set_calibration_distance(distance_m)
        return {"ok": True}

    @app.get("/api/logs")
    def get_logs(n: int = 50) -> list:
        return data.recent_lines(n)

    @app.post("/api/arm")
    def arm() -> dict:
        put_latest(to_control, "arm")
        return {"ok": True}

    @app.post("/api/disarm")
    def disarm() -> dict:
        put_latest(to_control, "disarm")
        return {"ok": True}

    @app.post("/api/reset_error")
    def reset_error() -> dict:
        put_latest(to_control, "reset_error")
        return {"ok": True}

    @app.post("/api/shoot")
    def shoot() -> dict:
        put_latest(to_control, "shoot")
        return {"ok": True}

    @app.get("/stream.mjpg")
    async def stream() -> StreamingResponse:
        return StreamingResponse(mjpeg(data), media_type="multipart/x-mixed-replace; boundary=frame")

    return app


def run_ui(cfg: AppConfig, frames_in: Any, status_in: Any, to_control: Any, stop: Any) -> None:
    start_worker()
    data = UiData(cfg.ui.log_lines, cfg.camera.width, cfg.camera.height)
    threading.Thread(target=event_loop, args=(status_in, data, stop), daemon=True).start()
    threading.Thread(target=render_loop, args=(cfg, frames_in, data, stop), daemon=True).start()

    server = uvicorn.Server(uvicorn.Config(
        make_app(data, to_control, cfg), host=cfg.ui.host, port=cfg.ui.port,
        log_level="warning", access_log=False, timeout_graceful_shutdown=2))
    server_thread = threading.Thread(target=server.run, daemon=True)
    server_thread.start()
    log.info("UI on http://%s:%d", cfg.ui.host, cfg.ui.port)
    try:
        while not stop.is_set() and server_thread.is_alive():
            stop.wait(0.5)
        if not stop.is_set():
            log.error("HTTP server stopped (is the port already in use?)")
    finally:
        server.should_exit = True
        server_thread.join(timeout=5.0)
        finish(to_control)
