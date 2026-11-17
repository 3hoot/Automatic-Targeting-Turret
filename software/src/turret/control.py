import logging
import math
import queue
import time

from .common import drain, finish, put_latest, report, start_worker
from .config import AppConfig, ControlConfig

log = logging.getLogger("control")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))

def aim_target(position: float, error_deg: float, gain: float,
               deadband_deg: float, low: float, high: float) -> float | None:
    log.debug("aim_target not implemented yet!")
    return None


def stop_axes(commands: object, c: ControlConfig) -> None:
    for axis_id in (c.pan_axis_id, c.tilt_axis_id):
        put_latest(commands, ("stop", axis_id))

def run_control(cfg: AppConfig, feedback: object, commands: object,
                from_ui: object, status: object, stop: object) -> None:
    start_worker()
    c = cfg.control
    cx, cy = cfg.camera.width / 2, cfg.camera.height / 2
    focal_px = cx / math.tan(math.radians(c.hfov_deg) / 2)     # focal length in pixels

    armed = False
    comms_up = False
    error_axes: set[int] = set()
    positions: dict[int, tuple[float, float]] = {}   # axis_id -> (degrees, time received)
    shots = 0
    last_shot = -1e9
    last_move = 0.0
    last_report = 0.0
    try:
        while not stop.is_set():  # type: ignore[attr-defined]
            for msg in drain(feedback):
                if msg[0] == "pos":
                    positions[msg[1]] = (msg[2], msg[3])
                elif msg[0] == "link":
                    comms_up = msg[1] == "up"
                elif msg[0] == "error":
                    # The ESP has stopped and disarmed that axis: stop everything, stay disarmed.
                    error_axes.add(msg[1])
                    if armed:
                        log.error("axis %d error: disarmed", msg[1])
                    armed = False
                    stop_axes(commands, c)

            for msg in drain(from_ui):
                if msg == "arm":
                    if error_axes:
                        log.error("not armed: axis error pending, fix it and run `turret reset-error`")
                    else:
                        armed = True
                        log.warning("ARMED")
                elif msg == "disarm":
                    armed = False
                    stop_axes(commands, c)
                    log.warning("disarmed")
                elif msg == "reset_error":
                    for axis_id in sorted(error_axes) or [c.pan_axis_id, c.tilt_axis_id]:
                        put_latest(commands, ("reset_error", axis_id))
                    error_axes.clear()
                    log.warning("RESET_ERROR sent; arm again to continue")
                elif msg == "shoot":
                    if armed == False:
                        log.warning("not armed: run `turret arm` first")
                    put_latest(commands, ("trigger",))
                elif isinstance(msg, tuple) and msg[0] == "move":
                    put_latest(commands, ("move", c.pan_axis_id, msg[1]))
                    put_latest(commands, ("move", c.tilt_axis_id, msg[2]))

            now = time.monotonic()

            if now - last_report >= 0.5:
                last_report = now
                report(status, "control", armed=armed)
    finally:
        finish(commands, status)
