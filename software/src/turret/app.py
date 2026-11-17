import logging
import multiprocessing as mp
import signal
import time

from .camera import run_camera
from .config import AppConfig
from .control import run_control
from .serial_io import run_serial
from .ui import run_ui

log = logging.getLogger("main")


def run(cfg: AppConfig) -> None:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(processName)-9s %(levelname)-7s %(message)s")
    ctx = mp.get_context("spawn")        # every worker starts from a clean interpreter
    stop = ctx.Event()                   # set = "everybody exit"

    # Message queues (all bounded: a slow reader can never grow memory or latency)
    frames_ui = ctx.Queue(maxsize=2)      # camera    -> ui
    commands = ctx.Queue(maxsize=16)      # control   -> serial
    feedback = ctx.Queue(maxsize=64)      # serial    -> control
    to_control = ctx.Queue(maxsize=8)     # ui        -> control (arm/disarm)
    status = ctx.Queue(maxsize=500)       # everybody -> ui

    workers = [
        ctx.Process(target=run_serial, name="serial",
                    args=(cfg, commands, feedback, status, stop), daemon=True),
        ctx.Process(target=run_control, name="control",
                    args=(cfg, feedback, commands, to_control, status, stop), daemon=True),
        ctx.Process(target=run_camera, name="camera",
                    args=(cfg, frames_ui, status, stop), daemon=True),
        ctx.Process(target=run_ui, name="ui",
                    args=(cfg, frames_ui, status, to_control, stop), daemon=True),
    ]
    signal.signal(signal.SIGTERM, lambda *_: stop.set())    # e.g. systemd stop
    for worker in workers:
        worker.start()

    reported_dead: set[str] = set()
    try:
        while not stop.is_set():
            for worker in workers:
                if not worker.is_alive() and worker.name not in reported_dead:
                    reported_dead.add(worker.name)
                    log.error("worker %s died (exit code %s)", worker.name, worker.exitcode)
            stop.wait(1.0)
    except KeyboardInterrupt:
        log.info("Ctrl+C: shutting down")
    finally:
        stop.set()
        deadline = time.monotonic() + 5.0
        for worker in workers:
            worker.join(timeout=max(0.1, deadline - time.monotonic()))
        for worker in workers:
            if worker.is_alive():
                log.warning("worker %s did not stop, terminating it", worker.name)
                worker.terminate()
        log.info("done")
