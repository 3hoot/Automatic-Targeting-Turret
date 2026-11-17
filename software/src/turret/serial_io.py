import logging
import logging.handlers
import queue
import threading
import time
from pathlib import Path
from typing import Literal

import serial

from .common import drain, finish, put_latest, report, start_worker
from .config import AppConfig, SerialConfig
from .protocol import Command, ProtocolError, decode, encode

log = logging.getLogger("comms")

ESP_OK = 0                # esp_err_t: every reply from the ESP is one of these codes
FAILURES_BEFORE_NO_RESPONSE = 3
LOG_MAX_BYTES = 1_000_000
LOG_BACKUPS = 5


def open_port(cfg: SerialConfig, set_port: Literal["comms", "logs"] = "comms") -> serial.Serial:
    if set_port == "comms":
        port = cfg.comms.port
    else:
        port = cfg.logs.port

    # 8N1, no flow control (only TX, RX and GND are wired).
    return serial.Serial(port, cfg.baudrate, timeout=0.05, write_timeout=0.5)


def read_exact(ser: serial.Serial, count: int, deadline: float) -> bytes:
    """Read exactly `count` bytes or raise TimeoutError."""
    data = b""
    while len(data) < count:
        if time.monotonic() > deadline:
            raise TimeoutError
        data += ser.read(count - len(data))
    return data


class Comms:
    def __init__(self, cfg: AppConfig, feedback: object, status: object) -> None:
        self.cfg = cfg
        self.feedback = feedback
        self.status = status
        self.ser: serial.Serial | None = None
        self.state = ""
        self.failures = 0
        self.errors = 0          # number of ERROR frames received from the ESP

    def set_state(self, state: str, detail: str = "") -> None:
        if state == self.state:
            return
        self.state = state
        log.info("comms link %s %s", state, detail)
        put_latest(self.feedback, ("link", state))
        report(self.status, "comms", state=state, detail=detail)

    def connect(self) -> bool:
        try:
            self.ser = open_port(self.cfg.serial)
        except serial.SerialException as exc:
            self.set_state("down", str(exc))
            return False
        self.failures = 0
        self.set_state("no_response", "port open, waiting for the ESP")
        return True

    def drop(self, exc: Exception) -> None:
        if self.ser is not None:
            self.ser.close()
        self.ser = None
        self.set_state("down", str(exc))

    def read_frame(self, deadline: float) -> bytes:
        assert self.ser is not None, "self.ser cannot be None"
        head = read_exact(self.ser, 3, deadline)                  # opcode, length, checksum
        return head + read_exact(self.ser, head[1], deadline)     # + data

    def handle_stray(self, frame: bytes) -> None:
        if frame[0] == Command.MCTRL_ERROR:
            axis = decode(Command.MCTRL_ERROR, frame)["axis_id"]
            self.errors += 1
            log.error("ESP: control error on axis %d (axis stopped and disarmed)", axis)
            put_latest(self.feedback, ("error", axis))
            report(self.status, "esp_error", axis_id=axis, active=True, count=self.errors)
        else:
            log.warning("ignoring unexpected frame 0x%02X", frame[0])

    def read_strays(self) -> None:
        assert self.ser is not None, "self.set cannot be None"
        while self.ser.in_waiting:
            self.handle_stray(self.read_frame(time.monotonic() + 0.1))

    def listen(self) -> None:
        if self.ser is None:
            return
        try:
            self.read_strays()
        except (TimeoutError, ProtocolError):
            self.ser.reset_input_buffer()       # garbage: there is no sync byte, so start clean
        except (serial.SerialException, OSError) as exc:
            self.drop(exc)

    def exchange(self, command: Command, timeout_s: float | None = None, **fields: float) -> dict | None:
        # Send one command, return the decoded reply, or None if it failed.
        if self.ser is None:
            return None
        request = encode(command, **fields)      # here we pray that no errors pop up
        wait = self.cfg.serial.reply_timeout_s if timeout_s is None else timeout_s
        try:
            self.read_strays()                   # do not lose an ERROR that arrived before
            self.ser.write(request)
            deadline = time.monotonic() + wait
            while True:
                frame = self.read_frame(deadline)
                if frame[0] == command:
                    reply = decode(command, frame)
                    break
                self.handle_stray(frame)
        except (TimeoutError, ProtocolError) as exc:
            self.ser.reset_input_buffer()
            self.failures += 1
            if self.failures >= FAILURES_BEFORE_NO_RESPONSE:
                self.set_state("no_response", f"{type(exc).__name__} {exc}")
            return None
        except (serial.SerialException, OSError) as exc:
            self.drop(exc)
            return None
        self.failures = 0
        self.set_state("up")
        return reply


def next_requests(q: object, timeout: float) -> list[tuple]:
    try:
        first = q.get(timeout=timeout)  # type: ignore[attr-defined]
    except queue.Empty:
        return []
    requests: list[tuple] = []
    for request in [first] + drain(q):
        if request[0] == "move":                     # ("move", axis_id, position)
            requests = [r for r in requests if not (r[0] == "move" and r[1] == request[1])]
        requests.append(request)                     # ("stop", axis), ("reset_error", axis), ("trigger",)
    return requests


def run_request(comms: Comms, cfg: AppConfig, request: tuple) -> tuple[bool, str]:
    kind = request[0]
    if kind == "trigger":
        reply = comms.exchange(Command.SERVO_TRIGGER, timeout_s=cfg.serial.trigger_timeout_s)
        if reply is None:
            return False, "no reply"
        codes = (reply["result_1"], reply["result_2"])      # fire (70 deg), then home (110 deg)
        return all(code == ESP_OK for code in codes), f"{codes[0]},{codes[1]}"
    if kind == "move":
        reply = comms.exchange(Command.MCTRL_MOVE, axis_id=request[1], position=request[2])
    elif kind == "stop":
        reply = comms.exchange(Command.MCTRL_STOP, axis_id=request[1])
    else:  # "reset_error"
        reply = comms.exchange(Command.MCTRL_RESET_ERROR, axis_id=request[1])
    if reply is None:
        return False, "no reply"
    return reply["result"] == ESP_OK, str(reply["result"])


def log_reader(cfg: AppConfig, status: object, stop: object) -> None:
    # Thread: read the ESP debug UART, split into lines, write file + send to UI.
    file_log = logging.getLogger("esp.uart0")
    file_log.propagate = False
    file_log.setLevel(logging.INFO)
    path = Path(cfg.serial.esp_log_file)
    path.parent.mkdir(parents=True, exist_ok=True)
    handler = logging.handlers.RotatingFileHandler(
        path, maxBytes=LOG_MAX_BYTES, backupCount=LOG_BACKUPS, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s.%(msecs)03d %(message)s", "%Y-%m-%d %H:%M:%S"))
    file_log.addHandler(handler)

    ser: serial.Serial | None = None
    buf = b""
    while not stop.is_set():  # type: ignore[attr-defined]
        if ser is None:
            try:
                ser = open_port(cfg.serial, "logs")
            except serial.SerialException as exc:
                report(status, "uart0", state="down", detail=str(exc))
                stop.wait(cfg.serial.reconnect_s)  # type: ignore[attr-defined]
                continue
            report(status, "uart0", state="up", detail=cfg.serial.logs.port)
            buf = b""
        try:
            buf += ser.read(ser.in_waiting or 1)
        except (serial.SerialException, OSError) as exc:
            ser.close()
            ser = None
            report(status, "uart0", state="down", detail=str(exc))
            continue
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("utf-8", errors="replace").rstrip("\r")
            if line:
                file_log.info(line)
                report(status, "esp_log", ts=time.time(), line=line)
        if len(buf) > 1024:      # no newline for a long time: discard to bound memory
            buf = b""
    if ser is not None:
        ser.close()


def run_serial(cfg: AppConfig, commands_in: object, feedback_out: object,
               status: object, stop: object) -> None:
    start_worker()
    threading.Thread(target=log_reader, args=(cfg, status, stop), daemon=True, name="uart0-log").start()
    comms = Comms(cfg, feedback_out, status)
    axes = (cfg.control.pan_axis_id, cfg.control.tilt_axis_id)
    positions: dict[str, float] = {}
    triggers_ok = 0
    next_poll = 0.0
    try:
        while not stop.is_set():  # type: ignore[attr-defined]
            if comms.ser is None and not comms.connect():
                stop.wait(cfg.serial.reconnect_s)  # type: ignore[attr-defined]
                continue
            comms.listen()

            # 1) requests from the control worker
            for request in next_requests(commands_in, timeout=0.02):
                ok, codes = run_request(comms, cfg, request)
                if request[0] == "trigger" and ok:
                    triggers_ok += 1
                report(status, "command", name=request[0], ok=ok, codes=codes, triggers_ok=triggers_ok)
                if request[0] == "reset_error" and ok:
                    report(status, "esp_error", axis_id=request[1], active=False, count=comms.errors)

            # 2) read the axis positions
            if time.monotonic() >= next_poll:
                for axis_id in axes:
                    reply = comms.exchange(Command.MCTRL_GET_POS, axis_id=axis_id)
                    if reply is not None:
                        put_latest(feedback_out, ("pos", axis_id, reply["position"], time.monotonic()))
                        positions[f"axis_{axis_id}"] = reply["position"]
                if positions:
                    report(status, "position", **positions)
                next_poll = time.monotonic() + 1.0 / cfg.serial.poll_hz
    finally:
        # Leave the ESP with its control loops halted (STOP does not disarm the motors).
        for axis_id in axes:
            comms.exchange(Command.MCTRL_STOP, axis_id=axis_id)
        if comms.ser is not None:
            comms.ser.close()
        finish(feedback_out, status)
