import logging
import queue
import signal
from typing import Any, NamedTuple


class Detection(NamedTuple):
    x: float
    y: float
    w: float
    h: float
    conf: float


def start_worker() -> None:
    # Ctrl+C reaches every process; only the parent reacts to it and sets `stop`.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(processName)-9s %(levelname)-7s %(message)s",
    )


def put_latest(q: Any, item: Any) -> None:
    try:
        q.put_nowait(item)
        return
    except queue.Full:
        pass
    try:
        q.get_nowait()
    except queue.Empty:
        pass
    try:
        q.put_nowait(item)
    except queue.Full:
        pass  # still full: drop the new item, a slow reader must never stall us


def drain(q: Any) -> list[Any]:
    items = []
    while True:
        try:
            items.append(q.get_nowait())
        except queue.Empty:
            return items


def report(status_q: Any, source: str, **fields: Any) -> None:
    # Send the latest state of `source` to the UI. A newer report replaces the old one.
    put_latest(status_q, (source, fields))


def finish(*queues: Any) -> None:
    for q in queues:
        q.cancel_join_thread()
