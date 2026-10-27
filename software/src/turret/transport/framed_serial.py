import logging
import queue
import threading
import serial

from transport.frame import Direction, Frame

MAX_RX_QUEUE_SIZE = 1000
DISCONNECT_TIMEOUT_S = 1.0 

logger = logging.getLogger(__name__)


class FramedSerial:
    def __init__(self, serial: serial.Serial):
        self._serial = serial
        self._tx_lock = threading.Lock()

        self._rx_thread: threading.Thread | None = None
        self._running = threading.Event()
        self._rx_queue: queue.Queue[Frame] = queue.Queue(maxsize=MAX_RX_QUEUE_SIZE) 

    def connect(self) -> None:
        if not self._serial.is_open:
            self._serial.open()

        if self._running.is_set():
            return

        self._serial.reset_input_buffer()
        
        self._running.set()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="FramedSerial RX Thread")
        self._rx_thread.start()

    def disconnect(self) -> None:
        self._running.clear()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=DISCONNECT_TIMEOUT_S)

        if self._serial and self._serial.is_open:
            self._serial.close()

    def transmit(self, frame: Frame) -> None:
        if not self._serial or not self._serial.is_open:
            raise RuntimeError("Cannot transmit frame: Serial port is not open")

        raw_bytes = frame.pack()
        with self._tx_lock:
            self._serial.write(raw_bytes)
            self._serial.flush()

    def receive(self, timeout: float | None = None) -> Frame | None:
        try:
            return self._rx_queue.get(block=True, timeout=timeout)
        except queue.Empty:
            return None

    def _rx_loop(self) -> None:
        assert self._serial is not None, "Serial port is not initialized"

        while self._running.is_set():
            try:
                header = self._serial.read(3)
                if len(header) < 3:
                    continue # Wait until we have enough bytes for a header

                data_length = header[1]

                data = b""
                if data_length > 0:
                    data = self._serial.read(data_length)
                    if len(data) < data_length:
                        continue # Truncated read, wait for the next iteration to read again

                raw_frame = header + data
                try:
                    frame = Frame.unpack(raw_frame, direction=Direction.RX)
                    self._rx_queue.put(frame)
                except ValueError as e:
                    logger.warning(f"Failed to unpack frame: {e}")
                    continue # Drop corrupted frame and continue reading

            except (serial.SerialException, OSError) as e:
                logger.error(f"Serial read error: {e}")
                self._running.clear()  # Stop the loop on serial errors
                break

    def __enter__(self) -> "FramedSerial":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.disconnect()