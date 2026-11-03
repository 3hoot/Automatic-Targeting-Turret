import time
import serial
import csv
import logging

from transport import frame as f, framed_serial as fs

logging.basicConfig(level=logging.INFO)

log_data = []
serial_port = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=1)
with fs.FramedSerial(serial_port) as framed_serial:

    # Placeholder for analysis purposes.
    ANALISED_AXIS_ID = 0
    SWEEP_FREQUENCY_HZ = 4000
    SWEEP_START = -50.0
    SWEEP_END = 50.0
    LOG_COUNT = 1000

    # Homing sequence
    framed_serial.transmit(f.Frame(f.Type.MOVE, f.Direction.TX,
                                   axis_id=ANALISED_AXIS_ID,
                                   end_position=SWEEP_START,
                                   step_frequency_hz=SWEEP_FREQUENCY_HZ,
                                   flags=1)) # Block until the move is complete
    dummy_response = framed_serial.receive(timeout=1.0) 

    time.sleep(5) # Wait for a few seconds to ensure the homing sequence is complete
    # Discard any log data that might have been generated during the homing sequence
    framed_serial.transmit(f.Frame(f.Type.LOG, f.Direction.TX,
                                   axis_id=ANALISED_AXIS_ID,
                                   log_count=LOG_COUNT))
    actual_count = 0
    count_response = framed_serial.receive(timeout=1.0)
    if count_response and count_response.frame_type == f.Type.LOG_RESPONSE:
        actual_count = count_response.sample_count
        print(f"Discarding {actual_count} log frames generated during homing sequence.")
    for _ in range(actual_count):
        _ = framed_serial.receive(timeout=1.0)  # Discard the log data

    # Sweeping sequence
    framed_serial.transmit(f.Frame(f.Type.MOVE, f.Direction.TX,
                                   axis_id=ANALISED_AXIS_ID,
                                   end_position=SWEEP_END,
                                   step_frequency_hz=SWEEP_FREQUENCY_HZ,
                                   flags=1)) # Block until the move is complete
    dummy_response = framed_serial.receive(timeout=1.0)

    time.sleep(5)  # Wait for a few seconds to ensure the sweeping sequence is complete
    # Logging sequence
    framed_serial.transmit(f.Frame(f.Type.LOG, f.Direction.TX,
                                   axis_id=ANALISED_AXIS_ID,
                                   log_count=LOG_COUNT))
    count_response = framed_serial.receive(timeout=1.0)
    if count_response and count_response.frame_type == f.Type.LOG_RESPONSE:
        actual_count = count_response.sample_count
        print(f"Expecting {actual_count} log frames from the logging sequence.")
    for _ in range(actual_count):
        frame = framed_serial.receive(timeout=1.0)
        if frame is None:
            logging.warning("Timeout while waiting for log data.")
            continue
        log_data.append(frame)

# Saving data to CSV
with open("log_data.csv", mode="w", newline="") as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(["timestamp_us", "current", "voltage", "angle", "position",
                         "target_position", "step_position", "commanded_hz", "applied_hz"])
    for frame in log_data:
        if frame.frame_type == f.Type.LOG:
            csv_writer.writerow([
                frame.timestamp_us,
                frame.current,
                frame.voltage,
                frame.angle,
                frame.position,
                frame.target_position,
                frame.step_position,
                frame.commanded_hz,
                frame.applied_hz
            ])
            