import serial
import logging

from transport import frame as f, framed_serial as fs

logging.basicConfig(level=logging.INFO)

serial_port = serial.Serial(port="/dev/ttyACM1", baudrate=115200, timeout=1)
with fs.FramedSerial(serial_port) as framed_serial:

    # Placeholder for analysis purposes.


    cmd = f.Frame(f.Type.MOVE, f.Direction.TX, 
                  axis_id=0, end_position=0.0, step_frequency_hz=8000, flags=0)
    # cmd = f.Frame(f.Type.GET_POS, f.Direction.TX, axis_id=1)
    # cmd = f.Frame(f.Type.RESET_WATCHDOG, f.Direction.TX, axis_id=0)
    print(f"Raw: {cmd.pack().hex()}")
    
    framed_serial.transmit(cmd)

    response = framed_serial.receive(timeout=1.0)
    print(f"Received response: {response.result if response else 'No response received'}")
