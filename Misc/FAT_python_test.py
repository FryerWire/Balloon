import serial
import time
import numpy as np

port = 'COM3'  # Update this to the correct port
baud = 115200

try:
    with serial.Serial(port, baud, timeout=1) as ser, open("log.csv", "w") as f:
        start = time.time()
        while time.time() - start < 60:  # log for 60 seconds
            if ser.in_waiting:
                line = ser.readline().decode().strip()
                print(line)
                f.write(line + '\n')
except serial.SerialException as e:
    print(f"Error: Could not open port {port}. {e}")
