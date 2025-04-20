
"""
Live Serial Data Logger
This script reads data from a live serial port and logs it to a CSV file.
"""



import serial



port = 'COM3'  
baud = 115200
try:
    with serial.Serial(port, baud, timeout = 1) as ser, open("walking_data.csv", "w") as f:
        while True:  # Infinite loop to keep logging until manually stopped
            if (ser.in_waiting):
                line = ser.readline().decode().strip()
                print(line)
                f.write(line + '\n')


except serial.SerialException as e:
    print(f"Error: Could not open port {port}. {e}")