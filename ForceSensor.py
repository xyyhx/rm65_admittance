import serial
import os
from datetime import datetime

def open_serial_port(com_port='COM4'):
    try:
        ser = serial.Serial(
            port=com_port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        return ser
    except serial.SerialException as e:
        print(f"无法打开串口 {com_port}: {e}")
        return None

def log_force_data(data, filename="force_log.txt"):
    os.makedirs("forcedata", exist_ok=True)
    path = os.path.join("forcedata", filename)
    with open(path, 'a') as f:
        f.write(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} {data}\n")