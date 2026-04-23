import os
import time
from datetime import datetime
from typing import Tuple, Optional
from module.ForceSensor import open_serial_port, hex_to_ieee754


def read_six_force(serial, log_dir: str = "forcedata") -> Optional[Tuple[float, ...]]:
    if not serial or not serial.is_open:
        print("串口未打开")
        return None

    try:
        cmd = bytearray([0x01, 0x04, 0x00, 0x00, 0x00, 0x0C, 0xB1, 0xDF])
        serial.write(cmd)
        response = serial.read(29)

        if len(response) < 27:
            print("接收数据长度不足")
            return None

        fx = hex_to_ieee754(response[11:15].hex())
        fy = hex_to_ieee754(response[7:11].hex())
        fz = hex_to_ieee754(response[3:7].hex())
        mx = hex_to_ieee754(response[23:27].hex())
        my = hex_to_ieee754(response[19:23].hex())
        mz = hex_to_ieee754(response[15:19].hex())

        now = datetime.now()
        timestamp = now.strftime("%Y-%m-%d %H:%M:%S")
        date_str = now.strftime("%Y-%m-%d")
        print(f"{timestamp}  {fx:.4f} {fy:.4f} {fz:.4f} {mx:.4f} {my:.4f} {mz:.4f}")

        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, f"{date_str}_data.txt")
        with open(log_path, "a") as f:
            f.write(f"{timestamp}  {fx:.4f} {fy:.4f} {fz:.4f} {mx:.4f} {my:.4f} {mz:.4f}\n")

        return fx, fy, fz, mx, my, mz

    except Exception as e:
        print(f"读取力传感器异常: {e}")
        return None


if __name__ == "__main__":
    ser = open_serial_port(com_port="COM4")
    if ser:
        try:
            while True:
                read_six_force(ser)
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("退出监控")
        finally:
            ser.close()
