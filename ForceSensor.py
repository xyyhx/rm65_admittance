import struct
import serial
from datetime import datetime
from robotic_arm_package.robotic_arm import *
import cv2


def hex_to_ieee754(hex_str):
    hex_str = hex_str.replace("0x", "").replace(" ", "").upper()
    if len(hex_str) == 8:
        float_value = struct.unpack('!f', bytes.fromhex(hex_str))[0]
    elif len(hex_str) == 16:
        float_value = struct.unpack('!d', bytes.fromhex(hex_str))[0]
    else:
        raise ValueError("输入的十六进制字符串长度无效，应该为8位或16位。")
    return round(float_value, 4)


def log_force_data(force_number, data):
    now = datetime.now()
    date_str = now.strftime("%Y-%m-%d")
    # filename = f"{date_str}_sensor_data.txt"
    filename = os.path.join("forcedata", f"{date_str}_{data}_data.txt")
    with open(filename, 'a') as file:
        file.write(f"{now.strftime('%Y-%m-%d %H:%M:%S')}  {force_number}\n")


def open_serial_port(com_port):
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


class ForceSensor:
    def __init__(self, com_port='COM4'):
        self.ser = open_serial_port(com_port)
        self.robot = Arm(RM65, "192.168.1.18")

    def read_force_sensor(self):
        if not self.ser or not self.ser.isOpen():
            print("串口未打开或连接丢失")
            return 0

        try:
            cur_weight = bytearray([0x01, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xCB])
            self.ser.write(cur_weight)
            response = self.ser.read(9)

            if response:
                hex_str = response[3:7].hex()
                force_val = hex_to_ieee754(hex_str)
                print(force_val)

                if force_val > 5 or force_val < -5:
                    self.robot.Move_Stop_Cmd()

                log_force_data(force_val)
                return force_val
            else:
                print("没有接收到数据。")
        except serial.SerialException as e:
            print(f"串口通信时发生错误: {e}")
        except ValueError as e:
            print(f"数据处理错误: {e}")
        return 0

    def move_arm(self, step_size, force):
        current_pose = self.robot.Get_Current_Arm_State()[2]
        new_pose = current_pose[:3]
        new_pose[2] += step_size
        self.robot.Force_Position_Move_Pose(new_pose + current_pose[3:], 1, 0, 2, force, False)

    def close_serial_port(self):
        if self.ser and self.ser.isOpen():
            self.ser.close()
            print("串口已关闭")

    def close_connection(self):
        self.robot.RM_API_UnInit()
        self.robot.Arm_Socket_Close()

    def z_axis_control(self):
        z_force = self.read_force_sensor()
        self.robot.Start_Force_Position_Move()
        initial_force = z_force
        now = datetime.now()
        time_str = now.strftime('%Y-%m-%d %H:%M:%S')
        print(f"{time_str}  {z_force}")
        # try:
        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            z_force = self.read_force_sensor()
            now = datetime.now()
            time_str = now.strftime('%Y-%m-%d %H:%M:%S')
            print(z_force)
            if z_force > abs(initial_force) + 0.5:
                print(f"{time_str} : {z_force}")
                print("z负方向调节")
                self.move_arm(-0.001, -10 * abs(z_force))
            elif z_force < -abs(initial_force) - 0.5:
                print(f"{time_str} : {z_force}")
                print("z正方向调节")
                self.move_arm(0.001, 10 * abs(z_force))
            else:
                print("保持不动")
                print(f"{time_str}  {z_force}")
        # except KeyboardInterrupt:
        #     print("检测到中断，停止控制。")
        # finally:
        #     print("机械臂停止运动，关闭力控模式")

    def __del__(self):
        self.close_serial_port()
        self.close_connection()


if __name__ == '__main__':
    # ser = open_serial_port(com_port="com4")
    # robot = Arm(RM65, "192.168.1.18")
    fs = ForceSensor()
    while True:
        force_value = fs.read_force_sensor()
