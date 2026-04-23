# -*- coding: utf-8 -*-
# 本程序为调整方向角、恒力控制
import numpy as np
import math
from module.six_force_read import six_force_read
from module.ForceSensor import *
# … 省略其他 import …


# —— 全局参数 —— #
# 重力补偿的质心位置
_L_CENTER = np.array([-1.70349578, 1.88272111, 3.35274154]).reshape(3, 1)

# 力和力矩的初始偏移量
_FORCE_OFFSET = np.array([
    3.3305045432326734 + 0.64299124,
    7.012257218018426 - 0.66152334,
    5.0224469004951615 + 0.17140979
]).reshape(3, 1)

_MOMENT_OFFSET = np.array([
    0.21734239266491043 - 1.30018643,
    -0.0545369615565644 + 1.40889167,
    0.10935129982501432 - 0.38617355
]).reshape(3, 1)


def _rotation_matrix_from_euler(angles):
    """
    根据欧拉角 (rx, ry, rz) 生成旋转矩阵，顺序：先绕 X，再绕 Y，最后绕 Z。
    angles: iterable of three floats, 单位为弧度。
    返回 3×3 的 NumPy 矩阵。
    """
    rx, ry, rz = angles

    # 采用 np.cos / np.sin 直接作用于标量
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    R_x = np.array([
        [1, 0, 0],
        [0, cx, -sx],
        [0, sx, cx]
    ])
    R_y = np.array([
        [cy, 0, sy],
        [0, 1, 0],
        [-sy, 0, cy]
    ])
    R_z = np.array([
        [cz, -sz, 0],
        [sz, cz, 0],
        [0, 0, 1]
    ])

    # 注意矩阵乘法顺序：Z @ Y @ X
    return R_z @ R_y @ R_x


def gravity_compensate(euler_angles):
    """
    计算在当前关节姿态下，重力在基座坐标系下的分量。
    euler_angles: 长度为3的 iterable（弧度制），对应绕 X/Y/Z 的旋转。
    返回 shape=(3,1) 的 NumPy 列向量 [g_x, g_y, g_z]^T。
    """
    R = _rotation_matrix_from_euler(euler_angles)
    # 从末端坐标系到基座坐标系的旋转矩阵
    R_end_to_base = R.T
    # 质心位置向量，形状 (3,1)
    return R_end_to_base @ _L_CENTER


def read_and_compensate(robot, ser):
    """
    读取六自由度力/力矩传感器数据并做重力及偏置补偿。

    参数:
      robot: 能返回机器人当前状态的对象，Get_Current_Arm_State()[2] 包含位姿信息。
      ser: 串口或通信句柄，用于 six_force_read()。

    返回:
      一个 6 元组 (fx, fy, fz, mx, my, mz)，均已补偿，均为标量。
    """
    # 1. 原始读数
    fx, fy, fz, mx, my, mz = six_force_read(ser)
    # 确保读数转为列向量
    F_raw = np.array([fx, fy, fz]).reshape(3, 1)
    M_raw = np.array([mx, my, mz]).reshape(3, 1)

    # 2. 获取当前末端姿态和欧拉角（弧度制）
    pose = robot.Get_Current_Arm_State()[2]
    xyz = np.array(pose[0:3]).reshape(3, 1)
    euler = pose[3:6]

    # 3. 重力分量
    g = gravity_compensate(euler)

    # 4. 计算重力引起的力矩补偿：m = r × g
    #    这里用向量叉乘实现
    mg = np.cross(xyz.flatten(), g.flatten()).reshape(3, 1)

    # 5. 力和力矩补偿
    F_actual = F_raw - g - _FORCE_OFFSET
    M_actual = M_raw - mg - _MOMENT_OFFSET

    # 6. 打印调试信息
    fx_a, fy_a, fz_a = F_actual.flatten()
    mx_a, my_a, mz_a = M_actual.flatten()
    print(f"fx_act: {fx_a:.6f}, fy_act: {fy_a:.6f}, fz_act: {fz_a:.6f}, "
          f"Mx_act: {mx_a:.6f}, My_act: {my_a:.6f}, Mz_act: {mz_a:.6f}")

    # 7. 返回标量
    return fx_a, fy_a, fz_a, mx_a, my_a, mz_a


def main():
    # —— 2. 初始化机械臂、串口等 —— #
    robot = Arm(RM65, "192.168.1.18")
    ser = open_serial_port(com_port="COM4")

    # file_path = "data0112.txt"
    file_path = datetime.now().strftime("%Y%m%d_%H%M%S") + ".txt"
    all_data_to_write = []

    try:
        for _ in range(500):
            # 2.1 读取并补偿力/力矩
            fx, fy, fz, mx, my, mz = read_and_compensate(robot, ser)

            # 2.2 记录数据
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            line = f"{timestamp} {fx:.3f} {fy:.3f} {fz:.3f} " \
                   f"{mx:.3f} {my:.3f} {mz:.3f}\n"
            all_data_to_write.append(line)

            # 采样间隔，根据需要调整
            time.sleep(0.01)

    except Exception as e:
        print(f"采集过程中发生错误: {e}")

    finally:
        # 关闭资源
        try:
            ser.close()
        except Exception:
            pass
        try:
            robot.shutdown()  # 或者 robot.disconnect()
        except Exception:
            pass

        # 写文件
        with open(file_path, 'w') as f:
            f.writelines(all_data_to_write)
        print(f"已将 {len(all_data_to_write)} 条数据写入 {file_path}")


if __name__ == '__main__':
    main()
