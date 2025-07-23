import time

from module.gravity_compensation import *
from module.six_force_read import *
from Robotic_Arm.rm_robot_interface import *

if __name__ == '__main__':
    data = [
        ([1.6, 4.54, 1.59], [0.031, 0.01, 0.08], [180, 0, 0]),
        ([-0.65, 6.83, 0.47], [0.433, 0.242, 0.094], [0, -90, 0]),
        ([3.96, 4.64, -0.35], [0.023, 0.004, 0.108], [0, 0, 180])
    ]
    # data = [
    #     ([3.51, 2.77, 2.21], [-0.06, -0.015, 0.091], [180, 0, 0]),
    #     ([-2.33, 5.98, 0.8], [0.393, 0.229, 0.07], [0, -90, 0]),
    #     ([2.12, 4.22, -0.29], [0.009, 0.007, 0.082], [0, 0, 180])
    # ]

    compensator = GravityCompensator(data)
    ser = open_serial_port(com_port="com4")
    # 实例化RoboticArm类
    # robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    robot = Arm(RM65, "192.168.1.18")
    # 创建机械臂连接，打印连接id
    # handle = robot.rm_create_robot_arm("192.168.1.18", 8080)
    # print(handle.id)
    # for _ in range(100):
    six_dim_force = []
    file_path = datetime.now().strftime("%Y%m%d_%H%M%S") + ".txt"  # 力数据存储文件名
    try:
        while True:
            fx, fy, fz, mx, my, mz = six_force_read(ser)
            # cur_pose = robot.rm_get_current_arm_state()[2]
            cur_pose = robot.Get_Current_Arm_State()[2]

            rx, ry, rz = cur_pose[3:6]
            f_ex, m_ex = compensator.compensate(
                [fx, fy, fz],
                [mx, my, mz],
                [rx/3.14*180, ry/3.14*180, rz/3.14*180]
            )
            date_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            six_dim_force.append(f"{date_time_str} {f_ex[0]} {f_ex[1]} {f_ex[2]} {m_ex[0]} {m_ex[1]} {m_ex[2]}\n")

            print(f"{f_ex}\t{m_ex}")
            time.sleep(0.5)
    finally:
        with open(file_path, 'a', encoding='utf-8') as file:
            file.writelines(six_dim_force)
        ser.close()
        robot.RM_API_UnInit()
        robot.Arm_Socket_Close()
    # robot.rm_delete_robot_arm()
