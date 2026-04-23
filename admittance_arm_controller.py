# -*- coding: utf-8 -*-
"""
机械臂导纳控制核心模块
整合重力补偿、多维导纳计算与机械臂实时控制
"""
import numpy as np
import time
from module.MultiAdmittance import MultiAxisAdmittanceController
from module.gravity_compensation import GravityCompensator
from module.six_force_read import read_six_force
from module.ForceSensor import open_serial_port

# 注意：需确保 robotic_arm 环境已配置
try:
    from robotic_arm_package.robotic_arm import Arm, RM65
except ImportError:
    print("警告：未找到 robotic_arm 包，机械臂功能将不可用")
    Arm = None
    RM65 = None

class AdmittanceArmController:
    def __init__(self, robot_ip, com_port, calibration_data):
        if Arm is None:
            raise ImportError("缺少 robotic_arm 依赖")

        # 1. 初始化硬件
        self.robot = Arm(RM65, robot_ip)
        self.ser = open_serial_port(com_port)
        if not self.ser:
            raise RuntimeError("串口连接失败")

        # 2. 初始化重力补偿器
        self.gravity_comp = GravityCompensator(calibration_data)

        # 3. 初始化多维导纳控制器
        # 配置：Z轴恒力控制（高刚度），XY轴柔顺（低刚度）
        self.admittance = MultiAxisAdmittanceController(
            mass=np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1]),
            damping=np.array([25, 25, 30, 2, 2, 2]),
            stiffness=np.array([10, 10, 1000, 10, 10, 10]),
            dt=0.002,  # 500Hz
            desired_wrench=np.array([0, 0, 5.0, 0, 0, 0]),  # Z轴期望力 5N
            a_max=np.array([200]*6),
            v_max=np.array([50]*6)
        )

        self.initial_pose = None

    def start(self):
        print("机械臂初始化完成，准备开始导纳控制...")
        state = self.robot.Get_Current_Arm_State()
        self.initial_pose = np.array(state[2])  # [x, y, z, rx, ry, rz]
        self.admittance.reset(position=np.zeros(6))

        try:
            self._control_loop()
        except KeyboardInterrupt:
            print("\n检测到中断，停止机械臂...")
            self.robot.Move_Stop_Cmd()
        finally:
            self.ser.close()
            self.robot.RM_API_UnInit()

    def _control_loop(self):
        print("进入导纳控制循环 (500Hz)")
        while True:
            t0 = time.time()

            # 1. 读取六维力
            force_data = read_six_force(self.ser)
            if not force_data:
                continue

            f_raw = np.array([force_data[0], force_data[1], force_data[2]])
            m_raw = np.array([force_data[3], force_data[4], force_data[5]])

            # 2. 获取当前姿态
            current_state = self.robot.Get_Current_Arm_State()
            current_pose = np.array(current_state[2])
            euler = current_pose[3:6]

            # 3. 重力补偿
            f_ext, m_ext = self.gravity_comp.compensate(f_raw, m_raw, euler)
            wrench_ext = np.concatenate([f_ext, m_ext])

            # 4. 导纳计算
            self.admittance.compute(wrench_ext)

            # 5. 计算目标位姿
            # 简化处理：直接将导纳累积位移叠加到初始位姿
            target_pose = self.initial_pose + self.admittance.position

            # 6. 发送指令
            self.robot.Force_Position_Move_Pose(
                target_pose.tolist(), 1, 0, 2, 0, False
            )

            # 7. 频率控制
            elapsed = time.time() - t0
            sleep_time = 0.002 - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

if __name__ == "__main__":
    CALIBRATION_DATA = [
        ([4.26, 5.45, 4.81], [-0.06, -0.01, -0.09], [180, 0, 0]),
        ([5.69, 6.88, -0.66], [0.21, 0.02, -0.11], [135, 90, 0]),
        ([3.63, 4.99, -3.25], [0.04, 0.02, -0.09], [0, 0, 135])
    ]

    ctrl = AdmittanceArmController(
        robot_ip="192.168.1.18",
        com_port="COM4",
        calibration_data=CALIBRATION_DATA
    )
    ctrl.start()