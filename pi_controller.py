import time


class PIController:
    def __init__(self, kp, ki, tolerance=0.01, dt=0.01, integral_limit=1.0):
        """
        初始化 PI 控制器参数。
        :param kp: 比例增益
        :param ki: 积分增益
        :param tolerance: 力矩差值容忍范围
        :param dt: 控制周期（秒）
        """
        self.kp = kp
        self.ki = ki
        self.tolerance = tolerance
        self.dt = dt
        self.integral = 0.0  # 积分项初值
        self.integral_limit = integral_limit

    def control(self, torque_difference):
        """
        计算调整机械臂角度的增量。
        :param torque_difference: 当前力矩差值
        :return: 角度调整量
        """
        self.integral += torque_difference * self.dt  # 更新积分项
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        return -self.kp * torque_difference - self.ki * self.integral

    def run(self, get_torque_difference, adjust_arm_angle):
        """
        控制器主循环。
        :param get_torque_difference: 函数，返回当前力矩差值
        :param adjust_arm_angle: 函数，调整机械臂角度
        """
        while True:
            torque_difference = get_torque_difference()
            print(f"当前力矩差值: {torque_difference}")

            if abs(torque_difference) <= self.tolerance:
                print("力矩差值已接近零，控制完成。")
                break

            angle_adjustment = self.control(torque_difference)
            adjust_arm_angle(angle_adjustment)
            print(f"调整角度: {angle_adjustment}")

            time.sleep(self.dt)  # 控制器循环周期


# 示例：模拟机械臂系统
class MockArmSystem:
    def __init__(self):
        self.torque_difference = 0.2  # 初始力矩差值
        self.angle = 0.0  # 机械臂当前角度

    def get_torque_difference(self):
        """
        模拟获取当前力矩差值。
        """
        return self.torque_difference

    def adjust_arm_angle(self, angle_adjustment):
        """
        模拟调整机械臂的角度。
        :param angle_adjustment: 调整角度
        """
        self.angle += angle_adjustment
        # 模拟力矩差值随角度调整减小
        self.torque_difference += angle_adjustment * 0.5


if __name__ == '__main__':
    # 创建模拟机械臂系统
    mock_arm_system = MockArmSystem()

    # 创建 PI 控制器
    pi_controller = PIController(kp=0.005, ki=0.01, tolerance=0.01, dt=0.1, integral_limit=0.5)

    # 启动控制器主循环
    pi_controller.run(mock_arm_system.get_torque_difference, mock_arm_system.adjust_arm_angle)
