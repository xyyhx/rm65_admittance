class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        """
        初始化PID控制器
        :param kp: 比例增益
        :param ki: 积分增益
        :param kd: 微分增益
        :param setpoint: 目标值
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        # 初始化误差值
        self.previous_error = 0
        self.integral = 0

    def compute(self, current_value, dt):
        """
        计算PID输出
        :param current_value: 当前输入值（力矩误差）
        :param dt: 时间间隔
        :return: 输出值（转角变化量）
        """
        # 计算误差
        error = self.setpoint - current_value

        # 积分项累积
        self.integral += error * dt

        # 微分项计算
        derivative = (error - self.previous_error) / dt if dt > 0 else 0

        # PID控制公式
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # 更新前一误差值
        self.previous_error = error

        return output


# 示例用法
if __name__ == "__main__":
    # 创建一个PID控制器实例
    pid = PIDController(kp=0.001, ki=0.01, kd=0.005, setpoint=10)

    # 模拟输入力矩误差
    current_torque = 8
    dt = 0.1  # 时间间隔（假设0.1秒）

    # 计算输出转角变化量
    angle_change = pid.compute(current_torque, dt)
    print(f"转角变化量: {angle_change:.4f}")
