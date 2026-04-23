# encode UFS-8
class AdmittanceController:
    def __init__(self, M_mass, B_damping, K_stiffness, dt, desired_force=0.0):
        """
        初始化导纳控制器
        :param mass: 虚拟质量 M
        :param damping: 虚拟阻尼 B
        :param stiffness: 虚拟刚度 K
        :param dt: 时间步长 Δt
        :param desired_force: 期望力
        """
        if dt <= 0:
            raise ValueError("时间步长 dt 必须为正值")

        self.mass = M_mass
        self.damping = B_damping
        self.stiffness = K_stiffness
        self.dt = dt
        self.desired_force = desired_force

        # 初始化状态变量
        self.position = 0.0  # x(t)
        self.velocity = 0.0  # x_dot(t)

    def compute(self, force_input):
        """
        根据输入的力计算位移变化量
        :param force_input: 输入力 F(t)
        :return: 当前位移 x(t+1)
        """
        # 计算力误差
        delta_force = self.desired_force - force_input

        # 计算加速度 (F_error - B * v - K * x) / M
        acceleration = (delta_force - self.damping * self.velocity - self.stiffness * self.position) / self.mass

        # 更新速度和位移
        self.velocity += acceleration * self.dt
        self.position += self.velocity * self.dt

        return self.position

    def set_desired_force(self, desired_force):
        """动态调整期望力"""
        self.desired_force = desired_force

    def set_damping(self, B_impedance):
        """动态调整阻抗参数"""
        self.damping = B_impedance


# 示例用法
if __name__ == "__main__":
    # 创建导纳控制器实例
    admittance = AdmittanceController(M_mass=0.25, B_damping=25, K_stiffness=1000, dt=0.01, desired_force=6)

    # 模拟外力输入
    force_inputs = [10.0, 8.0, 6.0, 4.0, 2.0, 0.0]  # 假设一组力输入

    print("时间步\t外力\t位移")
    for i, force in enumerate(force_inputs):
        displacement = admittance.compute(force)
        print(f"{i+1}\t{force:.2f}\t{displacement:.4f}")
