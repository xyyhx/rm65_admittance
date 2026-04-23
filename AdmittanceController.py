# -*- coding: utf-8 -*-
import numpy as np


class AdmittanceController:
    """
    离散形式的导纳控制器

    离散方程：
        M · (ẍ) + B · (ẋ) + K · x = F_err
    其中 F_err = F_desired - F_input

    本实现返回每次迭代的位移增量 Δx = x(t+Δt) - x(t)
    """

    def __init__(self,
                 mass: float,
                 damping: float,
                 stiffness: float,
                 dt: float,
                 desired_force: float = 0.0,
                 a_max: float = None,
                 v_max: float = None):
        """
        :param mass: 虚拟质量 M (>0)
        :param damping: 虚拟阻尼 B
        :param stiffness: 虚拟刚度 K
        :param dt: 控制周期 Δt (s, >0)
        :param desired_force: 期望力/力矩 F_desired
        :param a_max: 加速度限幅 (可选)
        :param v_max: 速度限幅 (可选)
        """
        if dt <= 0 or mass <= 0:
            raise ValueError("dt 和 mass 必须为正值")

        self.mass = mass
        self.damping = damping
        self.stiffness = stiffness
        self.dt = dt
        self.desired_force = desired_force

        # 状态变量
        self.position = 0.0  # x(t)
        self.velocity = 0.0  # ẋ(t)

        # 可选限幅
        self.a_max = a_max
        self.v_max = v_max

    def compute(self, force_input: float) -> float:
        """
        根据输入 F(t) 计算本周期的位移增量 Δx。

        :param force_input: 当前测得的外力/力矩 F(t)
        :return: Δx = x(t+dt) - x(t)
        """
        # 1) 计算力误差
        f_err = self.desired_force - force_input

        # 2) 根据导纳方程求加速度
        a = (f_err
             - self.damping * self.velocity
             - self.stiffness * self.position
             ) / self.mass

        # 加速度限幅（如需）
        if self.a_max is not None:
            a = np.clip(a, -self.a_max, self.a_max)

        # 3) 更新速度
        self.velocity += a * self.dt

        # 速度限幅（如需）
        if self.v_max is not None:
            self.velocity = np.clip(self.velocity, -self.v_max, self.v_max)

        # 4) 更新位置并返回增量
        new_pos = self.position + self.velocity * self.dt
        delta = new_pos - self.position
        self.position = new_pos
        return delta

    def set_desired_force(self, desired_force: float):
        """动态调整期望力/力矩"""
        self.desired_force = desired_force

    def reset(self, position: float = 0.0, velocity: float = 0.0):
        """重置状态，在新一轮接触开始时调用"""
        self.position = position
        self.velocity = velocity


if __name__ == "__main__":
    # —— 示例用法 —— #
    adm = AdmittanceController(
        mass=0.25,
        damping=25,
        stiffness=100,
        dt=0.01,
        desired_force=6.0,
        a_max=50.0,  # 可选：加速度限幅
        v_max=100.0  # 可选：速度限幅
    )

    # 模拟一组外力输入
    inputs = [10.0, 8.0, 6.0, 4.0, 2.0, 0.0]

    print("Step\tF_in\tΔx")
    for i, f in enumerate(inputs, start=1):
        dx = adm.compute(f)
        print(f"{i}\t{f:.2f}\t{dx:.6f}")
