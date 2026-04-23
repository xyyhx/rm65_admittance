# -*- coding: utf-8 -*-
import numpy as np


class MultiAxisAdmittanceController:
    """
    多维导纳控制器（用于实现在x/y方向运动中保持z方向恒力，零化各方向力矩）

    离散导纳方程：
        M·ẍ + B·ẋ + K·x = F_err
        适用于多个自由度（如3维位置 + 3维姿态）
    """

    def __init__(self,
                 mass: np.ndarray,
                 damping: np.ndarray,
                 stiffness: np.ndarray,
                 dt: float,
                 desired_wrench: np.ndarray,  # [Fx, Fy, Fz, Tx, Ty, Tz]
                 a_max: np.ndarray = None,
                 v_max: np.ndarray = None):
        if dt <= 0 or np.any(mass <= 0):
            raise ValueError("dt 和 mass 中所有元素都必须为正值")

        self.mass = mass
        self.damping = damping
        self.stiffness = stiffness
        self.dt = dt
        self.desired_wrench = desired_wrench

        self.position = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.velocity = np.zeros(6)

        self.a_max = a_max if a_max is not None else np.full(6, np.inf)
        self.v_max = v_max if v_max is not None else np.full(6, np.inf)

    def compute(self, measured_wrench: np.ndarray) -> np.ndarray:
        """
        :param measured_wrench: 实测外部作用力/力矩 [Fx, Fy, Fz, Tx, Ty, Tz]
        :return: 6维位移增量 [Δx, Δy, Δz, Δroll, Δpitch, Δyaw]
        """
        f_err = self.desired_wrench - measured_wrench

        a = (f_err - self.damping * self.velocity - self.stiffness * self.position) / self.mass
        a = np.clip(a, -self.a_max, self.a_max)

        self.velocity += a * self.dt
        self.velocity = np.clip(self.velocity, -self.v_max, self.v_max)

        delta = self.velocity * self.dt
        self.position += delta

        return delta

    def set_desired_wrench(self, desired_wrench: np.ndarray):
        self.desired_wrench = desired_wrench

    def reset(self, position: np.ndarray = None, velocity: np.ndarray = None):
        self.position = position if position is not None else np.zeros(6)
        self.velocity = velocity if velocity is not None else np.zeros(6)


if __name__ == "__main__":
    ctrl = MultiAxisAdmittanceController(
        mass=np.array([0.25, 0.25, 0.25, 0.1, 0.1, 0.1]),
        damping=np.array([15, 15, 30, 1, 1, 1]),
        stiffness=np.array([0, 0, 1000, 10, 10, 10]),
        dt=0.01,
        desired_wrench=np.array([0, 0, 5.0, 0, 0, 0]),  # z方向恒力，其余为零
        a_max=np.array([50] * 6),
        v_max=np.array([100] * 6)
    )

    print("Step\tFx\tFy\tFz\tΔz")
    for i in range(6):
        force = np.array([0, 0, 10 - i, 0.1 - i / 100, 0.1 - i / 100, 0.1 - i / 100])
        delta = ctrl.compute(force)
        print(f"{i + 1}\t{force[0]:.1f}\t{force[1]:.1f}\t{force[2]:.1f}\t{delta[2]:.6f}")
