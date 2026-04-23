# -*- coding: utf-8 -*-
import math
import numpy as np
from typing import List, Tuple


class GravityCompensator:
    """
    重力补偿器。通过多组标定数据求解传感器质心位置及零偏。
    """

    def __init__(self, calibration_data: List[Tuple[List[float], List[float], List[float]]]):
        self.M = np.empty((0, 3))
        self.F = np.empty((0, 6))
        self.f = np.empty((0, 3))
        self.R = np.empty((0, 6))

        self.x = self.y = self.z = 0.0
        self.k1 = self.k2 = self.k3 = 0.0
        self.U = self.V = self.g = 0.0
        self.F_x0 = self.F_y0 = self.F_z0 = 0.0
        self.M_x0 = self.M_y0 = self.M_z0 = 0.0

        for force, torque, euler in calibration_data:
            self._update_F(force)
            self._update_M(torque)
        self._solve_A()

        for force, _, euler in calibration_data:
            self._update_f(force)
            self._update_R(euler)
        self._solve_B()

    def compensate(
        self, force: List[float], torque: List[float], euler: List[float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        f_ex = self._solve_force(force, euler)
        t_ex = self._solve_torque(torque, euler)
        return f_ex, t_ex

    def _update_M(self, torque: List[float]) -> None:
        m_vec = np.array(torque).reshape(1, 3)
        self.M = np.vstack((self.M, m_vec))

    def _update_F(self, force: List[float]) -> None:
        Fx, Fy, Fz = force
        cross = np.array([
            [0, Fz, -Fy, 1, 0, 0],
            [-Fz, 0, Fx, 0, 1, 0],
            [Fy, -Fx, 0, 0, 0, 1]
        ])
        self.F = np.vstack((self.F, cross))

    def _solve_A(self) -> None:
        A = np.linalg.lstsq(self.F, self.M, rcond=None)[0].flatten()
        self.x, self.y, self.z, self.k1, self.k2, self.k3 = A

    def _update_f(self, force: List[float]) -> None:
        f_vec = np.array(force).reshape(1, 3)
        self.f = np.vstack((self.f, f_vec))

    def _update_R(self, euler: List[float]) -> None:
        R_mat = self._euler_to_rot(euler).T
        R_full = np.hstack((R_mat, np.eye(3)))
        self.R = np.vstack((self.R, R_full))

    def _solve_B(self) -> None:
        B = np.linalg.lstsq(self.R, self.f, rcond=None)[0].flatten()
        lx, ly, lz = B[:3]
        self.g = math.hypot(lx, ly, lz)
        self.U = math.asin(-ly / self.g) if self.g != 0 else 0.0
        self.V = math.atan2(-lx, lz)
        self.F_x0, self.F_y0, self.F_z0 = B[3:]

        self.M_x0 = self.k1 - self.F_y0 * self.z + self.F_z0 * self.y
        self.M_y0 = self.k2 - self.F_z0 * self.x + self.F_x0 * self.z
        self.M_z0 = self.k3 - self.F_x0 * self.y + self.F_y0 * self.x

    def _solve_force(self, force: List[float], euler: List[float]) -> np.ndarray:
        Fi = np.array(force).reshape(3, 1)
        bias = np.array([
            math.cos(self.U) * math.sin(self.V) * self.g,
            -math.sin(self.U) * self.g,
            -math.cos(self.U) * math.cos(self.V) * self.g
        ]).reshape(3, 1)
        my_f = np.vstack((bias, [[self.F_x0], [self.F_y0], [self.F_z0]]))
        R = self._euler_to_rot(euler).T
        block = np.hstack((R, np.eye(3)))
        return (Fi - block @ my_f).flatten()

    def _solve_torque(self, torque: List[float], euler: List[float]) -> np.ndarray:
        Ti = np.array(torque).reshape(3, 1)
        zero_t = np.array([self.M_x0, self.M_y0, self.M_z0]).reshape(3, 1)
        Gp = np.array([
            [0, -self.z, self.y],
            [self.z, 0, -self.x],
            [-self.y, self.x, 0]
        ])
        grav = np.array([
            math.cos(self.U) * math.sin(self.V) * self.g,
            -math.sin(self.U) * self.g,
            -math.cos(self.U) * math.cos(self.V) * self.g
        ]).reshape(3, 1)
        R = self._euler_to_rot(euler).T
        return (Ti - zero_t - (Gp @ R @ grav)).flatten()

    @staticmethod
    def _euler_to_rot(euler: List[float]) -> np.ndarray:
        th = np.radians(euler)
        Rx = np.array([[1, 0, 0], [0, math.cos(th[0]), -math.sin(th[0])], [0, math.sin(th[0]), math.cos(th[0])]])
        Ry = np.array([[math.cos(th[1]), 0, math.sin(th[1])], [0, 1, 0], [-math.sin(th[1]), 0, math.cos(th[1])]])
        Rz = np.array([[math.cos(th[2]), -math.sin(th[2]), 0], [math.sin(th[2]), math.cos(th[2]), 0], [0, 0, 1]])
        return Rx @ Ry @ Rz


if __name__ == "__main__":
    calib = [
        ([4.2643, 5.4578, 4.8185], [-0.0669, -0.0125, -0.0993], [180, 0, 0]),
        ([5.6904, 6.8851, -0.6655], [0.2174, 0.0257, -0.1194], [135, 90, 0]),
        ([3.631, 4.9923, -3.2577], [0.0449, 0.0207, -0.0923], [0, 0, 135])
    ]
    comp = GravityCompensator(calib)
    f_ex, m_ex = comp.compensate([4.2643, 5.4578, 4.8185], [-0.0669, -0.0125, -0.0993], [180, 0, 0])
    print("补偿力:", f_ex)
    print("补偿力矩:", m_ex)
