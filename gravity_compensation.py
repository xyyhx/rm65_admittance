# -*- coding: utf-8 -*-
import numpy as np
import math


class GravityCompensator:
    """
    重力补偿模块化类。
    初始化时输入校准数据列表（每条包含力、力矩、欧拉角），自动计算参数。
    调用 `compensate` 方法时，输入当前力、力矩、欧拉角，返回补偿后的力和力矩。
    """

    def __init__(self, calibration_sets):
        # 初始化空矩阵
        self.M = np.empty((0, 0))
        self.F = np.empty((0, 0))
        self.f = np.empty((0, 0))
        self.R = np.empty((0, 0))
        # 参数占位
        self.x = self.y = self.z = 0
        self.k1 = self.k2 = self.k3 = 0
        self.U = self.V = self.g = 0
        self.F_x0 = self.F_y0 = self.F_z0 = 0
        self.M_x0 = self.M_y0 = self.M_z0 = 0
        # 校准：获取三组数据求解 A、B
        for force, torque, euler in calibration_sets:
            self._update_F(force)
            self._update_M(torque)
        self._solve_A()
        for force, _, euler in calibration_sets:
            self._update_f(force)
            self._update_R(euler)
        self._solve_B()

    def compensate(self, force_data, torque_data, euler_data):
        """
        返回补偿后的接触力和接触力矩。
        :param force_data: 长度3列表 [Fx, Fy, Fz]
        :param torque_data: 长度3列表 [Mx, My, Mz]
        :param euler_data: 长度3列表 [roll, pitch, yaw]（度）
        :return: (force_ex, torque_ex)，均为长度3的 numpy 数组
        """
        force_ex = self._solve_force(force_data, euler_data)
        torque_ex = self._solve_torque(torque_data, euler_data)
        return force_ex, torque_ex

    def _update_M(self, torque_data):
        M_vec = np.array(torque_data).reshape(3, 1)
        self.M = np.vstack((self.M, M_vec)) if self.M.size else M_vec

    def _update_F(self, force_data):
        Fx, Fy, Fz = force_data
        cross = np.array([[0, Fz, -Fy, 1, 0, 0],
                          [-Fz, 0, Fx, 0, 1, 0],
                          [Fy, -Fx, 0, 0, 0, 1]])
        self.F = np.vstack((self.F, cross)) if self.F.size else cross

    def _solve_A(self):
        A = np.linalg.lstsq(self.F, self.M, rcond=None)[0]
        self.x, self.y, self.z, self.k1, self.k2, self.k3 = A.flatten()

    def _update_f(self, force_data):
        f_vec = np.array(force_data).reshape(3, 1)
        self.f = np.vstack((self.f, f_vec)) if self.f.size else f_vec

    def _update_R(self, euler_data):
        R_mat = self._euler_to_rot(euler_data)
        alpha = 0.0  # 传感器安装角度，如有偏差可修改
        R_alpha = np.array([[math.cos(alpha), -math.sin(alpha), 0],
                            [math.sin(alpha), math.cos(alpha), 0],
                            [0, 0, 1]])
        R_full = R_alpha @ R_mat.T
        block = np.hstack((R_full, np.eye(3)))
        self.R = np.vstack((self.R, block)) if self.R.size else block

    def _solve_B(self):
        B = np.linalg.lstsq(self.R, self.f, rcond=None)[0]
        lx, ly, lz, Fx0, Fy0, Fz0 = B.flatten()
        self.g = math.sqrt(lx ** 2 + ly ** 2 + lz ** 2)
        self.U = math.asin(-ly / self.g)
        self.V = math.atan(-lx / lz)
        self.F_x0, self.F_y0, self.F_z0 = Fx0, Fy0, Fz0
        # 初始力矩补偿值
        self.M_x0 = self.k1 - Fy0 * self.z + Fz0 * self.y
        self.M_y0 = self.k2 - Fz0 * self.x + Fx0 * self.z
        self.M_z0 = self.k3 - Fx0 * self.y + Fy0 * self.x

    def _solve_force(self, force_data, euler_data):
        Fi = np.array(force_data).reshape(3, 1)
        bias = np.array([math.cos(self.U) * math.sin(self.V) * self.g,
                         -math.sin(self.U) * self.g,
                         -math.cos(self.U) * math.cos(self.V) * self.g]).reshape(3, 1)
        my_f = np.vstack((bias, [[self.F_x0], [self.F_y0], [self.F_z0]]))
        R = self._euler_to_rot(euler_data).T
        block = np.hstack((R, np.eye(3)))
        force_ex = Fi - block @ my_f
        return force_ex.flatten()

    def _solve_torque(self, torque_data, euler_data):
        Ti = np.array(torque_data).reshape(3, 1)
        zero_t = np.array([self.M_x0, self.M_y0, self.M_z0]).reshape(3, 1)
        Gp = np.array([[0, -self.z, self.y],
                       [self.z, 0, -self.x],
                       [-self.y, self.x, 0]])
        grav = np.array([math.cos(self.U) * math.sin(self.V) * self.g,
                         -math.sin(self.U) * self.g,
                         -math.cos(self.U) * math.cos(self.V) * self.g]).reshape(3, 1)
        R = self._euler_to_rot(euler_data).T
        torque_ex = Ti - zero_t - (Gp @ R @ grav)
        return torque_ex.flatten()

    @staticmethod
    def _euler_to_rot(euler):
        # 输入角度（度），返回旋转矩阵
        th = np.radians(euler)
        Rx = np.array([[1, 0, 0], [0, math.cos(th[0]), -math.sin(th[0])], [0, math.sin(th[0]), math.cos(th[0])]])
        Ry = np.array([[math.cos(th[1]), 0, math.sin(th[1])], [0, 1, 0], [-math.sin(th[1]), 0, math.cos(th[1])]])
        Rz = np.array([[math.cos(th[2]), -math.sin(th[2]), 0], [math.sin(th[2]), math.cos(th[2]), 0], [0, 0, 1]])
        return Rx @ Ry @ Rz

if __name__ == '__main__':
    # 示例用法：
    calibration = [
        ([4.2643, 5.4578, 4.8185], [-0.0669, -0.0125, -0.0993], [180, 0, 0]),
        ([5.6904, 6.8851, -0.6655], [0.2174, 0.0257, -0.1194], [135, 90, 0]),
        ([3.631, 4.9923, -3.2577], [0.0449, 0.0207, -0.0923], [0, 0, 135])
    ]
    compensator = GravityCompensator(calibration)
    f_ex, m_ex = compensator.compensate([4.2643, 5.4578, 4.8185], [-0.0669, -0.0125, -0.0993], [180, 0, 0])
    print('补偿力:', f_ex)
    print('补偿力矩:', m_ex)
