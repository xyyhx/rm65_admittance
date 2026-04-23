import numpy as np
import math


class GravityCompensation:
    def __init__(self):
        # 初始化矩阵和变量
        self.M = np.empty((0, 3))  # 力矩矩阵
        self.F = np.empty((0, 6))  # 力数据矩阵
        self.f = np.empty((0, 3))  # 力数据向量
        self.R = np.empty((0, 6))  # 旋转矩阵

        self.x = self.y = self.z = 0
        self.k1 = self.k2 = self.k3 = 0
        self.U = self.V = self.g = 0

        self.F_x0 = self.F_y0 = self.F_z0 = 0
        self.M_x0 = self.M_y0 = self.M_z0 = 0

    def update_matrix(self, matrix, new_data, is_force=False):
        """
        通用矩阵更新方法。
        :param matrix: 要更新的矩阵（self.M, self.F 等）
        :param new_data: 新的数据
        :param is_force: 是否为力矩阵，影响数据结构
        :return: 更新后的矩阵
        """
        if is_force:
            formatted_data = np.array([[0, new_data[2], -new_data[1], 1, 0, 0],
                                       [-new_data[2], 0, new_data[0], 0, 1, 0],
                                       [new_data[1], -new_data[0], 0, 0, 0, 1]])
        else:
            formatted_data = np.array(new_data).reshape(-1, 3)

        return np.vstack((matrix, formatted_data)) if matrix.size else formatted_data

    def update_m(self, torque_data):
        self.M = self.update_matrix(self.M, torque_data)
        print("Updated M shape:", self.M.shape)

    def update_f(self, force_data):
        self.F = self.update_matrix(self.F, force_data, is_force=True)
        print("Updated F shape:", self.F.shape)


    def update_r(self, euler_data):
        R_array = self.euler_angles_to_rotation_matrix(euler_data)
        R_alpha = np.eye(3)  # 力传感器到末端的旋转矩阵，默认为单位矩阵
        R_combined = np.dot(R_alpha, R_array.T)

        R_expanded = np.hstack((R_combined, np.eye(3)))
        self.R = np.vstack((self.R, R_expanded)) if self.R.size else R_expanded

    def solve_a(self):
        print("F shape:", self.F.shape)
        print("M shape:", self.M.shape)
        if self.F.shape[0] != self.M.shape[0]:
            raise ValueError("Row count of F and M must match.")
        A = np.linalg.lstsq(self.F, self.M, rcond=None)[0]
        self.x, self.y, self.z, self.k1, self.k2, self.k3 = A.flatten()
        self._print_parameters("Solve_A", x=self.x, y=self.y, z=self.z, k1=self.k1, k2=self.k2, k3=self.k3)

    def solve_b(self):
        B = np.linalg.lstsq(self.R, self.f, rcond=None)[0]
        self.lx, self.ly, self.lz = B[:3].flatten()
        self.g = np.linalg.norm(B[:3])
        self.U = math.asin(-B[1] / self.g)
        self.V = math.atan2(-B[0], B[2])
        self.F_x0, self.F_y0, self.F_z0 = B[3:].flatten()
        self.M_x0 = self.k1 - self.F_y0 * self.z + self.F_z0 * self.y
        self.M_y0 = self.k2 - self.F_z0 * self.x + self.F_x0 * self.z
        self.M_z0 = self.k3 - self.F_x0 * self.y + self.F_y0 * self.x

        self._print_parameters("Solve_B", g=self.g / 9.81, U=self.U * 180 / math.pi, V=self.V * 180 / math.pi,
                               F_x0=self.F_x0, F_y0=self.F_y0, F_z0=self.F_z0,
                               M_x0=self.M_x0, M_y0=self.M_y0, M_z0=self.M_z0)

    def solve_force(self, force_data, euler_data):
        Force_input = np.array(force_data).reshape(-1, 1)
        my_f = np.array([math.cos(self.U) * math.sin(self.V) * self.g, -math.sin(self.U) * self.g,
                         -math.cos(self.U) * math.cos(self.V) * self.g, self.F_x0, self.F_y0, self.F_z0]).reshape(-1, 1)

        R_array = self.euler_angles_to_rotation_matrix(euler_data)
        R_expanded = np.hstack((R_array.T, np.eye(3)))
        Force_ex = Force_input - np.dot(R_expanded, my_f)

        print('接触力：', Force_ex.T)

    def solve_torque(self, torque_data, euler_data):
        Torque_input = np.array(torque_data).reshape(-1, 1)
        Gravity_input = np.array([math.cos(self.U) * math.sin(self.V) * self.g, -math.sin(self.U) * self.g,
                                   -math.cos(self.U) * math.cos(self.V) * self.g]).reshape(-1, 1)

        Gravity_param = np.array([[0, -self.z, self.y],
                                   [self.z, 0, -self.x],
                                   [-self.y, self.x, 0]])

        R_array = self.euler_angles_to_rotation_matrix(euler_data)
        Torque_zero = np.array([self.M_x0, self.M_y0, self.M_z0]).reshape(-1, 1)
        Torque_ex = Torque_input - Torque_zero - np.dot(np.dot(Gravity_param, R_array.T), Gravity_input)

        print('接触力矩：', Torque_ex.T)

    @staticmethod
    def euler_angles_to_rotation_matrix(theta):
        theta = [math.radians(i) for i in theta]
        R_x = np.array([[1, 0, 0],
                        [0, math.cos(theta[0]), -math.sin(theta[0])],
                        [0, math.sin(theta[0]), math.cos(theta[0])]])

        R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                        [0, 1, 0],
                        [-math.sin(theta[1]), 0, math.cos(theta[1])]])

        R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                        [math.sin(theta[2]), math.cos(theta[2]), 0],
                        [0, 0, 1]])

        return np.dot(R_x, np.dot(R_y, R_z))

    @staticmethod
    def _print_parameters(label, **kwargs):
        print(f"[{label}] Parameters:")
        for key, value in kwargs.items():
            print(f"  {key} = {value}")



def main():
    force_data = [4.8685, 6.6535, 4.0231]
    torque_data = [-0.0501, -0.0129, -0.1285]
    euler_data = [180, 0, 0]

    compensation = GravityCompensation()

    compensation.update_f(force_data)
    compensation.update_m(torque_data)
    compensation.update_r(euler_data)

    compensation.solve_a()
    compensation.solve_b()
    compensation.solve_force(force_data, euler_data)
    compensation.solve_torque(torque_data, euler_data)


if __name__ == '__main__':
    main()
