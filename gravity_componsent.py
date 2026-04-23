# import math
# import numpy as np
#
# x = 0.008517208526496888
# y = -0.00664022586205394
# z = -0.029287053409388045
# lx = -0.1952101
# ly = -0.65303065
# lz = -3.68483571
#
#
#
# def Gravity_componsent(theta):
#     # 绕X轴旋转
#     R_x = np.array([[1, 0, 0],
#                     [0, math.cos(theta[0]), -math.sin(theta[0])],
#                     [0, math.sin(theta[0]), math.cos(theta[0])]])
#
#     # 绕Y轴旋转
#     R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
#                     [0, 1, 0],
#                     [-math.sin(theta[1]), 0, math.cos(theta[1])]])
#
#     # 绕Z轴旋转
#     R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
#                     [math.sin(theta[2]), math.cos(theta[2]), 0],
#                     [0, 0, 1]])
#
#     # 旋转矩阵 R_s1 = R_Z * R_Y * R_X
#     R_s1 = np.dot(R_z, np.dot(R_y, R_x))
#     # 转置矩阵
#     R_1S = np.transpose(R_s1)
#
#     temp = np.array([[lx], [ly], [lz]])
#     g = np.dot(R_1S, temp)
#     return g
