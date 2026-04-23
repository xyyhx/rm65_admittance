from module.ForceSensor import *
import time

# x = -3.745394419955427e-05
# y = -0.003968943700599212
# z = -0.0674816575210064
# lx = 0.39042464
# ly = 1.01914835
# lz = -3.23570831
# F_x0 = 3.7574750817543094
# F_y0 = 7.959914983207458
# F_z0 = 0.18682512155208286
# M_x0 = 0.07397420869405213
# M_y0 = 0.09436654767089936
# M_z0 = -0.1398060378448754


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


def six_force_read(serial):
    six_weight = bytearray([0x01, 0x04, 0x00, 0x54, 0x00, 0x0C, 0xB1, 0xDF])
    serial.write(six_weight)
    response = serial.read(29)

    if response:
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        ch1_fz = hex_to_ieee754(response[3:7].hex())  # 4-7
        ch2_fy = hex_to_ieee754(response[7:11].hex())  # 8-11
        ch3_fx = hex_to_ieee754(response[11:15].hex())  # 12-15
        ch4_Mz = hex_to_ieee754(response[15:19].hex())  # 16-19
        ch5_My = hex_to_ieee754(response[19:23].hex())
        ch6_Mx = hex_to_ieee754(response[23:27].hex())
        # time  fx fy fz Mx My Mz
        print(f"{now.strftime('%Y-%m-%d %H:%M:%S')}  {ch3_fx} {ch2_fy} {ch1_fz} {ch6_Mx} {ch5_My} {ch4_Mz}\n")

        filename = os.path.join("forcedata", f"{date_str}_data.txt")
        with open(filename, 'a') as file:
            file.write(f"{now.strftime('%Y-%m-%d %H:%M:%S')}  {ch3_fx} {ch2_fy} {ch1_fz} {ch6_Mx} {ch5_My} {ch4_Mz}\n")
        return ch3_fx, ch2_fy, ch1_fz, ch6_Mx, ch5_My, ch4_Mz
    else:
        print("没有接收到数据。")
        return None


if __name__ == '__main__':
    ser = open_serial_port(com_port="com4")
    while True:
        six_force_read(ser)
        time.sleep(0.5)
