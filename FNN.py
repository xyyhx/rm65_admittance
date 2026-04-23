import numpy as np
import matplotlib.pyplot as plt


# 隶属函数（采用正态分布形式）
def membership_function(x_i, c_ij, sigma_ij):
    """
    计算正态分布形式的隶属函数值
    :param x_i: 论域中的输入样本
    :param c_ij: 隶属函数的中心位置参数
    :param sigma_ij: 隶属函数的宽度尺度参数
    :return: 隶属度值
    """
    return np.exp(-((x_i - c_ij) ** 2 / sigma_ij ** 2))


# 输入层
class InputLayer:
    def __init__(self, input_vector):
        self.input_vector = input_vector
        self.n = len(input_vector)  # 输入向量的维数

    def forward_propagation(self):
        """
        将输入向量传递到下一层
        :return: 输入向量
        """
        return self.input_vector


# 隶属函数层（模糊化层）
class MembershipLayer:
    def __init__(self, input_layer):
        self.input_layer = input_layer
        # 各输入分量模糊分级数，按照给定的模糊子集定义，两个输入都有7个模糊分级
        self.m_list = [7, 7]
        self.N_2 = sum(self.m_list)  # 总节点数
        # 隶属函数参数设置，根据输入范围及常见的模糊划分来初始化中心位置参数和宽度尺度参数
        self.c_ij_list = [
            [-2.5, -1.5, -0.5, 0, 0.5, 1.5, 2.5],  # 对应∆F的中心位置参数
            [-0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6]  # 对应∆F ̇的中心位置参数
        ]
        self.sigma_ij_list = [
            [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5],  # 对应∆F的宽度尺度参数
            [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]  # 对应∆F ̇的宽度尺度参数
        ]

    def forward_propagation(self):
        """
        计算各输入分量属于模糊语言变量集合的隶属度函数值
        :return: 二维列表，每个元素对应一个输入分量的隶属度列表
        """
        input_vector = self.input_layer.forward_propagation()
        membership_values = []
        start_index = 0
        for i in range(self.input_layer.n):
            m_i = self.m_list[i]
            membership_i = []
            for j in range(m_i):
                x_i = input_vector[i]
                c_ij = self.c_ij_list[i][j]
                sigma_ij = self.sigma_ij_list[i][j]
                membership_value = membership_function(x_i, c_ij, sigma_ij)
                membership_i.append(membership_value)
            membership_values.append(membership_i)
            start_index += m_i
        return membership_values


# 模糊推理层
class FuzzyInferenceLayer:
    def __init__(self, membership_layer):
        self.membership_layer = membership_layer

    def forward_propagation(self):
        """
        根据隶属函数层的输出计算模糊规则的匹配度，按照具体模糊规则来确定
        :return: 规则匹配度列表
        """
        membership_values = self.membership_layer.forward_propagation()
        rule_matches = []
        # 规则1
        rule1_match = min(membership_values[0][0], membership_values[1][0])  # ∆F为NL且∆F ̇为NL
        rule_matches.append(rule1_match)
        # 规则2
        rule2_match = min(membership_values[0][3], membership_values[1][3])  # ∆F为Z且∆F ̇为Z
        rule_matches.append(rule2_match)
        # 规则3
        rule3_match = min(membership_values[0][6], membership_values[1][6])  # ∆F为PL且∆F ̇为PL
        rule_matches.append(rule3_match)
        # 规则4
        rule4_match = min(membership_values[0][2], membership_values[1][4])  # ∆F为NS且∆F ̇为PS
        rule_matches.append(rule4_match)
        # 规则5
        rule5_match = min(membership_values[0][5], membership_values[1][1])  # ∆F为PM且∆F ̇为NM
        rule_matches.append(rule5_match)

        # 其他未提及规则的匹配度设为0（可根据实际情况补充更多规则或调整此处逻辑）
        remaining_rules = 49 - len(rule_matches)
        rule_matches.extend([0] * remaining_rules)

        return rule_matches


# 归一化层
class NormalizationLayer:
    def __init__(self, fuzzy_inference_layer):
        self.fuzzy_inference_layer = fuzzy_inference_layer

    def forward_propagation(self):
        """
        对模糊推理层输出的规则匹配度进行归一化处理
        :return: 归一化后的规则匹配度列表
        """
        rule_matches = self.fuzzy_inference_layer.forward_propagation()
        sum_rule_matches = sum(rule_matches)
        normalized_rule_matches = [match / sum_rule_matches for match in rule_matches]
        return normalized_rule_matches


# 输出层
class OutputLayer:
    def __init__(self, normalization_layer):
        self.normalization_layer = normalization_layer
        # 根据输出模糊子集定义及范围，初始化规则对应的输出值（示例简单平均分配，实际需按具体情况确定）
        self.output_values = np.linspace(0, 31.6, 5)  # 对应B={VL,L,M,H,VH}

    def forward_propagation(self):
        """
        根据归一化后的规则匹配度计算最终的系统输出值（采用加权求和法）
        :return: 最终输出值
        """
        normalized_rule_matches = self.normalization_layer.forward_propagation()
        numerator = sum([match * value for match, value in zip(normalized_rule_matches, self.output_values)])
        denominator = sum(normalized_rule_matches)
        return numerator / denominator

if __name__ == '__main__':

    # 示例用法，设置符合给定范围的输入向量
    input_vector = np.array([1, -0.03])  # 对应∆F和∆F ̇的值，在各自给定范围内取值
    input_layer = InputLayer(input_vector)
    membership_layer = MembershipLayer(input_layer)
    fuzzy_inference_layer = FuzzyInferenceLayer(membership_layer)
    normalization_layer = NormalizationLayer(fuzzy_inference_layer)
    output_layer = OutputLayer(normalization_layer)

    result = output_layer.forward_propagation()
    print("最终输出值:", result)

    # # 可视化隶属函数（简单示例，可根据需要进一步完善绘图细节）
    # x_range_F = np.linspace(-3, 3, 100)
    # x_range_F_dot = np.linspace(-1, 1, 100)
    #
    # membership_layer_F = MembershipLayer(InputLayer(np.array([0, 0])))
    # membership_layer_F_dot = MembershipLayer(InputLayer(np.array([0, 0])))
    #
    # membership_values_F = membership_layer_F.forward_propagation()[0]
    # membership_values_F_dot = membership_layer_F_dot.forward_propagation()[0]
    #
    # plt.figure(figsize=(12, 6))
    # plt.subplot(1, 2, 1)
    # labels_F = ['NL', 'NM', 'NS', 'Z', 'PS', 'PM', 'PL']
    # for i in range(7):
    #     plt.plot(x_range_F, [membership_values_F[i] for _ in range(len(x_range_F))], label=labels_F[i])
    # plt.title("∆F的隶属函数")
    # plt.xlabel("∆F (N)")
    # plt.legend()
    #
    # plt.subplot(1, 2, 2)
    # labels_F_dot = ['NL', 'NM', 'NS', 'Z', 'PS', 'PM', 'PL']
    # for i in range(7):
    #     plt.plot(x_range_F_dot, [membership_values_F_dot[i] for _ in range(len(x_range_F_dot))], label=labels_F_dot[i])
    # plt.title("∆F ̇的隶属函数")
    # plt.xlabel("∆F ̇ (N/s)")
    # plt.legend()
    #
    # plt.show()