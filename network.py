import numpy as np


class FuzzyImpedanceController:
    def __init__(self):
        # 定义输入变量的模糊集参数
        self.df_fuzzy_sets = {
            'NL': {'center': -1.0, 'sigma': 0.5},  # 负大
            'NS': {'center': -0.5, 'sigma': 0.3},  # 负小
            'Z': {'center': 0.0, 'sigma': 0.3},  # 零
            'PS': {'center': 0.5, 'sigma': 0.3},  # 正小
            'PM': {'center': 0.75, 'sigma': 0.25},  # 正中
            'PL': {'center': 1.0, 'sigma': 0.5}  # 正大
        }

        self.dfd_fuzzy_sets = {
            'NL': {'center': -1.0, 'sigma': 0.5},  # 负大
            'NM': {'center': -0.5, 'sigma': 0.3},  # 负中
            'Z': {'center': 0.0, 'sigma': 0.3},  # 零
            'PS': {'center': 0.5, 'sigma': 0.3},
            'PM': {'center': 0.75, 'sigma': 0.25},  # 正中
            'PL': {'center': 1.0, 'sigma': 0.5}  # 正大
        }

        # 定义模糊规则
        self.rules = [
            # 当∆F为NL且∆Ḟ为NL时，输出VH
            {'df': 'NL', 'dfd': 'NL', 'output': 'VH'},
            # 当∆F为Z且∆Ḟ为Z时，输出M
            {'df': 'Z', 'dfd': 'Z', 'output': 'M'},
            # 当∆F为PL且∆Ḟ为PL时，输出VH
            {'df': 'PL', 'dfd': 'PL', 'output': 'VH'},
            # 当∆F为NS且∆Ḟ为PS时，输出M
            {'df': 'NS', 'dfd': 'PS', 'output': 'M'},
            # 当∆F为PM且∆Ḟ为NM时，输出L
            {'df': 'PM', 'dfd': 'NM', 'output': 'L'}
        ]

        # 定义输出值映射
        self.output_values = {
            'VH': 31.6,  # 很高
            'M': 15,  # 中等
            'L': 2  # 低
        }

    def gaussian(self, x, center, sigma):
        """计算高斯隶属函数"""
        return np.exp(-((x - center) ** 2) / (2 * sigma ** 2))

    def compute_membership(self, x, fuzzy_sets):
        """计算输入值对各个模糊集的隶属度"""
        memberships = {}
        for label, params in fuzzy_sets.items():
            memberships[label] = self.gaussian(x, params['center'], params['sigma'])
        return memberships

    def calculate_impedance(self, df, dfd):
        """计算阻抗系数"""
        # 计算输入值的隶属度
        df_memberships = self.compute_membership(df, self.df_fuzzy_sets)
        dfd_memberships = self.compute_membership(dfd, self.dfd_fuzzy_sets)

        total_activation = 0.0
        total_output = 0.0

        # 应用模糊规则
        for rule in self.rules:
            df_label = rule['df']
            dfd_label = rule['dfd']
            output_label = rule['output']

            # 获取隶属度
            df_activation = df_memberships[df_label]
            dfd_activation = dfd_memberships[dfd_label]

            # 使用min作为t-norm计算规则激活度
            activation = min(df_activation, dfd_activation)

            # 获取输出值
            output_value = self.output_values[output_label]

            # 累加激活度和输出贡献
            total_activation += activation
            total_output += activation * output_value

        # 处理除零情况
        if total_activation == 0:
            return 5.0  # 默认返回中等值

        # 去模糊化（加权平均法）
        return total_output / total_activation


# 示例用法
if __name__ == "__main__":
    controller = FuzzyImpedanceController()

    # 测试不同输入情况
    test_cases = [
        (-1.0, -1.0),  # 情况1：NL & NL -> VH
        (0.0, 0.0),  # 情况2：Z & Z -> M
        (1.0, 1.0),  # 情况3：PL & PL -> VH
        (-0.5, 0.25),  # 情况4：NS & PS -> M
        (0.75, -0.5)  # 情况5：PM & NM -> L
    ]

    for df, dfd in test_cases:
        impedance = controller.calculate_impedance(df, dfd)
        print(f"输入 ∆F={df:.2f}, ∆Ḟ={dfd:.2f} 时，阻抗系数 B = {impedance:.2f}")