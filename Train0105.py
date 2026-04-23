import numpy as np
import matplotlib.pyplot as plt

# 隶属函数定义（高斯隶属函数）
def gauss_mf(x, c, sigma):
    return np.exp(-0.5 * ((x - c) / sigma) ** 2)

# 定义力误差和力误差变化率的隶属函数中心
force_error_centers = [-3, -2, -1, 0, 1, 2, 3]
force_error_rate_centers = [-2, -4/3, -2/3, 0, 2/3, 4/3, 2]

# 定义隶属函数的标准差
sigma_force_error = 0.425  # 力误差的标准差
sigma_force_error_rate = 0.285  # 力误差变化率的标准差
sigma_output = 3.37  # 输出B的标准差

# 定义输出B的隶属函数中心
output_centers = [0, 7.9, 15.8, 23.7, 31.6]

# 模糊规则类
class FuzzyRule:
    def __init__(self, centers, sigmas, output_center):
        self.centers = centers
        self.sigmas = sigmas
        self.output_center = output_center

    # 计算规则的隶属度
    def compute_membership(self, inputs):
        membership = 1.0
        for i, x in enumerate(inputs):
            membership *= gauss_mf(x, self.centers[i], self.sigmas[i])
        return membership

# 模糊神经网络类
class FuzzyNeuralNetwork:
    def __init__(self, rules, weights=None, learning_rate=0.01):
        self.rules = rules
        if weights is None:
            self.weights = np.random.rand(len(rules))
        else:
            self.weights = weights
        self.learning_rate = learning_rate

    # 前向传播
    def forward(self, inputs):
        memberships = np.array([rule.compute_membership(inputs) for rule in self.rules])
        weighted_sum = np.dot(self.weights, memberships)
        # 计算输出B的值
        output = np.sum(memberships * np.array([rule.output_center for rule in self.rules])) / np.sum(memberships)
        return output, memberships

    # 反向传播（梯度下降更新权重）
    def backward(self, memberships, error):
        for i in range(len(self.weights)):
            self.weights[i] -= self.learning_rate * error * memberships[i]

    # 保存权重
    def save_weights(self, filename):
        np.save(filename, self.weights)

    # 加载权重
    def load_weights(self, filename):
        self.weights = np.load(filename)

if __name__ == '__main__':

    # 定义模糊规则
    rule1 = FuzzyRule(centers=[force_error_centers[0], force_error_rate_centers[0]], sigmas=[sigma_force_error, sigma_force_error_rate], output_center=output_centers[4])  # VH
    rule2 = FuzzyRule(centers=[force_error_centers[3], force_error_rate_centers[3]], sigmas=[sigma_force_error, sigma_force_error_rate], output_center=output_centers[2])  # M
    rule3 = FuzzyRule(centers=[force_error_centers[6], force_error_rate_centers[6]], sigmas=[sigma_force_error, sigma_force_error_rate], output_center=output_centers[4])  # VH
    rule4 = FuzzyRule(centers=[force_error_centers[2], force_error_rate_centers[4]], sigmas=[sigma_force_error, sigma_force_error_rate], output_center=output_centers[2])  # M
    rule5 = FuzzyRule(centers=[force_error_centers[4], force_error_rate_centers[2]], sigmas=[sigma_force_error, sigma_force_error_rate], output_center=output_centers[1])  # L

    # 创建模糊神经网络
    fnn = FuzzyNeuralNetwork([rule1, rule2, rule3, rule4, rule5])

    # 训练数据（输入：力误差和力误差变化率，目标输出：阻抗参数）
    training_data = [
        ([-2.5, -1.8], 31.6),  # 对应规则1
        ([0.0, 0.0], 15.8),    # 对应规则2
        ([2.5, 1.8], 31.6),    # 对应规则3
        ([-0.5, 0.5], 15.8),    # 对应规则4
        ([1.0, -1.0], 7.9)      # 对应规则5
    ]

    # 训练过程
    for epoch in range(100):
        total_loss = 0
        for inputs, target in training_data:
            output, memberships = fnn.forward(inputs)
            error = output - target
            fnn.backward(memberships, error)
            total_loss += error ** 2
        if epoch % 10 == 0:
            print(f"Epoch {epoch}, Loss: {total_loss:.4f}")

    # 保存训练好的权重
    fnn.save_weights('fnn_weights.npy')

    # 测试
    test_input = [0.1, -0.1]
    output, _ = fnn.forward(test_input)
    print("测试输出阻抗参数:", output)

    # 记录损失
    losses = []

    for epoch in range(100):
        total_loss = 0
        for inputs, target in training_data:
            output, memberships = fnn.forward(inputs)
            error = output - target
            fnn.backward(memberships, error)
            total_loss += error ** 2
        losses.append(total_loss)

    # 绘制损失曲线
    plt.plot(losses)
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.title('Training Loss Curve')
    plt.show()
