import numpy as np

class GaussianMF:
    """Gaussian membership function"""
    def __init__(self, mean, sigma):
        self.mean = mean
        self.sigma = sigma

    def __call__(self, x):
        return np.exp(-0.5 * ((x - self.mean) / self.sigma) ** 2)

class FuzzyNeuralNetwork:
    """
    Fuzzy Neural Network for impedance tuning using fuzzy inference with Gaussian membership functions.
    Inputs:
      - error: contact force error (ΔF)
      - error_dot: rate of change of contact force error (ΔF_dot)
    Output:
      - impedance coefficient B
    """
    def __init__(self):
        # Define Gaussian MFs for error (ΔF)
        self.error_mfs = {
            'NL': GaussianMF(mean=-2.0, sigma=1.0),  # Negative Large
            'NS': GaussianMF(mean=-1.0, sigma=0.5),  # Negative Small
            'Z':  GaussianMF(mean=0.0, sigma=0.2),   # Zero
            'PM': GaussianMF(mean=1.0, sigma=0.5),   # Positive Medium
            'PL': GaussianMF(mean=2.0, sigma=1.0)    # Positive Large
        }
        # Define Gaussian MFs for error_dot (ΔF_dot)
        self.error_dot_mfs = {
            'NL': GaussianMF(mean=-2.0, sigma=1.0),
            'PS': GaussianMF(mean=1.0, sigma=0.5),
            'Z':  GaussianMF(mean=0.0, sigma=0.2),
            'NM': GaussianMF(mean=-1.0, sigma=0.5),
            'PL': GaussianMF(mean=2.0, sigma=1.0)
        }
        # Define rule consequents mapping to crisp impedance values
        # L (Low), M (Medium), VH (Very High)
        self.output_values = {
            'L': 3,
            'M': 15,
            'VH': 31.6
        }
        # Define fuzzy rules: (error_term, error_dot_term) -> output_label
        self.rules = [
            (('NL', 'NL'), 'VH'),
            (('Z',  'Z' ), 'M' ),
            (('PL', 'PL'), 'VH'),
            (('NS', 'PS'), 'M' ),
            (('PM', 'NM'), 'L' )
        ]

    def compute_impedance(self, error, error_dot):
        # Fuzzify inputs
        e_degrees = {term: mf(error) for term, mf in self.error_mfs.items()}
        ed_degrees = {term: mf(error_dot) for term, mf in self.error_dot_mfs.items()}

        # Evaluate rules and accumulate weighted outputs
        numerator = 0.0
        denominator = 0.0
        for (e_term, ed_term), out_label in self.rules:
            # rule firing strength
            strength = min(e_degrees.get(e_term, 0.0), ed_degrees.get(ed_term, 0.0))
            output_value = self.output_values[out_label]
            numerator += strength * output_value
            denominator += strength

        # Defuzzify: weighted average
        if denominator == 0:
            # no rule fired: default to medium
            return self.output_values['M']
        return numerator / denominator

if __name__ == '__main__':
    # 示例
    fnn = FuzzyNeuralNetwork()
    print(fnn.compute_impedance(-0.5, 0.5))
    # errors = [-3, -0.5, 0, 0.5, 3]
    # error_dots = [-3, -0.5, 0, 0.5, 3]
    # for e in errors:
    #     for ed in error_dots:
    #         B = fnn.compute_impedance(e, ed)
    #         print(f"error={e:.2f}, error_dot={ed:.2f} => B={B:.2f}")
