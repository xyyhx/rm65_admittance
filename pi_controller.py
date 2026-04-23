import time
from typing import Callable


class PIController:
    def __init__(
        self,
        kp: float,
        ki: float,
        tolerance: float = 0.01,
        dt: float = 0.01,
        integral_limit: float = 1.0,
    ):
        self.kp = kp
        self.ki = ki
        self.tolerance = tolerance
        self.dt = dt
        self.integral_limit = integral_limit
        self.integral = 0.0

    def control(self, error: float) -> float:
        self.integral += error * self.dt
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        return -self.kp * error - self.ki * self.integral

    def run(
        self,
        get_error: Callable[[], float],
        apply_control: Callable[[float], None],
        max_steps: int = None,
    ) -> None:
        step = 0
        while True:
            error = get_error()
            print(f"当前误差: {error:.4f}")

            if abs(error) <= self.tolerance:
                print("误差已收敛，控制完成。")
                break

            output = self.control(error)
            apply_control(output)
            print(f"输出控制量: {output:.4f}")

            time.sleep(self.dt)
            step += 1
            if max_steps and step >= max_steps:
                print("达到最大步数，退出循环。")
                break


if __name__ == "__main__":

    class MockSystem:
        def __init__(self):
            self.error = 0.2
            self.state = 0.0

        def get_error(self) -> float:
            return self.error

        def apply_control(self, u: float) -> None:
            self.state += u
            self.error += u * 0.5

    sys = MockSystem()
    ctrl = PIController(kp=0.005, ki=0.01, tolerance=0.01, dt=0.1, integral_limit=0.5)
    ctrl.run(sys.get_error, sys.apply_control)
