class PIDController:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        setpoint: float = 0.0,
        integral_limit: float = None,
        output_limit: float = None,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.reset()

    def reset(self) -> None:
        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, current_value: float, dt: float) -> float:
        if dt <= 0:
            raise ValueError("时间间隔 dt 必须为正值")

        error = self.setpoint - current_value
        self.integral += error * dt

        if self.integral_limit is not None:
            self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.output_limit is not None:
            output = max(-self.output_limit, min(output, self.output_limit))

        self.previous_error = error
        return output


if __name__ == "__main__":
    pid = PIDController(kp=0.001, ki=0.01, kd=0.005, setpoint=10, integral_limit=5.0)
    angle_change = pid.compute(current_value=8, dt=0.1)
    print(f"转角变化量: {angle_change:.4f}")
