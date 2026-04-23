# -*- coding: utf-8 -*-
import numpy as np


class AdmittanceController:
    def __init__(self,
                 mass: float,
                 damping: float,
                 stiffness: float,
                 dt: float,
                 desired_force: float = 0.0,
                 a_max: float = None,
                 v_max: float = None):
        if dt <= 0 or mass <= 0:
            raise ValueError("dt 和 mass 必须为正值")

        self.mass = mass
        self.damping = damping
        self.stiffness = stiffness
        self.dt = dt
        self.desired_force = desired_force

        self.position = 0.0
        self.velocity = 0.0

        self.a_max = a_max
        self.v_max = v_max

    def compute(self, force_input: float) -> float:
        f_err = self.desired_force - force_input

        a = (f_err
             - self.damping * self.velocity
             - self.stiffness * self.position
             ) / self.mass

        if self.a_max is not None:
            a = np.clip(a, -self.a_max, self.a_max)

        self.velocity += a * self.dt

        if self.v_max is not None:
            self.velocity = np.clip(self.velocity, -self.v_max, self.v_max)

        new_pos = self.position + self.velocity * self.dt
        delta = new_pos - self.position
        self.position = new_pos
        return delta

    def set_desired_force(self, desired_force: float):
        self.desired_force = desired_force

    def reset(self, position: float = 0.0, velocity: float = 0.0):
        self.position = position
        self.velocity = velocity


if __name__ == "__main__":
    adm = AdmittanceController(
        mass=0.25,
        damping=25,
        stiffness=100,
        dt=0.01,
        desired_force=6.0,
        a_max=50.0,
        v_max=100.0
    )

    inputs = [10.0, 8.0, 6.0, 4.0, 2.0, 0.0]

    print("Step\tF_in\tΔx")
    for i, f in enumerate(inputs, start=1):
        dx = adm.compute(f)
        print(f"{i}\t{f:.2f}\t{dx:.6f}")