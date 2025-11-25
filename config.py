import numpy as np
class SimulationConfig:
    def __init__(self):
        # --- Physics Constants ---
        self.g = 9.81
        self.m = 1.0
        self.L = 0.25
        self.Ix = 0.01
        self.Iy = 0.01
        self.Iz = 0.02
        self.dt = 0.01
        self.t_max = 30.0

        # --- PID Gains ---
        self.Kp_pos = np.array([2.0, 2.0, 4.0])
        self.Kd_pos = np.array([1.5, 1.5, 2.5])

        # Attitude Controller (Inner Loop)
        self.Kp_att = np.array([6.0, 6.0, 2.0])
        self.Kd_att = np.array([1.5, 1.5, 0.1])