import numpy as np
class PIDController:
    def __init__(self, config):
        self.cfg = config

    def update(self, state, target_pos):
        """
        Computes control inputs based on current state and target position.
        Returns: inputs [U1, U2, U3, U4], error_data (for visualization)
        """
        x, y, z, phi, theta, psi, vx, vy, vz, p, q, r = state

        # --- 1. Position Control (Outer Loop) ---
        # Calculate Error
        ex = target_pos[0] - x
        ey = target_pos[1] - y
        ez = target_pos[2] - z

        # PID (PD actually) for desired acceleration
        ax_des = self.cfg.Kp_pos[0] * ex - self.cfg.Kd_pos[0] * vx
        ay_des = self.cfg.Kp_pos[1] * ey - self.cfg.Kd_pos[1] * vy
        az_des = self.cfg.Kp_pos[2] * ez - self.cfg.Kd_pos[2] * vz

        # Desired Total Thrust (U1)
        # We need to overcome gravity + desired Z acceleration
        u1 = self.cfg.m * (self.cfg.g + az_des)

        # --- 2. Attitude Control (Inner Loop) ---
        # Invert the dynamics to find desired Roll/Pitch angles to achieve ax_des/ay_des
        # Small angle approximation for stability
        phi_des = (1 / self.cfg.g) * (ax_des * np.sin(psi) - ay_des * np.cos(psi))
        theta_des = (1 / self.cfg.g) * (ax_des * np.cos(psi) + ay_des * np.sin(psi))
        psi_des = 0.0  # Maintain 0 heading

        # Attitude Errors
        e_phi = phi_des - phi
        e_theta = theta_des - theta
        e_psi = psi_des - psi

        # Calculate Torques (U2, U3, U4)
        u2 = self.cfg.Kp_att[0] * e_phi - self.cfg.Kd_att[0] * p
        u3 = self.cfg.Kp_att[1] * e_theta - self.cfg.Kd_att[1] * q
        u4 = self.cfg.Kp_att[2] * e_psi - self.cfg.Kd_att[2] * r

        # Prevent negative thrust
        u1 = max(0.0, u1)

        inputs = np.array([u1, u2, u3, u4])

        # Return data for plotting: [x_err, y_err, z_err]
        errors = np.array([ex, ey, ez])

        return inputs, errors