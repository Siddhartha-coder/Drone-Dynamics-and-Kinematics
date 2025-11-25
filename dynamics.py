import numpy as np
class Quadrotor:
    def __init__(self, config):
        self.cfg = config
        # State Vector: [x, y, z, phi, theta, psi, u, v, w, p, q, r]
        # x,y,z: Position (Inertial)
        # phi,theta,psi: Euler Angles (Roll, Pitch, Yaw)
        # u,v,w: Linear Velocity (Inertial approximation for this sim)
        # p,q,r: Angular Velocity (Body)
        self.state = np.zeros(12)

    def equations_of_motion(self, state, inputs):
        """
        Calculates the derivatives of the state vector based on Newton-Euler equations.
        inputs: [Thrust, Torque_phi, Torque_theta, Torque_psi]
        """
        x, y, z, phi, theta, psi, u, v, w, p, q, r = state
        T, tau_phi, tau_theta, tau_psi = inputs

        # 1. Linear Accelerations
        # R_z(psi) * R_y(theta) * R_x(phi) transformation of thrust vector
        # F = ma
        ax = (np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi)) * T / self.cfg.m
        ay = (np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi)) * T / self.cfg.m
        az = -self.cfg.g + (np.cos(phi) * np.cos(theta)) * T / self.cfg.m

        # 2. Angular Accelerations (Euler's Equations)
        p_dot = (tau_phi - (self.cfg.Iz - self.cfg.Iy) * q * r) / self.cfg.Ix
        q_dot = (tau_theta - (self.cfg.Ix - self.cfg.Iz) * p * r) / self.cfg.Iy
        r_dot = (tau_psi - (self.cfg.Iy - self.cfg.Ix) * p * q) / self.cfg.Iz

        # Return derivatives: [vel_x, vel_y, vel_z, ang_vel_phi...]
        return np.array([u, v, w, p, q, r, ax, ay, az, p_dot, q_dot, r_dot])

    def step(self, inputs):
        """
        Integrates the state forward by one time step (dt) using Euler method.
        """
        ds = self.equations_of_motion(self.state, inputs)
        self.state += ds * self.cfg.dt
        return self.state