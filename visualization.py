import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np
class Visualizer:
    def __init__(self, history, waypoints):
        self.history = history
        self.waypoints = waypoints
        self.t = np.array(history['t'])

    def plot_dashboard(self):
        """Generates the static analysis dashboard (PID, Velocity, Inputs)"""
        fig = plt.figure(figsize=(16, 10))
        fig.canvas.manager.set_window_title("Flight Data Analysis")

        # --- Plot 1: 3D Trajectory (Static) ---
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        ax1.plot(self.history['x'], self.history['y'], self.history['z'], label='Drone Path', linewidth=2)
        wx = [p[0] for p in self.waypoints]
        wy = [p[1] for p in self.waypoints]
        wz = [p[2] for p in self.waypoints]
        ax1.scatter(wx, wy, wz, c='red', marker='x', s=50, label='Waypoints')
        ax1.set_title('3D Flight Path (Static)')
        ax1.set_xlabel('X');
        ax1.set_ylabel('Y');
        ax1.set_zlabel('Z')
        ax1.legend()

        # --- Plot 2: Position Errors (PID Performance) ---
        ax2 = fig.add_subplot(2, 3, 2)
        ax2.plot(self.t, self.history['ex'], label='X Error')
        ax2.plot(self.t, self.history['ey'], label='Y Error')
        ax2.plot(self.t, self.history['ez'], label='Z Error')
        ax2.set_title('PID Position Errors (m)')
        ax2.set_xlabel('Time (s)')
        ax2.grid(True)
        ax2.legend()

        # --- Plot 3: Velocities ---
        ax3 = fig.add_subplot(2, 3, 3)
        ax3.plot(self.t, self.history['u'], label='Vel X')
        ax3.plot(self.t, self.history['v'], label='Vel Y')
        ax3.plot(self.t, self.history['w'], label='Vel Z')
        ax3.set_title('Velocities (m/s)')
        ax3.grid(True)
        ax3.legend()

        # --- Plot 4: Thrust Input (U1) ---
        ax4 = fig.add_subplot(2, 3, 4)
        ax4.plot(self.t, self.history['u1'], color='purple')
        ax4.set_title('Control Input: Thrust (N)')
        ax4.set_xlabel('Time (s)')
        ax4.grid(True)

        # --- Plot 5: Torque Inputs (U2, U3, U4) ---
        ax5 = fig.add_subplot(2, 3, 5)
        ax5.plot(self.t, self.history['u2'], label='Roll Torque')
        ax5.plot(self.t, self.history['u3'], label='Pitch Torque')
        ax5.plot(self.t, self.history['u4'], label='Yaw Torque')
        ax5.set_title('Control Input: Torques (Nm)')
        ax5.set_xlabel('Time (s)')
        ax5.grid(True)
        ax5.legend()

        plt.tight_layout()

    def animate_flight(self):
        """Generates the 3D animation of the drone flying"""
        fig_ani = plt.figure(figsize=(10, 8))
        fig_ani.canvas.manager.set_window_title("3D Flight Animation")
        ax = fig_ani.add_subplot(111, projection='3d')

        # Set plot limits with some margin
        margin = 1.0
        all_x = self.history['x'] + [p[0] for p in self.waypoints]
        all_y = self.history['y'] + [p[1] for p in self.waypoints]
        all_z = self.history['z'] + [p[2] for p in self.waypoints]

        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        ax.set_zlim(min(0, min(all_z)), max(all_z) + margin)  # Keep ground at 0 usually

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Drone Trajectory Replay')

        # Plot Waypoints (Static)
        wx = [p[0] for p in self.waypoints]
        wy = [p[1] for p in self.waypoints]
        wz = [p[2] for p in self.waypoints]
        ax.scatter(wx, wy, wz, c='red', marker='x', s=50, label='Waypoints')

        # Dynamic Elements
        line, = ax.plot([], [], [], 'b--', lw=1, label='Flown Path')
        drone_point, = ax.plot([], [], [], 'ko', markersize=6, label='Drone')

        # Skip frames to speed up animation (target ~400 frames total)
        skip_step = max(1, len(self.t) // 400)
        indices = range(0, len(self.t), skip_step)

        def update(frame_idx):
            # Update path line up to current frame
            line.set_data(self.history['x'][:frame_idx], self.history['y'][:frame_idx])
            line.set_3d_properties(self.history['z'][:frame_idx])

            # Update drone position
            drone_point.set_data([self.history['x'][frame_idx]], [self.history['y'][frame_idx]])
            drone_point.set_3d_properties([self.history['z'][frame_idx]])

            return line, drone_point

        self.ani = animation.FuncAnimation(fig_ani, update, frames=indices, interval=20, blit=False)
        ax.legend()

    def show(self):
        plt.show()