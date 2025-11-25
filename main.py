import numpy as np
from config import SimulationConfig
from dynamics import Quadrotor
from controller import PIDController
from visualization import Visualizer

def get_user_waypoints():
    waypoints = []
    print("--- Drone Trajectory Input ---")
    print("Enter trajectory points (x, y, z).")
    print("Type 'done' when finished.")
    print("Type 'default' to load a standard test square.")

    count = 1
    while True:
        user_in = input(f"Point {count} [x,y,z] > ").strip()

        if user_in.lower() == 'default':
            return [
                np.array([0, 0, 0]),
                np.array([0, 0, 1]),
                np.array([2, 0, 1]),
                np.array([2, 2, 1]),
                np.array([0, 2, 1]),
                np.array([0, 0, 1]),
                np.array([0, 0, 0])
            ]

        if user_in.lower() == 'done':
            if len(waypoints) < 2:
                print("Please enter at least 2 points.")
                continue
            break

        try:
            parts = [float(x) for x in user_in.split(',')]
            if len(parts) != 3:
                print("Error: Must enter exactly 3 numbers separated by commas.")
                continue
            waypoints.append(np.array(parts))
            count += 1
        except ValueError:
            print("Error: Invalid numbers.")

    return waypoints


def run_simulation():
    # 1. Setup
    waypoints = get_user_waypoints()
    cfg = SimulationConfig()
    drone = Quadrotor(cfg)
    ctrl = PIDController(cfg)

    # 2. Data Logging containers
    history = {
        't': [],
        'x': [], 'y': [], 'z': [],
        'u': [], 'v': [], 'w': [],
        'ex': [], 'ey': [], 'ez': [],
        'u1': [], 'u2': [], 'u3': [], 'u4': []
    }

    # 3. Simulation Loop
    t = 0
    curr_wp_idx = 0
    tolerance = 0.2  # distance to waypoint to consider it "reached"

    print("\nStarting Simulation...")
    while t < cfg.t_max:
        if curr_wp_idx >= len(waypoints):
            # Hold the last position if all waypoints done
            target = waypoints[-1]
        else:
            target = waypoints[curr_wp_idx]

        # Check distance to current waypoint
        dist = np.linalg.norm(drone.state[0:3] - target)
        if dist < tolerance:
            if curr_wp_idx < len(waypoints) - 1:
                print(f"Reached Waypoint {curr_wp_idx + 1}: {target}")
                curr_wp_idx += 1

        # Run Control & Physics
        inputs, errors = ctrl.update(drone.state, target)
        drone.step(inputs)

        # Log Data
        history['t'].append(t)
        history['x'].append(drone.state[0])
        history['y'].append(drone.state[1])
        history['z'].append(drone.state[2])
        history['u'].append(drone.state[6])
        history['v'].append(drone.state[7])
        history['w'].append(drone.state[8])
        history['ex'].append(errors[0])
        history['ey'].append(errors[1])
        history['ez'].append(errors[2])
        history['u1'].append(inputs[0])
        history['u2'].append(inputs[1])
        history['u3'].append(inputs[2])
        history['u4'].append(inputs[3])

        t += cfg.dt

    print("Simulation Complete. Generating Plots...")

    # 4. Visualization
    viz = Visualizer(history, waypoints)

    # Generate the Static Dashboard (PID errors, Thrusts, etc)
    viz.plot_dashboard()

    # Generate the 3D Animation
    viz.animate_flight()

    # Show both windows
    viz.show()


if __name__ == "__main__":
    run_simulation()