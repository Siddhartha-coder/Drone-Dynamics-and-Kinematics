# 🛩️ Modular 6-DOF Drone Dynamics Simulator

A **Python-based simulation environment** for modeling the flight dynamics and control of a quadrotor drone.  
This project implements a **custom 6-DOF physics engine from scratch** (Newton–Euler formulation) and a **cascaded PID controller** to follow user-defined 3D waypoints.

---

## 🚀 Features

### 🧮 **6-DOF Physics Engine**
Simulates the full 12-state quadrotor dynamics:

- **Position:** \( x, y, z \)  
- **Orientation:** \( phi \) (roll), \( theta \) (pitch), \( \psi \) (yaw)  
- **Linear Velocities:** \( u, v, w \)  
- **Angular Velocities:** \( p, q, r \)

Built fully from first principles — **no external physics libraries**.

---

### 🎯 **Cascaded PID Control**
Implements a standard quadrotor control architecture:

#### **Outer Loop — Position Control**
Converts position error → desired acceleration.

#### **Inner Loop — Attitude Control**
Converts desired acceleration → desired roll/pitch angles → motor torques.

Control inputs produced:

- Total thrust: \( U_1 \)  
- Torques: \( U_2, U_3, U_4 \)

---

### 🧱 **Modular Architecture**
Clean separation of components:

```
dynamics/ # Physics engine
controller/ # PID controllers
config/ # Model parameters + tuning gains
visualization/ # 3D animation + plots
main.py # Entry point + simulation driver
```
### 📊 **Interactive Visualization**
Includes:

- **3D flight animation** (Matplotlib)
- **PID error plots**
- **Velocity profiles**
- **Control input visualizations**

All displayed automatically after the simulation.

---

### 📌 **Custom Waypoints**
Users can define any 3D trajectory via CLI:
```
- 'default' → Predefined square test path  
- Or enter coordinates manually (e.g., '5, 5, 10')  
- 'done' → Start simulation  
```
---

## 📦 Prerequisites
```
- Python 3.x  
- NumPy  
- Matplotlib  
```
Install dependencies:

```
pip install numpy matplotlib
```
## ▶️ Usage

Clone the repository or download all source files, then run:
```
python main.py
```
## 📁 File Structure
```
📦Drone-Simulator
│
├── main.py              # Entry point (user I/O + simulation loop)
├── config.py            # Physical constants + PID gains
├── dynamics.py          # Newton-Euler physics engine
├── controller.py        # Cascaded PID control logic
├── visualization.py     # 3D animation + performance plots
└── README.md            # Documentation
```
## 📜 License

This project is released under the MIT License.

## 🤝 Contributions

Pull requests are welcome!
For feature ideas or issues, feel free to open a ticket.

