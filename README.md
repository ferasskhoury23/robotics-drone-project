# Technion Robotics Final Project – Drone Trajectory Planner

This repository contains the final project for the "Introduction to Robotics" course (236972) at the Technion – Spring 2024–2025 semester.

## 🧠 Project Overview

In this project, we implemented a trajectory planning algorithm for a drone simulator built with **AirSim**. The task includes flying the drone through a cityscape environment using:
- A **predefined obstacle map** for planning the trajectory from the **start** to the **midpoint**.
- **Sensor-based obstacle avoidance** for navigating from the **midpoint** to the **goal** with no prior map.

The drone must fly at a constant altitude, avoid buildings, and reach the goal as fast as possible. The final solution is evaluated based on **speed**, **correctness**, and **course material knowledge**.

## 🧭 Algorithm Summary

The path planning is divided into two phases:

### 🔹 Phase 1: Start to Midpoint (Known Map)
- **Map**: Prior obstacle map at 100m is available.
- **Algorithms Used**:
  - **Visibility Graph**
  - **A\***
  - **Tangent Bug** (for local planning and backup)
- The visibility graph is constructed from known polygonal obstacles and used for A\* pathfinding.
- Tangent Bug acts as a fallback for complex or dynamic edges.

### 🔸 Phase 2: Midpoint to Goal (Unknown Map)
- **Map**: No prior information; the drone relies solely on onboard Lidar.
- **Algorithm Used**:
  - **Tangent Bug Algorithm** (real-time reactive obstacle avoidance)
- The drone continuously senses and navigates based on sensor input.

## 🚁 Simulation Details

- **Simulator**: AirSim (Unreal Engine)
- **Environment**: Simulated urban city with building obstacles.
- **Sensors**: Lidar sensor for 3D obstacle detection.
- **Motion Control**: Drone controlled via Python API.
- **Obstacle Map**: Provided at 100m above sea level via `obstacles_100m_above_sea_level.csv`.

## 📂 Project Structure

```
.
├── DroneClient.py        # Wrapper around AirSim API to control the drone
├── DroneTypes.py         # Custom classes for position, orientation, pose, point cloud
├── main.py               # Sample entry point for running the simulation
├── obstacles_100m_above_sea_level.csv  # Prior obstacle map
├── README.md             # Project documentation
├── TangentBug.py         # Tangent Bug algorithm implementation
├── map.py                # Map utilities and visibility graph
├── Vector2.py            # 2D vector operations
├── helpers.py            # Utility/helper functions
├── quat.py               # Quaternion helpers
```

## 🔧 Getting Started

### 1. Environment Setup

Install Python 3.8 and create a conda environment:
```bash
conda create --name robotics python=3.8
conda activate robotics
pip install airsim numpy msgpack-rpc-python
```

### 2. Running the Project

1. Launch the AirSim simulator.
2. Ensure the `settings.json` file is placed in `Documents/AirSim/`.
3. Run the Python script:
```bash
python main.py
```

## 📊 Output & Evaluation

- The simulation outputs real-time Lidar data and drone position.
- A live trajectory and obstacle plot is expected during the final demo.
- Penalties apply for obstacle collisions.
- Total flight time is a major factor in grading.


---

*Created as part of the Technion Robotics course project.*
