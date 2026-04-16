# [3-DOF Marine Simulation] - Autonomous Systems Simulator

## Overview
This repository contains a real-time, physics-based simulator developed for the [Técnico Solar Boat (TSB)] recruitment phase. Built using Unreal Engine (C++) and Python, this simulator tests the physical dynamics of an autonomous surface vehicle (ASV) and bridges simulated sensor data into a ROS2 compatible format for navigation and control testing.

## Key Features
* **Physics & Dynamics:** Implemented a 3-DOF model using Fossen's equations of motion (Surge, Sway, Yaw) for accurate marine hydrodynamics.
* **Non-Linear Propulsion:** Custom differential drive system mimicking the official BlueRobotics T200 thruster performance curves (current-to-thrust).
* **Virtual Sensor Suite:** Simulated GPS, IMU (AHRS), and 360 degree LiDAR.
* **Real-World Degradation:** Programmatic injection of Gaussian noise, signal latency (10Hz capping), and intermittent sensor dropouts to stress-test control algorithms.
* **ROS2 Bridge Architecture:**A lightweight UDP networking layer that streams JSON telemetry from Unreal Engine to a Python-based ROS2 bridge, ensuring clean separation between the simulation and control layers.

---

## Quickstart Guide

### Prerequisites
* **Unreal Engine:** Version [e.g., 5.3]
* **Python:** Version 3.8+ (with standard `socket` and `json` libraries)
* **OS:** Windows / Linux / Mac (probably)

### Installation & Build
1. **Clone the repository:**
   ```bash
   git clone [https://github.com/yourusername/your-repo-name.git](https://github.com/yourusername/your-repo-name.git)
   cd your-repo-name

2. **Unreal Engine Setup**
   Right-click the TSBphase2.uproject file and select Generate Visual Studio project files.
   Open the .sln file in Visual Studio and compile the project (Development Editor Win64).
   Launch the Unreal Editor.

3. **Start ROS2 Bridge**
   Open a terminal, navigate to the project directory, and run the bridge script to start listening for Unreal Engine telemetry: $ python ros2_bridge.py

4. **Start the Simulation**
   Hit Play in the Unreal Engine editor. Use the W, A, S, D keys to control the vehicle's thrusters.

5. **Verify Data**
   Check the Python terminal to see live, formatted ROS2 topic data (/imu/data, /gps/fix, /scan) streaming from the engine


## Architecture & Methodological Justifications

1. **Fossen's Hydrodynamic Model vs Native Game Physics**
Instead of relying on Unreal Engine's generic physics solver, I implemented Fossen's marine craft equations of motion natively in C++. This allows for absolute mathematical control over the mass, inertia tensors, added mass, and linear damping vectors, ensuring the simulation behaves identically to the provided academic parameters for the Otter USV.

2. **T200 Thruster Non-Linearity**
Standard video-game movement applies linear force. To satisfy real-world engineering constraints, I implemented a mathematical curve mapping Ampere draw (-20A to +20A) to Newtons based on the official BlueRobotics data. This accurately reflects the thruster's lower efficiency in reverse and provides a true differential-drive turning radius based on the physical 0.395m lever arm of the hull.

3. **UDP-to-Python ROS2 Bridge Architecture**
To satisfy the requirement for a "clean separation between simulation and control layers," I opted for a UDP bridge rather than compiling rclcpp natively inside Unreal Engine. Compiling ROS2 inside Unreal's build tool can lead to severe dependency conflicts and poor cross-platform scalability. By serializing the simulated sensor data into lightweight JSON and broadcasting it via UDP locally, Unreal Engine remains highly performant. A lightweight Python script (ros2_bridge.py) catches this data and acts as the official ROS2 publisher. This architecture is robust and fast

4. **Sensor Degrading Strategy**
To properly test navigation robustness, noise was not just randomized, but scaled using specific variances (e.g., 0.00001 for GPS Lat/Lon to simulate ~1.1m drift). A programmatic timer limits data publishing to realistic hardware frequencies (10Hz), and a probabilistic fail-state simulates a 5% random hardware packet drop.
   

## Repository Structure
├── Content/                  # Unreal Engine Maps, Blueprints, Meshes, and Materials
├── Source/                   # C++ Source Code 
│   └── TSBphase2/
│       ├── OtterUSV.cpp          # Core movement, Fossen equations, and UDP Sender
│       ├── OtterUSV.h
│       └── TSBphase2.Build.cs    # Build rules
├── ros2_bridge.py            # Python UDP listener and ROS2 node mocking
├── README.md                 # Project documentation
└── TSBphase2.uproject        # Unreal Engine project file



