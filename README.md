# ROS 2 Panda Voice-Controlled Pick & Place

This repository contains a full-stack ROS 2 manipulation pipeline for the Franka Emika Panda robot. It integrates computer vision, natural language intent parsing, and dynamic inverse kinematics (IK) to perform color-based sorting tasks in a simulated Gazebo Harmonic environment.

## 🎥 Project Overview
The system listens for voice commands (e.g., *"Pick up the green block"*). Once an intent is recognized, a top-down RGB camera pipeline calculates the absolute 3D world coordinates of the target object using 2D-to-3D mathematical projection. A custom C++ MoveIt 2 state machine then takes over, dynamically routing the robot to hover, grasp, and sort the object into a designated drop-off zone based on its color.

## Demo video

[![Watch the demo](https://www.youtube.com/watch?v=6uoRUwFMX70/hqdefault.jpg)](https://www.youtube.com/watch?v=6uoRUwFMX70)

### Key Features
* **Semantic Target Acquisition:** Uses Regex-based natural language parsing to extract actionable commands and target attributes.
* **Markerless 3D Vision:** An OpenCV Python pipeline that dynamically computes absolute `[X, Y, Z]` world coordinates from a 2D camera feed without relying on depth sensors or point clouds.
* **Dynamic MoveIt 2 State Machine:** A robust C++ node that handles IK solving, collision avoidance, and orthogonal grasp alignment to perfectly square the Panda's end-effector with the target.
* **Hardware-Accurate Actuation:** Compensates for the physical 45-degree wrist offset of the Franka Panda flange and manages direct ROS 2 controller synchronization for instantaneous gripper actuation.

---

## 🏗️ System Architecture

1. **Vision & Perception (`vision_pipeline.py`)**
   * Subscribes to the Gazebo camera image topic.
   * Applies HSV color masking and contour detection.
   * Projects 2D pixel coordinates `(u, v)` into 3D world space coordinates `(X, Y, Z)` using known camera intrinsics and environmental constraints.
   * Publishes target `PoseStamped` and target color string.

2. **Kinematics & Control (`panda_kinematics.cpp`)**
   * Subscribes to the target pose and color topics.
   * Uses `MoveGroupInterface` to plan Cartesian trajectories.
   * Actuates the `panda_hand_controller` via `JointTrajectory` messages.
   * Executes a 6-step state machine: Hover → Descend → Grasp → Lift → Translate → Release.

3. **Simulation & TF2 Framework**
   * Fully configured URDF/Xacro models resolving initial state collisions.
   * Synchronized Gazebo/ROS 2 clock bridges to prevent trajectory execution drift.

---

## 🚀 Installation & Setup

### Prerequisites
* **ROS 2** (Humble / Jazzy)
* **Gazebo Harmonic**
* **MoveIt 2**
* **OpenCV** (`cv-bridge`)

### Build Instructions
Clone the repository into your ROS 2 workspace:
```bash
cd ~/panda_manipulation_ws/src
git clone [ https://github.com/PaONEkaruchola/ros2-panda-voice-manipulation.git]( https://github.com/PaONEkaruchola/ros2-panda-voice-manipulation.git) my_panda_project
