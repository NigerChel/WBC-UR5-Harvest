# 🤖 WBC-UR5-Harvest

**Whole-Body Control (WBC)** system for a **UR5e robotic arm**, tailored for precision **fruit harvesting** in real-world agricultural environments.

This repository provides **ROS Melodic** packages implementing a whole-body control strategy for a 6-DoF **UR5e** manipulator, inspired by **Twan Koolen's** task-prioritized control approach. The system has been tested in **Gazebo simulation** and on a **real robot**.

---

## 🛠️ System Overview

| Component           | Specification          |
|---------------------|------------------------|
| 🤖 Robot Platform    | UR5e                   |
| 🐧 Operating System | Ubuntu 18.04           |
| 🔧 ROS Version      | Melodic                |
| 🧪 Simulation       | Gazebo                 |
| 🤝 Real Deployment  | Fully compatible       |
| 📦 Main Package     | `wbc_ur5`              |

---

## 📦 Features

- ✅ **Whole-body control** with task prioritization  
- 🎯 Cartesian task tracking with **gravity compensation**  
- 🍏 Integrated for **real-time fruit harvesting**  
- 🧩 Modular ROS architecture using nodes and services  
- 🤖 Compatible with **both simulation and physical UR5e**  

---

## 📁 Project Structure

```
WBC-UR5-Harvest/
├── README.md
├── src/
│   ├── wbc_ur5/                  # Whole-body controller
│   ├── ur5_description/         # Robot model and URDF
│   ├── ur5_controllers/         # Joint and task-space controllers
│   ├── gazebo_ros_link_attacher/  # Gazebo plugin for link attachment
│   ├── qpman/                   # QP solver manager
│   ├── robotiq/                 # Robotiq gripper description
│   ├── robotiq_urcap_control/  # URCap integration for Robotiq
│   ├── ros_ur3/                 # UR3 robot support
│   ├── trac_ik/                 # Inverse kinematics solver
│   ├── universal_robot/         # UR robot description and config
│   ├── universal_robots_ros_driver/  # ROS driver for UR robots
│   ├── ur_ikfast/               # IKFast plugin for UR
│   ├── ur_msgs/                 # Custom messages for UR
│   ├── whole_body_state_conversions/  # WBC state conversions
│   ├── whole_body_state_msgs/   # WBC message definitions
│   └── whole_body_state_rviz_plugin/  # RViz WBC visualization plugin
├── setup.bash
├── setup.sh
├── setup.zsh
└── .rosinstall
├── README.md
```

---

## 🚀 Installation

1. Clone into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/NigerChel/WBC-UR5-Harvest.git
cd ..
catkin_make
source devel/setup.bash
```

2. Install dependencies:

```bash
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 🧪 Usage

### ▶️ Run in Simulation (Gazebo)

```bash
roslaunch wbc_ur5 simulation.launch
```

### 🤖 Run on Real Robot

Make sure the **UR5e is powered on and reachable** at the correct IP address:

```bash
roslaunch wbc_ur5 real_robot.launch robot_ip:=192.168.x.x
```

---

## 🧠 System Architecture

The WBC system solves inverse dynamics using a **task-prioritized QP controller**. It includes:

- `task_controller`: Solves for joint torques to achieve Cartesian objectives  
- `ur5_interface`: Communicates with hardware or simulation  
- `state_publisher`: Publishes joint states and TF frames  
- `trajectory_manager`: Handles motion goals for harvesting tasks  

All components are decoupled and communicate via **standard ROS topics and services**.

---

## 🍎 Harvesting Task Description

This project was designed for **real-world agricultural harvesting**, with the following workflow:

1. Approach the fruit using visual servoing or predefined Cartesian poses  
2. Execute tasks with **end-effector constraints** to avoid fruit damage  
3. Use WBC for **smooth, constrained motion** in real-time  
4. Evaluate in both simulation and real orchard environments  

The control strategy is based on **Twan Koolen's WBC formulation**, using **Quadratic Programming (QP)** with strict task prioritization.

---

## 👥 Project Team

This project was developed by a multidisciplinary team as part of an applied robotics research initiative at **Tecnológico de Monterrey**:

- 👩‍🏫 **Rita Fuentes-Aguilar** – *Project Leader* 
- 👁️‍🗨️ **Eduardo Morales-Vargas** – *Postdoctoral Researcher*, contributed to the **vision and perception system**  
- 🔧 **Gustavo Hernández-Melgarejo** – *Postdoctoral Researcher*, developed and optimized the **robotic mechanical platform**  
- 🤖 **Niger Chel-Puc** – *Specialist in Intelligent Robotics*, led the **whole-body control design**, integration, and implementation in both simulation and real hardware

---

## 📌 Notes

- 🧑‍🎓 This work originated as part of the **Ph.D. research** of *Niger Chel* at **CINVESTAV**, where the whole-body control approach was first developed and tested in simulated environments.  
- 🏫 The system was later applied in real harvesting scenarios during his role as **Research Specialist** at **Tecnológico de Monterrey**, in a project focused on **intelligent robotics for precision agriculture**.  
- 🔄 Compatible with both **simulation** and **physical UR5e deployment**.

---

## 🤝 Acknowledgments

This work was supported by the **Challenge-Based Research Funding Program 2022**, grant number **I016-IAMSM006-C3-T3-E**, provided by **Tecnológico de Monterrey**.

We gratefully acknowledge all collaborators and contributors to the success of this interdisciplinary project.

---

## 📄 License & Attribution

This project was developed as part of academic and applied research, and remains attributed to **Tecnológico de Monterrey**.  
Authored and maintained by *Niger Chel*, 2025.








