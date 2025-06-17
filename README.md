# WBC-UR5-Harvest

Whole-Body Control (WBC) system for a UR5e robotic arm applied to fruit harvesting.

This repository contains ROS Melodic packages implementing a whole-body control algorithm for a 6-DoF UR5e manipulator, based on the approach of Twan Koolen. The system has been tested both in simulation (Gazebo) and on a real robot.

## 🛠 System Overview
- **Robot platform**: UR5e
- **ROS version**: Melodic
- **OS**: Ubuntu 18.04
- **Simulation**: Gazebo
- **Real-world deployment**: Fully compatible
- **Main package**: `wbc_ur5`

## 📦 Main Features
- Whole-body control for manipulation under task-space constraints
- Gravity compensation and Cartesian task tracking
- Integrated with real-time fruit harvesting application
- Modular architecture based on ROS nodes and services

## 📁 Project Structure

WBC-UR5-Harvest/
├── README.md
├── src/
│ └── [ROS packages including wbc_ur5, robot description, controllers, etc.]

## 🚀 Installation

1. Clone the repository into your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/NigerChel/WBC-UR5-Harvest.git
   cd ..
   catkin build
   source devel/setup.bash

   Install necessary dependencies:
   sudo apt-get update
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y

  🧪 Usage
Simulation in Gazebo:
roslaunch wbc_ur5 simulation.launch

Real Robot:
Ensure the UR5e is connected and configured with the correct IP.
roslaunch wbc_ur5 real_robot.launch robot_ip:=192.168.x.x

🧠 System Architecture
The system uses a whole-body controller to solve task-space control problems subject to joint constraints, gravity compensation, and redundancy resolution. The architecture includes:

task_controller: Computes desired joint torques

ur5_interface: Interfaces with the hardware or simulation

state_publisher: Publishes robot state and transforms

trajectory_manager: Manages task goals for harvesting

Each ROS node communicates via standard ROS topics and services to ensure modularity.

🍎 Harvesting Task Description
This project was designed as a contribution to a real-world agricultural harvesting task:

The robot approaches the fruit with visual-servoing or fixed pose commands

Executes Cartesian tasks with constraints using the WBC solver

Incorporates end-effector motion constraints to avoid damaging the fruit

Control is based on Twan Koolen's WBC formulation using quadratic programming (QP) with task prioritization. The real robot was tested on multiple harvesting experiments.

📌 Notes
Developed as part of a research project at Tecnológico de Monterrey

Although developed by a specialist (Niger Chel) during work at the institution, authorship is attributed to the Tec

Compatible with both simulation and physical deployment on UR5e

Developed at Tecnológico de Monterrey as part of an applied research project in intelligent robotics for precision agriculture.








