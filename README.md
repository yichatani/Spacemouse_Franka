# 🦾 Spacemouse-Franka Teleoperation and Data Collection System

A real-time teleoperation and data collection framework for the **Franka Emika Panda** robotic arm, controlled via a **3Dconnexion SpaceMouse**.  
This system enables intuitive Cartesian control of the robot and synchronized recording of **RGB-D images**, **TCP pose**, and **gripper actions**, allowing the creation of high-quality datasets for robotic manipulation and learning research.

---

## 🚀 Features

- **Real-time teleoperation (200 Hz)** using SpaceMouse input  
- **Cartesian impedance control** for smooth and compliant robot motion  
- **Asynchronous ROS publishing** of end-effector pose (`/franka_pose`)  
- **RGB-D + pose synchronization** using `message_filters`  
- **Gripper control** integrated with SpaceMouse buttons (open / grasp)  
- **Safety mechanisms** including workspace limits, micro deadzones, and overrun watchdogs  
- **Non-blocking visualization** of RGB and depth streams  
- **Easily extendable** for data logging, trajectory replay, or imitation learning  
