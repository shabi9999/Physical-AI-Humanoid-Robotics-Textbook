---
sidebar_position: 100
id: glossary
title: Robotics Glossary & References
description: Key terms and concepts in robotics, ROS 2, and humanoid robotics
---

# Robotics Glossary & References

## ROS 2 Terms

### Node
A Node is an executable that uses ROS 2 to communicate with other Nodes using topics, services, and actions. Each Node is typically responsible for a single responsibility (sensor reading, motor control, etc.).

### Topic
A named bus over which Nodes exchange Messages of a single, nominally anonymous, many-to-many nature. Topics are asynchronous and allow decoupled communication.

### Service
A synchronous request-reply between client and server. Services are used for request-reply semantics where a client needs an immediate response.

### Action
A communication pattern for long-running tasks. Actions provide feedback while a task is being executed and support cancellation.

### Parameter
A named value that can be set at runtime. Parameters are used to configure Node behavior dynamically.

### Publisher
A Node that sends Messages on a Topic.

### Subscriber
A Node that receives Messages from a Topic.

### Message
A data structure that can be sent between Nodes. Messages are defined using ROS 2 Interface Definition Language (IDL).

### QoS (Quality of Service)
Settings that control the communication behavior of Publishers and Subscribers, including reliability, durability, and history depth.

---

## Robotics Concepts

### URDF (Unified Robot Description Format)
An XML format for specifying a robot's structure, including links, joints, and properties. Used by ROS 2 for visualization and physics simulation.

### Gazebo
An open-source robot simulation platform that provides physics simulation, sensor modeling, and dynamic environment interaction.

### SLAM (Simultaneous Localization and Mapping)
A technique for a robot to create a map of its environment while simultaneously determining its location within that map.

### Kinematics
The study of motion without considering forces. Forward kinematics computes end-effector position from joint angles; inverse kinematics solves the reverse problem.

### Dynamics
The study of motion considering forces and torques. Used to simulate robot behavior and control design.

### Trajectory Planning
The process of computing a sequence of states (joint positions, velocities) that moves the robot from start to goal configuration.

### Control System
A system that regulates robot behavior using feedback (e.g., PID controllers for joint control).

### End-Effector
The part of the robot that interacts with the environment (gripper, tool, etc.).

---

## Humanoid Robotics Specific Terms

### Bipedal Locomotion
Walking on two legs. Requires balance control and dynamic motion planning.

### Humanoid Robot
A robot designed to resemble and mimic human form and behavior.

### Actuator
A motor or device that produces motion. Humanoid robots typically use servo motors or hydraulic actuators.

### Sensor Fusion
Combining data from multiple sensors (IMU, force/torque sensors, encoders) for accurate state estimation.

### Center of Mass (CoM)
The point representing the average position of mass in the robot's body. Critical for balance control.

---

## AI/ML Terms

### LLM (Large Language Model)
A deep learning model trained on vast amounts of text to understand and generate human language.

### Vision Transformer
A transformer-based architecture for computer vision tasks, treating images as sequences of patches.

### Speech Recognition
Converting spoken words into text. Whisper is an OpenAI model for automatic speech recognition.

### Semantic Understanding
Extracting meaning from language or visual input, beyond just pattern matching.

---

## Digital Twin Concepts

### Digital Twin
A virtual representation of a physical system that can be used for simulation, analysis, and control.

### Synthetic Data
Artificially generated data used for training machine learning models, useful when real data is expensive or difficult to obtain.

### Physics Simulation
Using computational methods to simulate physical interactions (collisions, forces, friction) in a virtual environment.

---

## References & Resources

### Official Documentation
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/)
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaac/)

### Tools & Libraries
- **ROS 2 CLI Tools**: ros2, colcon, rosdep
- **Visualization**: RViz2, Foxglove Studio
- **Simulation**: Gazebo, PyBullet, CoppeliaSim
- **Planning**: MoveIt 2, OMPL
- **ML Frameworks**: PyTorch, TensorFlow, JAX

### Learning Resources
- ROS 2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- Gazebo Tutorials: https://gazebosim.org/garden/
- Robotics Stack Exchange: https://robotics.stackexchange.com/

---

## Contributors & Acknowledgments

This course was developed as part of the Humanoid Robotics hackathon initiative, combining knowledge from:
- ROS 2 Community
- NVIDIA Isaac Research Team
- Gazebo Simulation Community
- Academic Robotics Programs

---

**Last Updated:** December 2024
**Version:** 1.0
