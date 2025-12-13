---
sidebar_position: 1
---

# Module 1: ROS 2 Fundamentals for Humanoid Robotics

Welcome to **Module 1: ROS 2 Fundamentals**! This module teaches you the foundational concepts of **ROS 2** (Robot Operating System 2)—the industry-standard middleware that powers modern robot development.

## What You'll Learn

After completing this module, you'll understand:
- How ROS 2 nodes communicate through topics, services, and actions
- How to write Python agents that control robots
- How to model robot structure using URDF (Unified Robot Description Format)
- How ROS 2 is used in Modules 2–4 for simulation, perception, and intelligent control

## Prerequisites

This module assumes you have:
- **Basic Python knowledge**: functions, classes, imports (no deep learning required)
- **Linux familiarity**: terminal commands, file paths, environment variables
- **Robotics curiosity**: willing to learn how robots think and communicate

**Time to Prepare**: 30 minutes (install ROS 2, verify setup)

## Your 75-90 Minute Learning Journey

### Chapter 1: ROS 2 Core Concepts (25 minutes)

**What You'll Learn**:
- What ROS 2 is and why roboticists use it
- **Nodes**: Individual computational units that do work
- **Topics**: Named data buses for asynchronous message passing
- **Services**: Request/response communication for synchronous operations
- **Quality of Service (QoS)**: Reliability and latency settings

**Real-World Context**: A robot has 50+ nodes: camera driver, LiDAR driver, motion controller, state estimator, path planner. ROS 2 makes them all communicate seamlessly.

**Time**: ~25 minutes | **Difficulty**: Beginner

[Go to Chapter 1: ROS 2 Core Concepts →](/module1/ch1-ros2-core)

---

### Chapter 2: Bridging Python Agents to ROS 2 (28 minutes)

**What You'll Learn**:
- How **Python agents** make intelligent decisions
- **rclpy**: Python library for writing ROS 2 nodes
- Building a **perception-decision-action loop**: subscribe to sensors → make decisions → publish commands
- Integrating custom AI/ML logic with ROS 2

**Real-World Context**: A humanoid robot sees an object (subscription to camera topic), decides "I should pick it up" (agent logic), and sends gripper commands (publish to action topic).

**Prerequisites**: ✅ Chapter 1 (understand ROS 2 topics and services)

**Time**: ~28 minutes | **Difficulty**: Intermediate

[Go to Chapter 2: Bridging Python Agents to ROS 2 →](/module1/ch2-agent-bridge)

---

### Chapter 3: Creating Humanoid URDF Models (17 minutes)

**What You'll Learn**:
- **URDF**: XML format for describing robot structure
- **Links**: Rigid bodies (torso, arm, head, legs)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- Building a humanoid robot model from scratch
- Loading URDF into simulation tools (Gazebo, Isaac Sim)

**Real-World Context**: You describe your robot's kinematics in URDF: 1 torso link, 2 arm links per arm (upper + forearm), 5 finger joints per hand. Simulators use this to understand how your robot moves.

**Prerequisites**: ✅ Chapters 1-2 (understand ROS 2 context)

**Time**: ~17 minutes | **Difficulty**: Beginner-Intermediate

[Go to Chapter 3: Creating Humanoid URDF Models →](/module1/ch3-urdf-model)

---

## Complete Module Breakdown

| Chapter | Time | Difficulty | Learning Goal |
|---------|------|------------|---|
| 1: ROS 2 Core Concepts | 25 min | Beginner | Understand nodes, topics, services |
| 2: Python Agent Bridging | 28 min | Intermediate | Write agents that control robots |
| 3: URDF Humanoid Modeling | 17 min | Beginner-Int | Describe robot structure |
| **TOTAL** | **~70 min** | | **ROS 2-ready roboticist** |

## How This Module Connects to the Rest

```
Module 1: ROS 2 Fundamentals
├─ Concepts: Nodes, topics, services, URDF
├─ Applied in Module 2: Gazebo and Unity simulation
├─ Applied in Module 3: Isaac Sim, VSLAM, Nav2 perception
└─ Applied in Module 4: Voice, language, and action execution
```

## How to Use This Module

### Reading Strategy

**Recommended**: Read sequentially (Ch1 → 2 → 3)
- Each chapter builds on previous concepts
- You'll progress from theory (what ROS 2 is) to practice (building robots and agents)
- By Chapter 3, you understand the foundation for all robotics in this textbook

**Can you skip chapters?**:
- **No**: All three chapters are essential foundations
- ROS 2 (Ch1) is used everywhere in Modules 2–4
- Python agents (Ch2) are essential for intelligent control (Module 4)
- URDF (Ch3) is loaded into every simulation (Modules 2–3)

### Active Learning

While reading each chapter:

1. **Read with a terminal open** - Copy code examples and run them
2. **Modify parameters** - Change topic names, message rates, robot dimensions to see what breaks
3. **Visualize with RViz** - See node graphs, topic flow, and robot models
4. **Connect to later modules** - Ask "Where will I use this in Module 2, 3, or 4?"

## Key Concepts Glossary

| Term | Definition | Learn More |
|------|-----------|-----------|
| **ROS 2** | Robot Operating System 2 - middleware for robot development | Chapter 1 |
| **Node** | Independent computational unit in ROS 2 | Chapter 1 |
| **Topic** | Named data bus for asynchronous pub/sub communication | Chapter 1 |
| **Service** | Request/response communication for synchronous operations | Chapter 1 |
| **Action** | Long-running request with feedback and result | Chapters 1-4 |
| **rclpy** | Python library for writing ROS 2 nodes | Chapter 2 |
| **Agent** | Python code that makes intelligent decisions | Chapter 2 |
| **URDF** | Unified Robot Description Format (XML robot structure) | Chapter 3 |
| **Link** | Rigid body in robot model | Chapter 3 |
| **Joint** | Connection between two links (revolute, prismatic, fixed) | Chapter 3 |
| **Quality of Service (QoS)** | Settings for reliability and latency of ROS 2 communication | Chapter 1 |
| **Message** | Data structure passed through ROS 2 topics | Chapter 1 |

## FAQ

### Q: Do I need to be a ROS 2 expert before starting?

**A**: No! This module teaches ROS 2 from scratch. We assume you know basic Python and Linux, but no ROS 2 experience is required.

### Q: Will I run code on a real robot in this module?

**A**: No. This module teaches ROS 2 *concepts* and shows how to write code, but all execution is simulated (starting in Module 2). The foundation you build here prepares you for real robot control in advanced modules.

### Q: What's the difference between topics and services?

**A**: Topics are for **continuous streaming** (sensor data flows constantly). Services are for **one-off requests** (ask a question, get an answer). Chapter 1 explains both.

### Q: Can I use ROS 2 on Windows?

**A**: Officially, ROS 2 is best on Linux. However:
1. **WSL2 (Windows Subsystem for Linux)** - Run Linux on Windows
2. **Docker** - Run ROS 2 in a container
3. **VirtualBox** - Run Ubuntu virtual machine

We recommend option 1 or 2 for beginners.

### Q: What comes after Module 1?

**A**:
- **Module 2**: Gazebo and Unity simulation (loads your URDF, uses ROS 2 topics for sensor data)
- **Module 3**: Perception and navigation (Isaac Sim, VSLAM, Nav2 all built on Module 1 foundations)
- **Module 4**: Voice and intelligent control (agents send actions via ROS 2 actions)

### Q: Do I need to memorize ROS 2 message types?

**A**: No! We'll introduce common ones (geometry_msgs, sensor_msgs, std_msgs) as needed. You can always look up the others online.

## Next Steps

Ready to build your first ROS 2 system? Start with **Chapter 1: ROS 2 Core Concepts**.

After Module 1, you'll have the foundation to understand:
- How robots communicate internally (topics, services)
- How to write intelligent decision-making code (Python agents)
- How to describe robot structure (URDF)

This knowledge will power your learning through Modules 2–4!

---

**Module 1 Created**: 2025-12-09 | **Level**: Beginner-to-Intermediate
**Duration**: 70-90 minutes | **Prerequisites**: Python + Linux basics
