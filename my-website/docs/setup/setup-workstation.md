---
sidebar_position: 2
id: setup-workstation
title: Digital Twin Workstation Setup
description: Set up ROS 2, Gazebo, and Isaac Sim for local development
---

# Digital Twin Workstation Setup

This guide will help you set up a complete development environment on your local machine (PC/Mac/Linux) with ROS 2, Gazebo, and NVIDIA Isaac Sim.

## System Requirements

- **OS**: Ubuntu 22.04 LTS (Recommended)
- **RAM**: 16GB minimum (32GB recommended)
- **Storage**: 50GB free space
- **GPU**: NVIDIA GPU (RTX 2080 or better recommended)
- **Internet**: Stable connection for downloads

## Step 1: Install ROS 2 Humble

### 1.1 Set up Ubuntu repositories

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 1.2 Add ROS repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.3 Install ROS 2 Humble

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### 1.4 Source ROS 2 setup script

```bash
source /opt/ros/humble/setup.bash
```

To make it permanent, add to your `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 2: Install Development Tools

```bash
# Build tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep2

# Gazebo and physics
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-state-publisher

# MoveIt 2 (motion planning)
sudo apt install ros-humble-moveit

# Nav2 (navigation)
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

## Step 3: Install NVIDIA Isaac Sim (Optional but Recommended)

### 3.1 Install NVIDIA Container Toolkit (if using Docker)

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### 3.2 Download Isaac Sim

Visit [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac/sim) and download the latest version.

Or use Docker:

```bash
docker pull nvcr.io/nvidia/isaac-sim:latest
```

## Step 4: Set Up Your Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build

# Source workspace
source install/setup.bash
```

Add to `.bashrc`:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Step 5: Verify Installation

Test ROS 2:

```bash
ros2 topic list
```

Test Gazebo:

```bash
gazebo --version
```

Test that everything works:

```bash
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2
ros2 run demo_nodes_py listener
```

You should see messages being published and received.

## Step 6: Install Python Dependencies

For AI/ML components:

```bash
pip install torch torchvision torchaudio
pip install transformers
pip install openai  # For LLM integration
pip install opencv-python
```

## Troubleshooting

### Common Issues

**Issue**: ROS 2 commands not found
- **Solution**: Run `source /opt/ros/humble/setup.bash` or check your `.bashrc`

**Issue**: Gazebo crashes
- **Solution**: Update graphics drivers `sudo apt install nvidia-driver-XXX`

**Issue**: Isaac Sim doesn't start
- **Solution**: Ensure NVIDIA Container Runtime is installed if using Docker

## Next Steps

Now that your workstation is set up:

1. Go through [Module 1: ROS 2 Fundamentals](../module1/intro.md)
2. Set up your first Gazebo simulation
3. Learn about URDF and robot modeling

---

**For more help**: Visit [ROS 2 Documentation](https://docs.ros.org/en/humble/) or [Gazebo Docs](https://gazebosim.org/)
