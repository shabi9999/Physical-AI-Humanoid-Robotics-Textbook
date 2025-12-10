---
sidebar_position: 3
id: setup-edge-kit
title: Physical AI Edge Kit Setup
description: Deploy to NVIDIA Jetson and humanoid robots
---

# Physical AI Edge Kit Setup

This guide covers deploying your learned models to actual hardware using NVIDIA Jetson and Edge AI frameworks.

## Hardware Requirements

- **NVIDIA Jetson Orin** (preferred) or **Jetson AGX Xavier**
- **Power Supply**: 60W+ (depending on Jetson model)
- **Storage**: microSD card (minimum 64GB recommended)
- **Network**: Ethernet or WiFi connection
- **Humanoid Robot** (optional): Compatible with ROS 2

## Step 1: Flash Jetson OS

### 1.1 Using Jetson Orin Nano Developer Kit

Download [JetPack SDK](https://developer.nvidia.com/embedded/jetpack) (version 5.1.1 or later).

**On your host PC:**

```bash
# Install tools
sudo apt-get install -y git git-lfs
git clone https://github.com/NVIDIA/nvidia-jetson-nano-image-build.git
cd nvidia-jetson-nano-image-build

# Flash the image
./tools/setup.sh
```

**Or use NVIDIA SDK Manager:**

1. Download [NVIDIA SDK Manager](https://developer.nvidia.com/nvidia-sdk-manager)
2. Select target device (Jetson Orin/Xavier)
3. Follow on-screen prompts to flash

## Step 2: Initial Jetson Setup

```bash
# SSH into Jetson
ssh nvidia@<jetson_ip>

# Update system
sudo apt update && sudo apt upgrade -y

# Install Python and development tools
sudo apt install -y python3-pip python3-dev
sudo apt install -y git curl wget

# Verify GPU
nvidia-smi
```

## Step 3: Install ROS 2 Humble

On the Jetson, follow the same steps as the workstation:

```bash
# Add ROS 2 repository
locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Install ROS 2
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop-full
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 4: Install Edge AI Frameworks

### For ONNX Runtime (Recommended)

```bash
# Install ONNX Runtime with GPU support
pip3 install --upgrade pip
pip3 install onnxruntime-gpu

# Verify
python3 -c "import onnxruntime; print(onnxruntime.get_device())"
```

### For TensorRT

```bash
# Already installed with JetPack, but verify:
python3 -c "import tensorrt; print(tensorrt.__version__)"

# Install TensorRT Python bindings if needed
sudo apt install -y libnvinfer8 libnvinfer-dev libnvinfer-plugin8 python3-libnvinfer
```

### For PyTorch (Jetson-optimized)

```bash
# Download PyTorch for Jetson (replace with latest version)
wget https://nvidia.box.com/shared/static/p57fsjrxN9p00hDAaxzt5RM6IY0zlv0.whl -O torch-2.1.0-cp310-cp310-linux_aarch64.whl

pip3 install torch-2.1.0-cp310-cp310-linux_aarch64.whl
pip3 install torchvision torchaudio
```

## Step 5: Deploy ROS 2 Package to Jetson

On your host machine:

```bash
# Create deployment package
cd ~/ros2_ws
colcon build --packages-select your_package

# Package everything
tar -czf ros2_package.tar.gz install/

# SCP to Jetson
scp ros2_package.tar.gz nvidia@<jetson_ip>:~/
```

On Jetson:

```bash
# Extract and source
tar -xzf ros2_package.tar.gz
source install/setup.bash

# Run your package
ros2 launch your_package your_launch_file.py
```

## Step 6: Connect to Humanoid Robot

### Hardware Connections

- Connect Jetson to robot via USB, CAN bus, or Ethernet
- Ensure power distribution is correct
- Verify all sensors are connected

### ROS 2 Drivers

Install drivers for your robot platform:

```bash
# Example: Boston Dynamics Spot
sudo apt install ros-humble-spot-ros2

# Example: Unitree Go2
git clone https://github.com/unitreerobotics/go2_ros2_sdk.git
cd go2_ros2_sdk
colcon build
```

## Step 7: Run Real-Time Control

Create a launch file for your robot:

```yaml
# robot_control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='robot_controller',
            name='robot_controller',
        ),
    ])
```

Run it:

```bash
ros2 launch your_package robot_control.launch.py
```

## Performance Optimization

### GPU Memory Management

```bash
# Monitor GPU usage
watch -n 1 nvidia-smi

# Set persistent mode
sudo nvidia-smi -pm 1
```

### CPU Frequency Scaling

```bash
# Set max performance
echo "sudo jetson_clocks" >> ~/.bashrc
sudo jetson_clocks
```

## Network Configuration

### For Robot Swarm/Multi-Agent

```bash
# Set ROS_DOMAIN_ID for network isolation
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0

# Export ROS variables
export ROS_MASTER_URI=http://<jetson_ip>:11311
export ROS_IP=<jetson_ip>
```

## Troubleshooting

**Issue**: CUDA out of memory
- **Solution**: Reduce batch size, use quantization, or upgrade to Jetson Orin

**Issue**: ROS 2 nodes can't communicate
- **Solution**: Check network settings, ensure same ROS_DOMAIN_ID

**Issue**: Jetson overheating
- **Solution**: Add heatsink, improve ventilation, or throttle performance

## Deployment Checklist

- [ ] JetPack OS flashed successfully
- [ ] ROS 2 Humble installed and working
- [ ] Edge AI framework (ONNX/TensorRT) installed
- [ ] Your ROS 2 package deployed
- [ ] Robot drivers installed
- [ ] Network connectivity verified
- [ ] GPU memory optimized
- [ ] Performance benchmarked

---

**Next Steps**: Deploy your first VLA model to the robot (see Module 4)
