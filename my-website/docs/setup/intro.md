---
sidebar_position: 1
id: setup-intro
title: Setup Guides Overview
description: Getting your development environment ready for Humanoid Robotics
---

# Setup Guides

Before diving into the modules, you need to set up your development environment. This section covers three main deployment scenarios:

## Three Deployment Models

### 1. **Digital Twin Workstation**
For local development and simulation on your PC/Mac with full physics simulation.

**Best for:**
- Learning and experimentation
- Development before running on hardware
- Prototyping behaviors and controllers

**Requirements:**
- 16GB+ RAM
- GPU (optional but recommended for faster simulation)
- ROS 2 Humble
- Gazebo
- NVIDIA Isaac Sim or Alternative

---

### 2. **Physical AI Edge Kit**
For running inference and control on a humanoid robot or embedded system.

**Best for:**
- Deploying models to actual hardware
- Real-time control
- Production deployments

**Requirements:**
- NVIDIA Jetson (Orin or Xavier)
- ROS 2 Humble
- CUDA-capable GPU
- Edge AI framework (ONNX Runtime, TensorRT)

---

### 3. **Cloud-Native Development**
For collaborative development, distributed systems, and cloud deployments.

**Best for:**
- Team collaboration
- Distributed simulations
- Model training at scale
- API-based services

**Requirements:**
- Docker/Kubernetes
- Cloud account (AWS, GCP, Azure, or local)
- Container registry

---

## Quick Start Path

1. **Start with Digital Twin Workstation** - Get everything running locally first
2. **Progress through Modules 1-4** - Learn the concepts
3. **Transition to Edge Kit** - Deploy to real hardware
4. **Optionally use Cloud** - For large-scale training or team collaboration

---

## System Requirements Summary

| Aspect | Workstation | Edge Kit | Cloud |
|--------|------------|----------|-------|
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 Jetson | Docker |
| RAM | 16GB+ | 8GB+ (Jetson) | Flexible |
| GPU | Recommended | Built-in | Optional |
| ROS 2 | Humble | Humble | Humble |
| Cost | Free (OSS) | ~$500-2000 | Pay-as-you-go |

---

## Next Steps

Choose your deployment model and follow the corresponding setup guide:

- [Digital Twin Workstation Setup](./setup-workstation)
- [Physical AI Edge Kit Setup](./setup-edge-kit)
- [Cloud-Native Development Setup](./setup-cloud)

---

**Note:** You can start with one model and transition to another as your needs evolve. The learning path works across all three!
