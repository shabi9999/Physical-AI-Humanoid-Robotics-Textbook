# Chapter 3: URDF Modeling for Humanoid Robots - Code Examples

This directory contains a complete URDF (Unified Robot Description Format) model of a simplified humanoid robot and tools to work with it.

## Files

- **simple_humanoid.urdf** - XML file describing a humanoid robot structure (15 links, 14 joints)
- **visualize_urdf.py** - Python script to parse, analyze, and display URDF structure
- **README.md** - This file (setup and run instructions)

## What is URDF?

**URDF** is an XML-based format that describes:
- **Links**: Rigid bodies (limbs, joints, etc.)
- **Joints**: Connections between links with motion constraints
- **Inertial properties**: Mass and moment of inertia for physics
- **Visual geometry**: How the robot looks (rendered in RViz)
- **Collision geometry**: For planning and collision detection

## Prerequisites

### Environment Setup

1. **For parsing/validation (Python only)**:
   - Python 3.8+
   - No external dependencies required!

2. **For ROS 2 validation and visualization**:
   - ROS 2 Humble installed and sourced
   - `check_urdf` tool (part of urdfdom-tools)
   - RViz for visualization

### Install Prerequisites (if needed)

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop-full

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install urdfdom-tools for check_urdf
sudo apt install liburdfdom-tools
```

## Understanding the URDF Structure

### Robot Anatomy

The `simple_humanoid.urdf` models a humanoid with:

```
                    HEAD
                     |
    LEFT ARM -- TORSO -- RIGHT ARM
                     |
    LEFT LEG ---- HIPS ---- RIGHT LEG
```

**Detailed Structure**:
- **1 base link** (kinematic tree root, no mass)
- **1 torso** (main body, 20 kg)
- **1 head** (3 kg)
- **2 arms** (left/right):
  - Shoulder (1 kg each)
  - Elbow (0.8 kg each)
  - Wrist (0.5 kg each)
- **2 legs** (left/right):
  - Hip (2 kg each)
  - Knee (1.5 kg each)
  - Ankle (1 kg each)

**Total mass**: ~40 kg

### Joint Types

| Type | Motion | Examples |
|------|--------|----------|
| **revolute** | Rotational (bounded) | Shoulder, elbow, hip, knee |
| **prismatic** | Linear (not used in humanoid) | Extension arms, lifts |
| **fixed** | No motion | Head to torso (simplified) |

## Running the Examples

### Example 1: Parse and Visualize URDF (No ROS 2 Required)

Parse the URDF file and display its structure:

```bash
cd examples/module1/chapter3
python3 visualize_urdf.py
```

**Expected Output**:
```
============================================================
URDF Parser Summary
============================================================
Robot name: simple_humanoid
File: .../simple_humanoid.urdf

Structure:
  Links: 15
  Joints: 14

Link Properties:
  With inertial properties: 14
  With visual geometry: 14
  With collision geometry: 14

Joint Types:
  fixed: 2
  revolute: 12

Kinematic Tree:

base_link
    +-- torso_link [fixed]
    |   +-- head_link [fixed]
    |   +-- left_shoulder_link [revolute]
    |       +-- left_elbow_link [revolute]
    |           +-- left_wrist_link [revolute]
    ...
[OK] URDF validation passed!
[SUCCESS] URDF parsing and visualization complete!
```

**What You're Seeing**:
- Robot name from the URDF file
- Count of links and joints
- Properties of each link
- Kinematic tree (parent-child relationships)
- Validation result (tree has no cycles)

### Example 2: Validate URDF with ROS 2 Tool

Use the ROS 2 `check_urdf` command to validate:

```bash
source /opt/ros/humble/setup.bash
cd examples/module1/chapter3
check_urdf simple_humanoid.urdf
```

**Expected Output**:
```
robot name is: simple_humanoid
---------- Successfully Parsed XML --------
root Link: base_link
    child(1):  torso_link
        child(1): head_link
        child(2): left_shoulder_link
            child(1): left_elbow_link
                child(1): left_wrist_link
        child(3): right_shoulder_link
            ...
    TF Tree:
    base_link
        - base_to_torso
            - torso_link
                - torso_to_head
                    - head_link
                - torso_to_left_shoulder
                    - left_shoulder_link
                        ...
Transform accepted. There are 12 non-fixed joints and 3 frames.
```

**What This Tells You**:
- URDF is syntactically valid XML
- Kinematic tree structure is correct
- Number of joints and frames
- No missing links or broken references

### Example 3: Visualize in RViz (With ROS 2)

Visualize the robot model in RViz:

**Terminal 1 - Start RViz with URDF**:
```bash
source /opt/ros/humble/setup.bash
cd examples/module1/chapter3

# Launch RViz with the URDF model
# Note: This requires creating a launch file or using robot_state_publisher
# For now, we'll describe the manual steps below
```

**Manual steps to visualize in RViz**:

1. Start RViz:
   ```bash
   source /opt/ros/humble/setup.bash
   rviz2
   ```

2. In RViz:
   - Set Fixed Frame to `base_link`
   - Add a `RobotModel` display
   - In the RobotModel display settings, set the `Robot Description` parameter to the path of your URDF file

3. You should see the humanoid model appear in 3D

## Understanding the URDF File

### Link Definition

```xml
<link name="torso_link">
  <!-- Physics properties -->
  <inertial>
    <mass value="20.0"/>  <!-- 20 kg -->
    <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3"/>
  </inertial>

  <!-- Visual appearance for rendering -->
  <visual>
    <geometry>
      <cylinder radius="0.15" length="0.6"/>
    </geometry>
    <material name="white"/>
  </visual>

  <!-- Collision shape for planning -->
  <collision>
    <geometry>
      <cylinder radius="0.15" length="0.6"/>
    </geometry>
  </collision>
</link>
```

**Key concepts**:
- **Inertial**: How the link behaves in physics simulation
- **Visual**: How it looks in RViz (for visualization)
- **Collision**: For path planning and collision detection

### Joint Definition

```xml
<joint name="torso_to_left_shoulder" type="revolute">
  <!-- Parent and child links -->
  <parent link="torso_link"/>
  <child link="left_shoulder_link"/>

  <!-- Position and orientation offset -->
  <origin xyz="0.25 0.2 0.4" rpy="0 0 0"/>

  <!-- Axis of rotation (around Y axis) -->
  <axis xyz="0 1 0"/>

  <!-- Motion constraints -->
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  <!-- Range: -90 to +90 degrees, max torque: 10 Nm, max speed: 1 rad/s -->
</joint>
```

**Key concepts**:
- **Parent/Child**: Links that the joint connects
- **Origin**: Position and orientation relative to parent
- **Axis**: Direction of rotation (X, Y, or Z)
- **Limits**: Angular range, effort (torque), velocity

## Common Issues & Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| `FileNotFoundError: simple_humanoid.urdf not found` | Not in chapter3 directory | `cd examples/module1/chapter3` before running |
| `ParseError: XML syntax error` | Malformed URDF XML | Check for unclosed tags, proper nesting |
| `check_urdf: command not found` | ROS 2 tools not installed | Run `sudo apt install liburdfdom-tools` |
| Visualization shows nothing in RViz | RobotModel display not added | Add a new RobotModel display in RViz |
| "Transform accepted. ... frames" shows different number | Joint modifications | Re-validate after changing URDF |

## Modifying the URDF

### Example 1: Change Joint Limits

Edit `simple_humanoid.urdf` and modify a joint limit:

```xml
<!-- Original shoulder joint (90 degrees each way) -->
<limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>

<!-- Modified shoulder joint (120 degrees each way) -->
<limit lower="-2.09" upper="2.09" effort="10" velocity="1.0"/>
<!-- -2.09 to 2.09 radians = -120 to +120 degrees -->
```

Then validate:
```bash
check_urdf simple_humanoid.urdf
```

### Example 2: Change Link Mass

Edit link properties:

```xml
<!-- Make torso heavier -->
<mass value="30.0"/>  <!-- Was 20.0 kg -->
```

### Example 3: Change Geometry

```xml
<!-- Make torso thinner -->
<cylinder radius="0.10" length="0.6"/>  <!-- Was radius 0.15 -->
```

## Learning Path

1. **Start with visualize_urdf.py**: Understand URDF structure
2. **Review simple_humanoid.urdf**: See how links and joints are defined
3. **Run check_urdf**: Validate the model
4. **Modify joint limits**: Understand constraint effects
5. **Visualize in RViz**: See the complete 3D model

## Next Steps

After mastering this example:

### Immediate Modifications

1. **Add more joints**:
   - Split head into multiple neck joints
   - Add fingers/hand joints
   - Add waist (spine) joints

2. **Add sensors**:
   - Add camera link as child of head
   - Add IMU as fixed frame
   - Add pressure sensors in feet

3. **Improve geometry**:
   - Use mesh files instead of primitives
   - Add visual colors and materials
   - Add texture maps

### Advanced Topics (Module 2+)

- **Simulation**: Use URDF in Gazebo physics simulator
- **Planning**: Use URDF for motion planning algorithms
- **Control**: Calculate inverse kinematics using URDF structure
- **Real Robot**: Load URDF onto actual humanoid platform

## Resources

- [URDF XML Specification](https://wiki.ros.org/urdf/XML)
- [ROS 2 URDF Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [RViz User Guide](https://wiki.ros.org/rviz)
- [Gazebo URDF Documentation](http://gazebosim.org/tutorials?tut=build_model)
- Example robot URDFs:
  - [Boston Dynamics Atlas](https://github.com/openai/gym-fetch/tree/master/gym/envs/robotics/assets)
  - [Unitree A1](https://github.com/unitreerobotics/unitree_ros)
  - [PAL Robotics TALOS](https://github.com/pal-robotics/talos_robot)

## Glossary

| Term | Definition |
|------|-----------|
| **URDF** | Unified Robot Description Format (XML-based) |
| **Link** | Rigid body component of the robot |
| **Joint** | Connection between two links with motion constraints |
| **Revolute** | Joint that allows rotation (like a hinge) |
| **Fixed** | Joint with no motion (rigid connection) |
| **Kinematic Tree** | Hierarchical structure of links and joints |
| **Inertia** | Mass properties (resistance to motion) |
| **Visual Geometry** | Shape for rendering (RViz visualization) |
| **Collision Geometry** | Shape for collision detection and planning |

