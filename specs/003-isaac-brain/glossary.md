# Module 3 Glossary: Key Terms & Definitions

**Purpose**: Define all key terminology used consistently across all 4 chapters
**Created**: 2025-12-08 | **Feature**: 003-isaac-brain

---

## Core Terms (Defined in Specification)

These 7 terms are defined in the specification and must be used consistently throughout Module 3:

### 1. Photorealistic Simulation

**Definition**: Computer simulation with high visual fidelity that closely mimics real-world lighting, materials, and physics for training perception models.

**First Introduced**: Chapter 1, Section 1.1 (What is Photorealistic Simulation?)

**Why Important**: Enables training of robust perception models by providing diverse, labeled training scenarios without real-world data collection costs.

**Related Terms**: Isaac Sim, domain adaptation, synthetic data

**Example**: Isaac Sim uses photorealistic rendering (reflections, shadows, material properties) to generate training images that look realistic to perception models.

**Context**: Contrasted with analytical simulation (simpler models, faster computation, lower visual fidelity).

---

### 2. Synthetic Data

**Definition**: Artificially generated training data from simulations rather than real-world capture.

**First Introduced**: Chapter 2, Section 1.1 (What is Synthetic Data?)

**Why Important**: Scales data collection, reduces costs, enables automatic labeling, allows controlled experimentation.

**Related Terms**: Photorealistic simulation, domain adaptation, training dataset, synthetic variation

**Example**: Instead of photographing 10,000 images of objects, Isaac Sim generates them automatically with random poses, lighting, and backgrounds.

**Context**: Addresses the data problem in machine learning: collecting real data is expensive, time-consuming, and sometimes dangerous.

---

### 3. Domain Adaptation

**Definition**: Techniques to bridge the gap between synthetic training data and real-world deployment (the "sim-to-real" problem).

**First Introduced**: Chapter 2, Section 1.2 (Domain Adaptation and Sim-to-Real Gap)

**Why Important**: Synthetic and real data differ (rendering vs. physics, controlled vs. messy), so models trained on synthetic data don't always work on real robots without adaptation.

**Related Terms**: Sim-to-real gap, domain randomization, synthetic data, transfer learning

**Example**: Domain randomization involves training on images with all possible lighting conditions, so the model learns features robust to any lighting in the real world.

**Context**: Central challenge in robotics: training in simulation is safe and efficient, but transferring to real robots is risky without proper adaptation.

---

### 4. VSLAM (Visual Simultaneous Localization and Mapping)

**Definition**: Simultaneously builds a map of the environment and estimates the robot's position using only visual information from a camera.

**First Introduced**: Chapter 3, Section 1.1 (What is VSLAM?)

**Why Important**: Enables mobile humanoid robots to navigate unknown environments with only a camera (no GPS, no pre-made map).

**Related Terms**: Localization, mapping, visual odometry, loop closure, Isaac ROS

**Example**: A robot enters a building it's never seen. VSLAM tracks visual features (corners, edges, textures) from frame to frame, estimating the robot's motion and building a real-time map.

**Context**: Fundamental technology for mobile robots; enables autonomy in GPS-denied environments (indoors, forests, underwater).

---

### 5. Costmap

**Definition**: A 2D grid representation where each cell indicates whether it's safe (low cost) or unsafe (high cost) for the robot to traverse.

**First Introduced**: Chapter 4, Section 1.1 (Costmaps and Obstacle Representation)

**Why Important**: Provides a common representation for path planners to understand the environment and avoid obstacles efficiently.

**Related Terms**: Grid map, occupancy grid, obstacle representation, planning, Nav2

**Example**: A costmap might represent a room as an 100×100 grid where empty space has cost 0, walls have cost 254, and unknown areas have cost 128.

**Context**: Core data structure in navigation; enables fast path planning by converting continuous maps into discrete grids.

---

### 6. Path Planning

**Definition**: Finding a collision-free path from a start location to a goal location.

**First Introduced**: Chapter 4, Section 1.2 (Path Planning Algorithms)

**Why Important**: Autonomous robots must navigate around obstacles without human input. Planning algorithms solve this efficiently.

**Related Terms**: Motion planning, trajectory, collision-free, goal, algorithm (Dijkstra, A*, RRT), Nav2

**Example**: Given a goal location and a map with obstacles, a path planner computes a route that avoids walls and other robots.

**Context**: Final step in the perception-localization-planning pipeline; executed continuously as the environment changes.

---

### 7. Trajectory Following

**Definition**: Executing a planned path by issuing motion commands to the robot's motors/actuators.

**First Introduced**: Chapter 4, Section 1.4 (Trajectory Following)

**Why Important**: Planning computes a path, but execution ensures the robot actually follows it while handling dynamic obstacles and physical constraints.

**Related Terms**: Path, motion commands, velocity, control, ROS 2 cmd_vel topic, feedback control

**Example**: A planned path says "move 1 meter forward, then turn 45 degrees left." Trajectory following issues motor commands to execute this motion while adjusting for slip, external forces, etc.

**Context**: Bridge between high-level planning and low-level robot control.

---

## Additional Terms (Used in Chapter Explanations)

These terms are used across the module but are explained within their respective chapters:

### Chapter 1 Terms

**Isaac Sim**: Industrial-grade photorealistic physics simulator from NVIDIA, built on Omniverse.

**Coordinate Frame**: A reference system for positions and rotations (e.g., world frame, robot base frame, camera frame).

**Physics Engine**: Software that simulates physical laws (gravity, collisions, dynamics) in a virtual environment.

**Sensor Simulation**: Synthetic generation of sensor data (images, depth, IMU) from the simulated environment.

**3D Scene**: Virtual environment containing 3D models, lights, and physics configurations.

---

### Chapter 2 Terms

**Domain Randomization**: Training technique that applies random variations (lighting, poses, backgrounds) to synthetic data to improve real-world robustness.

**Training Dataset**: Collection of data and labels used to train a machine learning model.

**Model Training**: Process of teaching a neural network or other ML model to recognize patterns in labeled data.

**Perception Model**: Trained model that identifies objects, landmarks, or features in images.

**Data Diversity**: Variation in synthetic data (different lighting, angles, scenarios) to cover real-world conditions.

---

### Chapter 3 Terms

**Visual Odometry**: Process of estimating robot motion (translation and rotation) by tracking visual features across consecutive camera frames.

**Feature Detection**: Identifying distinctive points (corners, edges, textured regions) in images that can be tracked reliably.

**Feature Matching**: Finding correspondences between features in consecutive images to track their movement.

**Ego-Motion Estimation**: Computing the robot's motion (how far/fast it moved, which direction) from feature displacements.

**Loop Closure Detection**: Recognizing when the robot returns to a previously visited location, reducing map drift.

**VSLAM Drift**: Accumulation of small odometry errors over time, causing the map and pose to diverge from reality.

**Isaac ROS**: NVIDIA's middleware suite providing hardware-accelerated perception and navigation packages for ROS 2.

---

### Chapter 4 Terms

**Global Planner**: Computes an optimal path from start to goal considering the full map (slower, strategic).

**Local Planner**: Computes immediate next steps to follow the global path while avoiding obstacles (faster, reactive).

**Collision-Free Path**: A trajectory that doesn't hit walls, obstacles, or other robots.

**Dynamic Obstacle**: Moving obstacle (person, other robot) not present in the map.

**Replanning**: Computing a new path when unexpected obstacles block the original route.

**Motion Controller**: System that converts planned trajectories into motor commands.

**Nav2**: ROS 2's navigation framework, including costmap, global/local planners, trajectory followers.

---

## Terminology Usage Rules

**For writers of Chapters 1-4**:

1. **First Use**: Define the term clearly on first introduction (in the section where it's first used)
   - Example: "**VSLAM** (Visual Simultaneous Localization and Mapping) is..."

2. **Subsequent Uses**: Use the term consistently (don't alternate between terms)
   - ✅ CORRECT: "VSLAM builds a map" (consistent term)
   - ❌ WRONG: "VSLAM builds a map... visual localization then maps..." (inconsistent)

3. **Cross-References**: Link back to glossary when introducing a term
   - Example: "See the glossary for definitions of key terms like..."

4. **Consistency Across Chapters**: Use the same definition and phrasing when a term appears in multiple chapters
   - All chapters use "VSLAM (Visual Simultaneous Localization and Mapping) is..."
   - All chapters use the same definition of "costmap"

5. **Avoid Synonyms**: Don't use multiple terms for the same concept
   - Pick one: either "path planning" OR "motion planning", not both
   - Pick one: either "pose" OR "position and orientation", not both

6. **Section 1 (Core Concepts)**: Defines most terms; subsequent sections can reference the definitions

---

## Terminology Checklist

When completing each chapter, verify:

- [ ] All 7 core terms are used consistently (if relevant to chapter)
- [ ] Terms are defined on first use
- [ ] No synonyms or alternative terms used inconsistently
- [ ] Chapter-specific terms are defined before use
- [ ] Glossary entries match chapter usage exactly
- [ ] Cross-references between chapters use same terminology
- [ ] No contradictions between chapter definitions

---

## Cross-Referencing Format

In chapter text, when referencing a glossary term:

**Format**: Use inline definition or link format

✅ **Option 1 (Inline)**: "Domain adaptation (techniques to bridge the sim-to-real gap) is crucial..."

✅ **Option 2 (Link to glossary)**: "Domain adaptation is crucial (see Glossary: Domain Adaptation)..."

✅ **Option 3 (Forward reference)**: "VSLAM is the focus of Chapter 3. For now, understand that it..."

---

## Glossary Update Procedure

As chapters are written and new terms emerge:

1. Add new terms to this glossary under "Additional Terms" section
2. Verify definition matches chapter usage
3. Note first chapter where term appears
4. Link related terms together
5. Update terminology checklist if new core terms needed

---

**Last Updated**: 2025-12-08
**Maintainer**: Module 3 content team
