---
sidebar_position: 4
---

# Chapter 4: Nav2 Path Planning

## Overview

In this chapter, you'll learn how humanoid robots plan collision-free paths from their current location to a goal using **Nav2 (Navigation2)**—ROS 2's production-ready path planning system.

## What You'll Learn

- What path planning is and why it's essential for autonomous robots
- **Costmaps**: How robots represent obstacles and free space as a grid
- **Global Planning**: Finding optimal paths around obstacles (big picture)
- **Local Planning**: Avoiding obstacles in real-time (immediate reactions)
- **Nav2 Pipeline**: How all components work together
- Real-world scenarios: navigation with unexpected obstacles, dynamic replanning

## The Problem Nav2 Solves

A humanoid robot has explored a room (from Chapter 3: VSLAM) and knows:
- Its current position: (2.0, 3.0)
- Goal location: (15.0, 8.0)
- Map with obstacles (walls, furniture)

**Question**: How does it get from current position to goal safely?

**Simple approach** (naive):
```
Draw a straight line from (2, 3) to (15, 8)
Tell robot: "Walk forward"
→ Robot crashes into chair!
```

**Smart approach** (Nav2):
```
1. Global Planner: Plan a path that avoids all obstacles
   Output: (2,3) → (5,3) → (5,10) → (15,10) → (15,8)
2. Local Planner: Execute path while avoiding dynamic obstacles
   If person walks in front: Replan locally!
3. Controller: Send motor commands to follow the path
```

This is what Nav2 does!

## Costmaps: The Robot's Internal Representation

**Costmap** is a 2D grid where each cell represents whether it's:
- **Free** (robot can walk here)
- **Obstacle** (wall, furniture, person)
- **Unknown** (not yet explored)

### Simple Example

```
Costmap (top-down view of a room):

0 = free space (robot can walk)
X = obstacle (wall, furniture)
? = unknown

    0 1 2 3 4 5 6
  0 X X X X X X X
  1 X 0 0 0 0 0 X
  2 X 0 0 0 0 0 X
  3 X 0 0 0 X 0 X
  4 X 0 0 0 X 0 X
  5 X 0 X 0 0 0 X
  6 X X X X X X X

Robot starts at (1, 1) marked as R
Goal is at (5, 1) marked as G

R 0 0 0 0 G
```

### Creating a Costmap

The costmap comes from **Chapter 3: VSLAM**:
- VSLAM builds a 3D map with obstacles
- Convert to 2D top-down view
- Mark occupied cells as obstacles
- Mark free cells as walkable

```
3D VSLAM Map:
[point cloud with walls, chairs, obstacles]
↓
2D Projection (bird's eye view):
[walls become lines, obstacles become occupied cells]
↓
Costmap:
[0 0 0 X 0]
[0 X X X 0]
```

### Inflation Radius

To be safe, robots expand obstacles:

```
Original Obstacle:    Inflated Obstacle:
[0 X 0]              [X X X]
[X X X]              [X X X]
[0 X 0]              [X X X]

Why? Robot has physical width. Expanding the obstacle ensures
the robot doesn't graze obstacles with its body.
```

## Global Planning: Finding an Optimal Path

**Global planner** asks: "What's the best path from start to goal considering the map?"

### Path Planning Algorithms

Several algorithms exist. Here are the main ones:

#### Algorithm 1: Dijkstra's Algorithm

```
Start at (2, 3), Goal at (15, 8), Costmap with obstacles

Algorithm:
1. Mark start as "visited"
2. Expand to all neighbors, record distances:
   (1,3): distance 1
   (3,3): distance 1
   (2,2): distance 1
   (2,4): distance 1
3. Pick the neighbor with smallest distance
4. Expand from that neighbor
5. Repeat until you reach the goal

Result: Shortest path found!
```

**Advantages**: Guaranteed shortest path
**Disadvantages**: Slow for large maps (explores many cells)

#### Algorithm 2: A* (A-Star)

A* is like Dijkstra but smarter—it uses a **heuristic**:

```
Dijkstra explores all directions equally.

A* thinks: "Goal is to the northeast. Let me prioritize
exploring cells that are closer to the goal."

Heuristic: distance(current, goal) as the crow flies
```

**Result**: Finds the same shortest path as Dijkstra but **10-100x faster** by avoiding unnecessary exploration.

**Advantage**: Much faster than Dijkstra
**Disadvantage**: Requires heuristic (but distance-to-goal is simple and effective)

#### Algorithm 3: Rapidly-exploring Random Tree (RRT)

RRT uses randomization:

```
Start at (2, 3), Goal at (15, 8)

Algorithm:
1. Place a node at start
2. Pick a random point in the map
3. Find the closest node to that random point
4. Grow a branch toward the random point
5. Repeat hundreds of times
6. When a node gets close to the goal, connect to it

Result: A tree of paths (not necessarily optimal, but
found quickly)
```

**Advantage**: Works well in complex environments
**Disadvantage**: Path might not be optimal

### Algorithm Comparison

| Algorithm | Speed | Optimality | Best For |
|-----------|-------|-----------|----------|
| **Dijkstra** | Slow | Guaranteed shortest path | Small maps, when optimality matters |
| **A*** | Fast | Guaranteed shortest path | Most real-world applications (Nav2 default) |
| **RRT** | Very fast | Good (not optimal) | Complex, high-dimensional spaces |

### Nav2's Global Planner

Nav2 typically uses **A*** because it balances:
- **Speed**: Fast enough for real-time use
- **Optimality**: Finds short paths
- **Simplicity**: Easy to understand and debug

## Local Planning: Reacting to the Unexpected

**Problem with global planning alone**:

```
Global Plan: (2,3) → (5,3) → (5,10) → (15,10) → (15,8)

Robot executes:
  Move to (5, 3): ✓
  Move to (5, 10): But a PERSON is standing in the way!
  Global plan assumes static world, but world is dynamic!
```

**Local Planner** solves this:

```
Local Planner monitors:
  - Robot's current position
  - Costmap around robot (±2 meters)
  - Obstacles in the way

If obstacle appears:
  1. Slow down
  2. Calculate safe micro-paths around obstacle
  3. If possible: steer around obstacle
  4. If blocked: Signal to global planner to replan
```

### Dynamic Window Approach (DWA)

Nav2's default local planner:

```
Current Robot State:
  - Position: (5, 3)
  - Velocity: moving forward 0.5 m/s
  - Turning rate: 0 rad/s

DWA Algorithm:
1. Predict: "If I accelerate, what positions will I reach?"
   Position in 0.5 sec: (5.25, 3.0)
   Position in 1.0 sec: (5.50, 3.0)
   Position in 1.5 sec: (5.75, 3.0)

2. Simulate: "Will I collide with obstacles?"
   Check predicted positions against costmap
   (5.25, 3.0): ✓ free
   (5.50, 3.0): ✓ free
   (5.75, 3.0): ✗ obstacle ahead!

3. Adjust: "What velocity avoids collision?"
   Try turning left slightly: (5.22, 3.08) ✓
   Send: "Steer left, reduce forward speed"

4. Repeat every 50ms
```

## The Complete Nav2 Pipeline

Here's how everything fits together:

```
[VSLAM Input]
  ↓
  Robot pose (x, y, theta)
  Map with obstacles
  ↓
[Costmap Node]
  ↓
  Convert map to grid costmap
  Add inflation radius
  Update dynamic obstacles
  ↓
[Global Planner]
  ↓
  Input: start, goal, costmap
  Output: planned path (sequence of waypoints)
  Algorithm: A*
  ↓
[Local Planner (DWA)]
  ↓
  Input: current state, goal, costmap
  Output: velocity commands (linear + angular)
  ↓
[Controller]
  ↓
  Convert velocity to motor commands
  Send to humanoid robot's actuators
  ↓
[Robot Motion]
  ↓
  Robot follows path while avoiding obstacles
```

## Real-World Example: Robot Navigating an Office

**Scenario**: Humanoid robot must go from office A to office C (passing through hallway).

### Initial Plan

Global planner creates path:
```
Office A → Hallway → Office C

Visual:
[A] ——→ [Hallway] ——→ [C]
Start         midpoint      Goal
```

### Execution Phase 1: Hallway Clear

Robot moves forward along planned path. Local planner checks obstacles every 50ms—all clear.

```
Robot position: Office A entrance
Costmap: No obstacles ahead
Local planner: "Safe to move forward"
Action: Walk forward at 0.5 m/s
```

### Execution Phase 2: Dynamic Obstacle!

Person walks into hallway.

```
Robot position: Middle of hallway
Costmap updated: Obstacle detected!
Local planner: "Obstacle 1 meter ahead"
Action 1: Slow to 0.2 m/s
Action 2: Steer left to navigate around person
Action 3: Continue to goal
```

If local planner can't navigate around (blocked), it signals global planner:

```
"Path blocked. Replan!"
Global Planner: Recalculates using updated costmap
If alternate path exists: Replan
If no path exists: Stop and wait
```

### Execution Phase 3: Goal Reached

Robot reaches office C.

```
Robot position: Within 0.3m of goal
Local planner: "Goal reached"
Action: Stop motors
Status: Success!
```

## Real-World Challenges

### Challenge 1: Uncertain Costmaps

VSLAM costmap isn't perfect:

```
Reality: Wall at (10, 5)
VSLAM: Wall detected at (10.1, 5.2)  (10cm error)

Global planner might plan a path that's too close to wall.
Local planner detects actual wall and brakes!
```

**Solution**: Add safety margin (inflation radius larger than expected error).

### Challenge 2: Humans are Unpredictable

```
Local planner assumes: If person ahead, person walks away
Reality: Person blocks path and doesn't move

Robot behavior: Stops, waits, eventually signals "blocked"
```

**Solution**: Combine Nav2 with social navigation (adjust paths to give humans space).

### Challenge 3: Uneven Terrain

Nav2 assumes 2D grid costmap (flat ground).

For humanoid robots on stairs:

```
Standard Nav2: Can't handle 3D
Solution: Use 3D planning (more complex, slower)
Or: Limit navigation to flat terrain
```

## Comparison: Global vs. Local Planning

| Aspect | Global Plan | Local Plan |
|--------|---|---|
| **Time Horizon** | Full path (seconds to minutes) | Next few seconds |
| **Information** | Full map known | Immediate environment (±2m) |
| **Computation** | Done once (A* on full map) | Continuous (DWA every 50ms) |
| **Adaptability** | Static (only replans when blocked) | Dynamic (reacts every 50ms) |
| **Purpose** | Optimal route | Safe execution |

**Together**: Global plan = destination, Local plan = safe journey.

## What Happens Next

You've now learned all the components for a complete perception-and-navigation system:

1. **Chapter 1 (Isaac Sim)**: Training in simulation
2. **Chapter 2 (Synthetic Data)**: Generating training data
3. **Chapter 3 (VSLAM)**: Perceiving and localizing
4. **Chapter 4 (Nav2)**: Planning and navigating

The complete cycle: **Perceive → Localize → Plan → Execute**

In later modules (not yet published), you'll learn:
- **Module 4**: How voice commands trigger this entire pipeline
- **Module 5**: How to implement complete systems using ROS 2

## Key Takeaways

✓ **Costmaps** represent obstacles as a 2D grid (free, occupied, unknown)
✓ **Global Planning (A*)** finds optimal paths considering the complete map
✓ **Local Planning (DWA)** reacts to dynamic obstacles and ensures safe execution
✓ **Replanning** automatically recalculates when the world changes
✓ **Together, global + local planning** enable truly autonomous humanoid navigation
✓ **Nav2 is production-ready**: Used in hundreds of robots in the real world

## Complete System Understanding

You now understand the entire perception-localization-planning pipeline:

```
From VSLAM (Chapter 3):
  "I am at (5, 5) facing north, and here's the map"
        ↓
To Nav2 (Chapter 4):
  "I will navigate to (15, 15) safely"
        ↓
Result:
  Humanoid robot autonomously explores,
  localizes itself,
  plans paths,
  avoids obstacles,
  reaches goals
```

## Next Steps

You've completed **Module 3: The AI-Robot Brain**! You now understand:
- How to train perception in simulation
- How to generate realistic training data
- How robots localize themselves with cameras
- How robots plan and execute safe navigation

**Ready for more?** Upcoming modules will have you:
- Build complete systems using ROS 2
- Implement VSLAM and Nav2 from scratch
- Deploy on real humanoid robots
- Integrate with voice commands and AI agents

---

**Learning Outcome**: You now understand how robots independently perceive their environment, localize themselves, plan collision-free paths, and execute navigation in the real world.

**Congratulations on completing Module 3!** 🎉

You've learned the perception and navigation systems that enable humanoid robots to autonomously explore and interact with the world. These technologies are deployed in thousands of robots worldwide.

Next: Higher-level modules will teach you to orchestrate these systems with AI and voice commands!
