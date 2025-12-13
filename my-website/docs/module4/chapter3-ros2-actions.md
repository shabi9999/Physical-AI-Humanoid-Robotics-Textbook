---
title: "ROS 2 Action Integration"
module: 4
chapter: 3
id: "ch3-ros2-actions"
sidebar_position: 3
learning_objectives:
  - "Understand ROS 2 Action Servers and their lifecycle for long-running robot tasks"
  - "Recognize trajectory planning and execution concepts with feedback mechanisms"
  - "Apply action feedback for robust error handling and real-time adjustments"
prerequisites:
  - "Module 1: ROS 2 Fundamentals completed"
  - "Chapter 1-2: Speech and Planning completed"
related_chapters:
  - "chapter1-whisper"
  - "chapter2-llm-planning"
  - "chapter4-complete-vla"
keywords:
  - "ROS 2"
  - "actions"
  - "trajectory"
  - "execution"
  - "feedback"
  - "action server"
difficulty: "Intermediate"
estimated_reading_time: "18 minutes"
estimated_word_count: 5000
created_at: "2025-12-08"
chunk_count: 10
searchable_terms:
  - "action server"
  - "goal"
  - "feedback"
  - "result"
  - "trajectory"
  - "motion planning"
  - "ROS 2 actions"
  - "execution"
  - "waypoint"
  - "collision detection"
---

# Chapter 3: ROS 2 Action Integration

## What is an Action Server?

An **Action Server** is a ROS 2 component that executes long-running tasks with progress feedback.

**Difference from topics and services**:

| Component | Use Case | Example |
|-----------|----------|---------|
| **Topic** | Continuous data stream | Sensor readings (camera at 30 Hz) |
| **Service** | Request/response (fast) | "What time is it?" → Response in ms |
| **Action** | Goal with feedback (slow) | "Move arm to position" → Updates while executing |

## The Action Lifecycle

Actions follow a specific pattern:

```
1. Client sends GOAL: "Move gripper to (x=0.5, y=0.3, z=0.2)"
   ↓
2. Server ACCEPTS goal
   ↓
3. Server EXECUTES (plans trajectory, sends motor commands)
   While executing, sends FEEDBACK: "30% progress", "50% progress"
   ↓
4. Server finishes execution
   ↓
5. Server sends RESULT: "Success! Reached position"
   ↓
6. Client receives result
```

## Real-World Scenario: Robot Arm Reaching

Let's trace a complete action execution:

### Initial State
Robot arm at rest. User commands: "Move gripper to position (0.5, 0.3, 0.2)"

### Step 1: Send Goal (from LLM output)
```
Goal message:
{
  "target_position": {x: 0.5, y: 0.3, z: 0.2},
  "max_speed": 0.5,
  "timeout": 5.0
}
```

### Step 2: Action Server Accepts Goal

The Action Server checks if the goal is valid:
```
Server checks:
  ✅ Position reachable? YES
  ✅ No obstacles? YES
  ✅ Within joint limits? YES

Status: GOAL_ACCEPTED
```

### Step 3: Plan Trajectory
```
Motion planner computes:
  - Current position: (0.0, 0.0, 0.0)
  - Target position: (0.5, 0.3, 0.2)
  - Waypoints: [pt0, pt1, pt2, pt3] (smooth path)
  - Time to execute: 2 seconds
```

### Step 4: Execute with Feedback
```
Time 0.0s: Start moving
  Feedback: {"progress": 0%, "current_position": (0.0, 0.0, 0.0)}

Time 0.5s: Moving along trajectory
  Feedback: {"progress": 25%, "current_position": (0.125, 0.075, 0.05)}

Time 1.0s: Midway through
  Feedback: {"progress": 50%, "current_position": (0.25, 0.15, 0.1)}

Time 1.5s: Almost there
  Feedback: {"progress": 75%, "current_position": (0.375, 0.225, 0.15)}

Time 2.0s: Reached target
  Feedback: {"progress": 100%, "current_position": (0.5, 0.3, 0.2)}
```

### Step 5: Send Result
```
Result message:
{
  "success": true,
  "final_position": (0.5, 0.3, 0.2),
  "distance_error": 0.002,
  "execution_time": 2.05
}
```

## Action Server Implementation Basics

Here's what an Action Server does internally:

```python
class MoveArmAction:
    def __init__(self):
        self.server = ActionServer(
            node,
            MoveArm,
            'move_arm',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        # Extract goal
        target_position = goal_handle.request.target_position

        # Plan trajectory
        trajectory = self.plan_trajectory(target_position)

        # Execute with feedback
        for i, waypoint in enumerate(trajectory):
            # Send motor commands
            self.send_motor_command(waypoint)

            # Send progress feedback
            feedback_msg = MoveArm.Feedback()
            feedback_msg.progress = (i + 1) / len(trajectory)
            goal_handle.publish_feedback(feedback_msg)

            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MoveArm.Result(success=False)

            time.sleep(0.1)

        # Send final result
        result = MoveArm.Result()
        result.success = True
        result.final_position = self.get_current_position()
        goal_handle.succeed()
        return result
```

## Real-World Challenges

### Challenge 1: Obstacle Appears Mid-Execution

```
Robot executing: Move gripper from A to B
Trajectory: [A → pt1 → pt2 → B]

At pt1: COLLISION DETECTED!
  Someone placed obstacle in path

Action Server response:
  1. Stop arm immediately (safety)
  2. Send feedback: "Obstacle detected, replanning..."
  3. Replan around obstacle
  4. Continue execution
  5. Send result: "Success (replanned)"
```

### Challenge 2: Timeout (Movement Too Slow)

```
Goal: Move arm in 5 seconds
At 4.9 seconds: 95% done
At 5.0 seconds: 97% done (not finished!)
At 5.1 seconds: TIMEOUT

Action Server response:
  1. Cancel execution
  2. Send result: "Failed: timeout"
  3. Arm stops immediately
```

### Challenge 3: Joint Limit Violation

```
Goal: Move gripper to (0.5, 0.3, 2.0) meters

Arm's maximum reach: 1.8 meters (vertically)
Target is 2.0 meters up!

Action Server response:
  1. Check trajectory before execution
  2. Detect unreachable position
  3. Send result: "Failed: unreachable"
  4. Don't even attempt motion
```

## Trajectory Planning Concepts

Trajectory is a **time-parameterized path**:

```
Simple path: Just positions [A → B → C]
Trajectory: Positions + velocities + timing

Example:
  Waypoint 0: pos=(0,0,0), time=0.0s
  Waypoint 1: pos=(0.1,0,0), time=0.5s, velocity=(0.2,0,0)
  Waypoint 2: pos=(0.25,0,0), time=1.0s, velocity=(0.3,0,0)
  Waypoint 3: pos=(0.5,0.3,0.2), time=2.0s, velocity=(0,0,0)
```

Robot controller tracks this trajectory:
- At time 0.25s: Robot should be at pos ≈ (0.05, 0, 0)
- At time 0.75s: Robot should be at pos ≈ (0.175, 0, 0)

## Integration with Module 1 and Module 3

### ROS 2 Concepts (Module 1)

Action Servers build on core [ROS 2 concepts from Module 1](/module1/ch1-ros2-core):

- **[ROS 2 Actions](/module1/ch1-ros2-core)**: The foundation for this chapter
- **[Topics](/module1/ch1-ros2-core)**: Action servers publish feedback on internal topics
- **[Services](/module1/ch1-ros2-core)**: Actions extend services to add feedback
- **[Python Agents](/module1/ch2-agent-bridge)**: Coordinate multiple actions for complex tasks
- **[URDF Models](/module1/ch3-urdf-model)**: Define robot structure for trajectory validation

See [Module 1: ROS 2 Fundamentals](/module1/ch1-ros2-core) for detailed concepts.

### Perception Integration (Module 3)

Feedback loops connect to perception systems from [Module 3](/module3/intro):

**Closed-Loop Execution Pattern**:
```
Action Server executing motion
    ↓
Sends feedback: "Moving to position..."
    ↓
[VSLAM System (Module 3)](/module3/chapter3-vslam) provides localization feedback
    ↓
Compare planned vs actual position
    ↓
If error > threshold: Replan and adjust
    ↓
[Nav2 (Module 3)](/module3/chapter4-nav2) validates against obstacles
```

For detailed perception integration, see [Module 3: Perception Systems](/module3/intro).

---

## Action Server in the Complete VLA Pipeline

```
Chapter 1 (Whisper): Audio → Text
    "Pick up the blue object on the left"
              ↓
Chapter 2 (LLM): Text → Structured Action
    {action: pick_up, object: blue_object, location: left}
              ↓
Chapter 3 (ROS 2 Actions): Action → Execution

    Action Server:
    1. Accepts goal from LLM
    2. Plans trajectory (consider robot limits, obstacles)
    3. Executes waypoints with feedback
    4. Handles failures (timeout, collision)
    5. Returns final result
              ↓
Chapter 4 & Perception Feedback:
    Sensors (vision, force, VSLAM) confirm execution
    Loop back for next command or error recovery
```

**Action Server's role**: Reliably bridge semantic plans to physical motion execution

---

## Edge Cases: When Actions Fail

### Scenario 1: Gripper Force Feedback Detects Slip

```
Goal: Pick up object with force=50N
Action Server executing:
  - Closes gripper gradually
  - Monitors force sensor
  - At 35N: Force sensor shows slip!
  - Object is slipping out

Action Server response:
  1. Increase force to 60N
  2. Send feedback: "Gripper slip detected, increasing force"
  3. Continue with adjusted force
  4. Result: "Success with adjusted force"
```

**Key insight**: Robots don't execute plans blindly—they sense and adapt!

### Scenario 2: Partial Goal Achievement

```
Goal: Stack 3 blocks in tower
Trajectory: Pick block1 → place on table → pick block2 → stack on block1

Progress:
  ✅ Block 1 placed
  ⚠️ Block 2 picked, but...
  ❌ Block 1 fell over (external disturbance)

Action Server detects failed precondition:
  1. Cancel current action
  2. Send result: "Partial success: 1/3 stacked"
  3. Client must decide: retry from start or different strategy
```

### Scenario 3: Resource Contention

```
Goal: Move left arm to position X
Constraint: Right arm is also moving

Action Server checks:
  - Left arm path: Clear
  - But right arm path: CONFLICTS with left arm!

Response options:
  1. Serialize: Wait for right arm to finish
  2. Replan: Find collision-free path
  3. Reject: Send failure, let client decide
```

---

## Key Takeaways

✓ **Action Servers** execute long-running tasks with continuous feedback (unlike topics or services)
✓ **Lifecycle**: Goal → Accept → Execute (feedback) → Result (success/failure/timeout)
✓ **Feedback** enables monitoring, early stopping, and user awareness of progress
✓ **Challenges**: Obstacles, timeouts, unreachability, force limits, slip detection—all handled gracefully
✓ **Trajectories** are time-parameterized paths with positions, velocities, and accelerations
✓ **Safety checks** before execution validate reachability, joint limits, and collision freedom
✓ **Cancellation** possible at any time (immediate stop or graceful wind-down)
✓ **Integration**: Works with [Module 1 ROS 2 architecture](/module1/ch1-ros2-core) and [Module 3 perception](/module3/intro) for closed-loop control
✓ **Real-world systems** adapt based on sensor feedback—never execute blindly

---

## Next: Chapter 4

You now understand each component of the VLA system:
- **Chapter 1**: Whisper listens and transcribes
- **Chapter 2**: LLM understands and plans
- **Chapter 3**: Action Server executes with feedback

How do they work together in real time? In **[Chapter 4: Complete VLA Pipeline](/module4/ch4-complete-vla)**, you'll see the full system working together with perception loops and error recovery.

---

**Learning Outcome**: You now understand how ROS 2 Action Servers reliably execute motion commands with feedback, handle real-world challenges like obstacles and force limits, and integrate with perception systems for closed-loop control.
