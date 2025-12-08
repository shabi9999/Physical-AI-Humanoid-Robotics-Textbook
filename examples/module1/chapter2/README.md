# Chapter 2: Python Agent Bridging to ROS 2 - Code Examples

This directory contains runnable Python examples demonstrating how to integrate autonomous agents with ROS 2.

## Files

- **simple_agent.py** - Standalone Python agent with decision logic (no ROS 2 dependency)
- **sensor_bridge.py** - ROS 2 node that subscribes to sensor data and feeds it to an agent
- **control_publisher.py** - ROS 2 node that publishes agent decisions as control commands
- **README.md** - This file (setup and run instructions)

## Prerequisites

### Environment Setup

1. **Source ROS 2 Humble**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Verify Python 3.8+**:
   ```bash
   python3 --version
   ```

### Required Packages

- ROS 2 Humble
- Python 3.8+
- rclpy (included with ROS 2)
- std_msgs (included with ROS 2)

### Install ROS 2 Humble (if not already installed)

```bash
# Update package lists
sudo apt update

# Install ROS 2 Humble desktop
sudo apt install ros-humble-desktop-full

# Source the setup script
source /opt/ros/humble/setup.bash
```

## Understanding the Examples

### Architecture

These three examples demonstrate a complete agent-ROS 2 integration pattern:

```
Sensor Data → [Agent] → Control Commands
```

1. **simple_agent.py**: The agent logic (standalone, testable without ROS 2)
2. **sensor_bridge.py**: Receives sensor data from ROS 2 → feeds to agent
3. **control_publisher.py**: Asks agent for decisions → publishes to ROS 2

### Why This Pattern?

Separating agent logic from ROS 2 communication provides:
- **Testability**: Test agent offline without ROS 2
- **Reusability**: Use the same agent in different ROS 2 setups
- **Clarity**: Agent code focuses on decision-making, not ROS 2 plumbing
- **Modularity**: Easy to swap different agent implementations

## Running the Examples

### Example 1: Simple Agent (Standalone - No ROS 2 Needed)

The simplest example - test agent logic directly in Python:

```bash
python3 simple_agent.py
```

**Expected Output**:
```
=== Simple Agent Demo ===

Simulating sensor data and agent decisions:

Sensor:  2.5 | Decision: move_forward     | Count: 1
Sensor:  3.0 | Decision: move_forward     | Count: 2
Sensor:  4.5 | Decision: move_forward     | Count: 3
Sensor:  5.2 | Decision: move_forward     | Count: 4
Sensor:  6.8 | Decision: move_forward     | Count: 5
Sensor:  8.5 | Decision: turn_right       | Count: 6
Sensor:  3.0 | Decision: move_forward     | Count: 7
...
✓ Agent ran successfully without ROS 2 dependency
```

**What to Notice**:
- The agent runs completely independently
- No ROS 2 topics, no nodes, no middleware
- Decision logic is pure Python
- Good for testing logic before ROS 2 integration

---

### Example 2: Sensor Bridge (ROS 2 Input)

This example shows how to feed ROS 2 sensor data to an agent:

**Terminal 1 - Run Sensor Bridge Node**:
```bash
source /opt/ros/humble/setup.bash
python3 sensor_bridge.py
```

**Expected Output**:
```
[INFO] [sensor_bridge_node]: Sensor Bridge Node started
[INFO] [sensor_bridge_node]: Received sensor: 5.0
[INFO] [sensor_bridge_node]: Agent decision: move_forward
```

**Terminal 2 - Send Test Sensor Data**:
```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /sensor_data std_msgs/Float32 "{data: 5.0}" --once
```

**What to Notice**:
- Sensor Bridge subscribes to `/sensor_data` topic
- When a message arrives, the callback receives it
- The sensor value is passed to the agent
- Agent makes a decision based on the value
- Decision is logged

**Optional Terminal 3 - Inspect the Topic**:
```bash
# List all active topics
ros2 topic list

# Get topic information
ros2 topic info /sensor_data

# Show messages in real-time
ros2 topic echo /sensor_data

# Show message frequency
ros2 topic hz /sensor_data
```

---

### Example 3: Control Publisher (ROS 2 Output)

This example shows how to publish agent decisions as control commands:

**Terminal 1 - Run Control Publisher Node**:
```bash
source /opt/ros/humble/setup.bash
python3 control_publisher.py
```

**Expected Output**:
```
[INFO] [control_publisher_node]: Control Publisher Node started
[INFO] [control_publisher_node]: Publishing control: move_forward
[INFO] [control_publisher_node]: Publishing control: move_forward
[INFO] [control_publisher_node]: Publishing control: move_forward
[INFO] [control_publisher_node]: Publishing control: turn_left
[INFO] [control_publisher_node]: Publishing control: move_forward
```

**Terminal 2 - Listen to the Published Commands**:
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /control_command
```

**Expected output from echo**:
```
data: move_forward
---
data: move_forward
---
data: move_forward
---
data: turn_left
---
```

**What to Notice**:
- Control Publisher runs independently
- Every 500ms (0.5 seconds), it asks the agent for a decision
- Each decision is published to `/control_command` topic
- Any ROS 2 node can subscribe and receive these commands

---

### Example 4: Full Integration (All Three Together)

Run all three examples together to see the complete flow:

**Terminal 1 - Run Sensor Bridge**:
```bash
source /opt/ros/humble/setup.bash
python3 sensor_bridge.py
```

**Terminal 2 - Run Control Publisher**:
```bash
source /opt/ros/humble/setup.bash
python3 control_publisher.py
```

**Terminal 3 - Send Test Sensor Data**:
```bash
source /opt/ros/humble/setup.bash

# Send different sensor readings to see agent behavior change
ros2 topic pub /sensor_data std_msgs/Float32 "{data: 2.0}" --rate 1
```

**Terminal 4 - Monitor Published Commands**:
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /control_command
```

**What You're Seeing**:
- Sensor data flows in from Terminal 3
- Sensor Bridge (Terminal 1) receives it and logs the agent decision
- Control Publisher (Terminal 2) also asks the agent for decisions
- Commands flow out to Terminal 4

## Integration Pattern Explained

### Part 1: The Agent (simple_agent.py)

The `SimpleAgent` class is the **brain** of the system:
- Pure Python, no ROS 2 dependencies
- Takes sensor input, returns decision
- Can be tested offline
- Easy to modify decision logic

```python
agent = SimpleAgent()
decision = agent.process_sensor_data(5.0)  # Returns "move_forward", "turn_left", etc.
```

### Part 2: Input Bridge (sensor_bridge.py)

The `SensorBridgeNode` **pulls** data from ROS 2:
- Subscribes to `/sensor_data` topic
- Receives sensor messages asynchronously
- Feeds them to the agent
- Logs decisions

```
[ROS 2 Topic] → [Subscription Callback] → [Agent] → [Log]
```

### Part 3: Output Bridge (control_publisher.py)

The `ControlPublisherNode` **pushes** decisions to ROS 2:
- Uses a timer to regularly ask the agent for decisions
- Publishes decisions to `/control_command` topic
- Other ROS 2 nodes can subscribe and act on these commands

```
[Agent] → [Timer Callback] → [Publisher] → [ROS 2 Topic]
```

### Complete Flow

When all three run together:
```
Sensor Data
    ↓
[/sensor_data topic]
    ↓
[SensorBridgeNode callback] → [SimpleAgent] → [Log]
                                    ↓
                        (agent state is updated)
                                    ↓
[ControlPublisherNode timer] → [SimpleAgent] → [Publish]
    ↓
[/control_command topic]
    ↓
[Other ROS 2 nodes can subscribe]
```

## Common Issues & Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| `ModuleNotFoundError: No module named 'rclpy'` | ROS 2 not sourced | Run `source /opt/ros/humble/setup.bash` |
| `from simple_agent import SimpleAgent` fails | Not in chapter2 directory | Make sure you're in the correct directory |
| Sensor Bridge shows no output | No sensor data being published | Use Terminal 3 to send test data |
| `ros2: command not found` | ROS 2 not in PATH | Source ROS 2: `source /opt/ros/humble/setup.bash` |
| Control Publisher no output | ROS 2 not sourced | Run `source /opt/ros/humble/setup.bash` first |
| Topic echo shows no messages | Publisher not running | Make sure Control Publisher is running in another terminal |
| Permission denied on .py files | Files not executable | Run `chmod +x *.py` |

## Learning Path

1. **Start with simple_agent.py**: Understand agent logic independently
2. **Progress to sensor_bridge.py**: See how ROS 2 input feeds to agent
3. **Try control_publisher.py**: See how agent decisions become ROS 2 output
4. **Run all together**: See complete integration pattern

## Modifications to Try

### Modify Agent Logic

Edit `simple_agent.py` and change the decision logic:

```python
def make_decision(self):
    # Try different thresholds or logic
    if self.last_sensor_reading > 10.0:
        return "turn_right"
    elif self.decision_counter % 3 == 0:
        return "stop"
    else:
        return "move_forward"
```

Then re-run: `python3 control_publisher.py`

### Change Publishing Rate

Edit `control_publisher.py` and modify the timer interval:

```python
# Change from 0.5 seconds to 1.0 second (1Hz)
self.timer = self.create_timer(1.0, self.timer_callback)
```

### Change Topic Names

Edit `sensor_bridge.py` or `control_publisher.py` to use different topic names:

```python
# Change from 'sensor_data' to 'distance_sensor'
self.create_subscription(Float32, 'distance_sensor', self.sensor_callback, 10)
```

Then publish to the new topic:
```bash
ros2 topic pub /distance_sensor std_msgs/Float32 "{data: 5.0}" --once
```

## Next Steps

After mastering these examples:
- Create your own agent with more sophisticated logic
- Combine this with Chapter 1 patterns (services, multiple topics)
- Add sensor processing (low-pass filtering, averaging)
- Create a real-world obstacle avoidance agent
- Read Chapter 3 for URDF modeling to simulate the robot

## Resources

- [rclpy Official Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Subscriber-And-Publisher.html)
- [ROS 2 Topic Concept](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [Timer and Callbacks in ROS 2](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [ROS 2 Python Executor Pattern](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Colcon-To-Build-Packages.html)
