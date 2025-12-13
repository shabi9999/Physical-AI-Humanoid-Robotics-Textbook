# Chapter 1: ROS 2 Core Concepts - Code Examples

This directory contains runnable Python examples for Chapter 1 of the ROS 2 Fundamentals course.

## Files

- **hello_ros2.py** - Minimal ROS 2 node (prints "Hello from ROS 2!")
- **publisher.py** - Topic publisher (publishes greetings every 1 second)
- **subscriber.py** - Topic subscriber (receives and prints messages)
- **service_server.py** - Service server (AddTwoInts service)
- **service_client.py** - Service client (calls AddTwoInts service)

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

### Install ROS 2 Humble (if not already installed)

```bash
# Update package lists
sudo apt update

# Install ROS 2 Humble desktop
sudo apt install ros-humble-desktop-full

# Source the setup script
source /opt/ros/humble/setup.bash
```

## Running the Examples

### Example 1: Hello ROS 2

The simplest example - just logs a message.

```bash
python3 hello_ros2.py
```

**Expected Output**:
```
[INFO] [hello_node]: Hello from ROS 2!
```

### Example 2: Publisher & Subscriber

Demonstrates one-way communication using topics.

**Terminal 1 - Run Publisher**:
```bash
python3 publisher.py
```

Expected output:
```
[INFO] [publisher_node]: Published: Hello from ROS 2! Message #0
[INFO] [publisher_node]: Published: Hello from ROS 2! Message #1
...
```

**Terminal 2 - Run Subscriber**:
```bash
python3 subscriber.py
```

Expected output:
```
[INFO] [subscriber_node]: Received: Hello from ROS 2! Message #0
[INFO] [subscriber_node]: Received: Hello from ROS 2! Message #1
...
```

**Terminal 3 (Optional) - Inspect Topic**:
```bash
# List all active topics
ros2 topic list

# Show messages in real-time
ros2 topic echo /greeting

# Get topic information
ros2 topic info /greeting

# Show topic type
ros2 topic type /greeting

# Show message frequency and bandwidth
ros2 topic hz /greeting
ros2 topic bw /greeting
```

### Example 3: Service Client & Server

Demonstrates two-way communication using services.

**Terminal 1 - Run Service Server**:
```bash
python3 service_server.py
```

Expected output:
```
[INFO] [service_server_node]: Service /add_two_ints is ready
[INFO] [service_server_node]: Request: 5 + 3 = 8
[INFO] [service_server_node]: Request: 10 + 20 = 30
```

**Terminal 2 - Run Service Client**:
```bash
python3 service_client.py
```

Expected output:
```
[INFO] [service_client_node]: Waiting for service...
[INFO] [service_client_node]: Result: 8
```

**Terminal 3 (Optional) - Call Service from Command Line**:
```bash
# List all active services
ros2 service list

# Get service information
ros2 service info /add_two_ints

# Call the service with different values
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 100, b: 23}"
```

Expected response:
```
result:
  sum: 123
```

## Common Issues & Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| `ModuleNotFoundError: No module named 'rclpy'` | ROS 2 not sourced | Run `source /opt/ros/humble/setup.bash` |
| `Permission denied` | Script not executable | Run `chmod +x *.py` |
| `ros2: command not found` | ROS 2 not in PATH | Source ROS 2: `source /opt/ros/humble/setup.bash` |
| `Service not found` when calling | Service server not running | Keep service_server.py running in background |
| Subscriber shows no messages | Publisher not running yet | Start publisher first, then subscriber |
| `[WARN] Waiting for service...` keeps repeating | Service server hasn't started | Start service_server.py in another terminal |

## Learning Path

1. **Start with hello_ros2.py**: Understand basic node structure
2. **Move to publisher.py**: Learn about topics and one-way communication
3. **Add subscriber.py**: See how multiple nodes interact
4. **Try service_server.py**: Understand request/response patterns
5. **Run service_client.py**: Practice synchronous communication

## Next Steps

After mastering these examples:
- Try modifying the code (change topic names, message frequencies)
- Create your own node with both a publisher and subscriber
- Experiment with different message types (Int32, Float64, etc.)
- Read the full Chapter 1 content for deeper explanations
- Move to Chapter 2: Agent Bridging

## Resources

- [ROS 2 Official Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Python Client Library](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Topic CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [ROS 2 Service CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
