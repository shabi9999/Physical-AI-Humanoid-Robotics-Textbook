# ROS 2 Examples

This directory contains runnable code examples for the ROS 2 Fundamentals for Humanoid Robotics course.

## Structure

```
examples/
└── module1/
    ├── chapter1/           # ROS 2 Core Concepts examples
    │   ├── hello_ros2.py           (To be created)
    │   ├── publisher.py             (To be created)
    │   ├── subscriber.py            (To be created)
    │   ├── service_server.py        (To be created)
    │   ├── service_client.py        (To be created)
    │   └── README.md                (Setup and execution guide)
    │
    ├── chapter2/           # Agent Bridging examples
    │   ├── simple_agent.py          (To be created)
    │   ├── sensor_bridge.py         (To be created)
    │   ├── control_publisher.py     (To be created)
    │   ├── mock_sensor.py           (To be created)
    │   └── README.md                (Setup and execution guide)
    │
    ├── chapter3/           # URDF Modeling examples
    │   ├── simple_humanoid.urdf     (To be created)
    │   ├── visualize_urdf.py        (To be created)
    │   ├── urdf_template.urdf       (To be created)
    │   └── README.md                (Setup and validation guide)
    │
    └── tests/              # Example validation tests
        ├── test_chapter1.py         (To be created)
        ├── test_chapter2.py         (To be created)
        └── test_chapter3.py         (To be created)
```

## Prerequisites

All examples assume:

- **ROS 2 Humble** installed on Ubuntu 22.04 LTS
- **Python 3.8+** available in your environment
- ROS 2 environment sourced: `source /opt/ros/humble/setup.bash`

### Installation

If you haven't installed ROS 2 Humble yet:

```bash
# Add ROS 2 repository
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release ubuntu-keyring

# Add ROS 2 GPG key
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -

# Add ROS 2 repository
sudo add-apt-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop-full

# Source setup script
source /opt/ros/humble/setup.bash
```

## Running Examples

### Chapter 1: ROS 2 Core Concepts

Each example in chapter1 runs independently. See chapter1/README.md for detailed instructions.

Quick start:
```bash
cd chapter1

# Terminal 1: Publisher
python3 publisher.py

# Terminal 2: Subscriber
python3 subscriber.py

# Terminal 3: Verify with CLI
ros2 topic list
ros2 topic echo /greeting
```

### Chapter 2: Agent Bridging

Examples demonstrate Python agents subscribing to sensors and publishing control commands.

See chapter2/README.md for detailed instructions.

### Chapter 3: URDF Modeling

Examples show how to create and visualize URDF models of humanoid robots.

See chapter3/README.md for detailed instructions.

## Testing

Run the example validation tests:

```bash
# Install pytest if not already installed
pip3 install pytest

# Run all tests
pytest tests/ -v

# Run specific test file
pytest tests/test_chapter1.py -v
```

## CI/CD Validation

All examples are automatically tested in ROS 2 Docker containers via GitHub Actions on every commit.

See `.github/workflows/test-examples.yml` for the CI/CD configuration.

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| `ModuleNotFoundError: rclpy` | Source ROS 2: `source /opt/ros/humble/setup.bash` |
| `Permission denied` | Make scripts executable: `chmod +x *.py` |
| Topic not showing up | Ensure publisher node is still running |
| Service call timeout | Check service server is running in another terminal |

## Contributing

When adding new examples:

1. Add code inline to the chapter markdown file
2. Place runnable script in the appropriate chapter directory
3. Add comments explaining each section
4. Include a test in the tests/ directory
5. Update the chapter README.md with instructions

## Resources

- [ROS 2 Official Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Python Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Humble Release Notes](https://docs.ros.org/en/humble/Releases/Release-Humble.html)
