#!/usr/bin/env python3
"""
ROS 2 Control Publisher Node - Agent Decisions to ROS 2

This node demonstrates the complete agent-ROS 2 bridging pattern:
1. Create an agent that makes autonomous decisions
2. Use a timer to periodically ask the agent for decisions
3. Publish those decisions as ROS 2 control commands

Key concepts:
1. Timer-based periodic publishing (synchronous pattern)
2. Publishing decisions from an agent to ROS 2 topics
3. Complete agent integration: agent logic → ROS 2 communication
4. Decoupling agent logic from ROS 2 infrastructure

Demonstrates:
- Creating a ROS 2 publisher (String messages)
- Creating a timer callback for periodic publishing
- Integrating SimpleAgent with timer-based control
- Complete bidirectional agent-ROS 2 communication

Run with:
    source /opt/ros/humble/setup.bash
    python3 control_publisher.py

In another terminal, listen to the published commands:
    ros2 topic echo /control_command

Expected output:
    [INFO] [control_publisher_node]: Control Publisher Node started
    [INFO] [control_publisher_node]: Publishing control: move_forward
    [INFO] [control_publisher_node]: Publishing control: move_forward
    [INFO] [control_publisher_node]: Publishing control: move_forward
    [INFO] [control_publisher_node]: Publishing control: turn_left
    ...
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Import the agent from the same directory
from simple_agent import SimpleAgent


class ControlPublisherNode(Node):
    """
    ROS 2 Node that publishes agent decisions as control commands.

    This node demonstrates the output side of agent-ROS 2 integration:
    - It creates an instance of SimpleAgent
    - It uses a timer to periodically ask the agent for decisions
    - It publishes those decisions to a ROS 2 topic (/control_command)

    This is the complement to SensorBridgeNode:
    - SensorBridgeNode: Sensor Input → Agent → Decision (logged)
    - ControlPublisherNode: Agent → ROS 2 Topic (for other nodes to use)

    Together, these nodes demonstrate how agents bridge to ROS 2:
    Sensor Data → [Agent] → Control Commands
    """

    def __init__(self):
        """
        Initialize the Control Publisher Node.

        Sets up:
        1. Node name: 'control_publisher_node'
        2. Publisher to /control_command topic (String messages)
        3. Instance of SimpleAgent for decision-making
        4. Timer to periodically publish decisions (500ms interval)
        """
        # Call parent class constructor with node name
        super().__init__('control_publisher_node')

        # Create a Python agent instance
        # This agent will make decisions (we'll ask it in the timer callback)
        self.agent = SimpleAgent()

        # Create a publisher for control commands
        # Parameters:
        #   - Message type: String (std_msgs.msg.String)
        #   - Topic name: 'control_command'
        #   - Queue size: 10 (buffer up to 10 messages if subscribers are slow)
        self.publisher = self.create_publisher(String, 'control_command', 10)

        # Create a timer that calls timer_callback every 0.5 seconds (500ms)
        # This is how we periodically ask the agent for decisions and publish them
        # Parameters:
        #   - Interval: 0.5 seconds (500ms = 2Hz publishing rate)
        #   - Callback: self.timer_callback (called at each interval)
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Log that the node has started
        self.get_logger().info('Control Publisher Node started')

    def timer_callback(self):
        """
        Timer callback called periodically (every 500ms).

        This function is called by ROS 2 at regular intervals. In each call:
        1. Ask the agent to make a decision
        2. Create a ROS 2 message with that decision
        3. Publish the message to the /control_command topic

        The timer callback pattern is useful for:
        - Periodic sensor polling
        - Regular state checks
        - Synchronous publish rates
        - Control loops at fixed frequencies
        """
        # Ask the agent to make a decision
        # In a full system, we'd pass current sensor data here.
        # For now, we're just asking the agent to use its internal state.
        decision = self.agent.make_decision()

        # Create a String message with the decision
        msg = String()
        msg.data = decision

        # Publish the message to /control_command topic
        # Any other ROS 2 nodes subscribed to this topic will receive it
        self.publisher.publish(msg)

        # Log what we're publishing (for debugging)
        self.get_logger().info(f'Publishing control: {decision}')


def main(args=None):
    """
    Entry point for the ROS 2 application.

    Initializes ROS 2, creates the node, spins (keeps running),
    and performs cleanup on shutdown.

    Args:
        args: Command-line arguments (passed to rclpy.init)
    """
    # Initialize ROS 2
    # This must be called once at the start of the program
    rclpy.init(args=args)

    # Create an instance of the ControlPublisherNode
    node = ControlPublisherNode()

    # Spin: keep the node alive and process callbacks
    # This blocks until the node is shut down (Ctrl+C)
    # While spinning, the timer callback will be called every 500ms,
    # triggering periodic publishing of agent decisions
    rclpy.spin(node)

    # Cleanup after shutdown
    # Destroy the node (close subscriptions, publishers, timers, etc.)
    node.destroy_node()

    # Shutdown ROS 2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
