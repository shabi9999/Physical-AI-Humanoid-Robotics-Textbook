#!/usr/bin/env python3
"""
ROS 2 Sensor Bridge Node

This node demonstrates how to bridge a Python agent to ROS 2 by:
1. Subscribing to sensor data from a ROS 2 topic
2. Processing sensor data through an agent
3. Making decisions based on the agent's logic

Key concepts:
1. Callback-driven sensor reception (async pattern)
2. Integrating a Python agent with rclpy Node
3. ROS 2 topic subscription and message handling
4. Logging and debugging with rclpy.Node.get_logger()

Demonstrates:
- Creating a custom ROS 2 Node
- Subscribing to a topic with a callback function
- Feeding sensor data to an agent
- Logging decision information

Run with:
    source /opt/ros/humble/setup.bash
    python3 sensor_bridge.py

In another terminal, publish test data:
    ros2 topic pub /sensor_data std_msgs/Float32 "{data: 5.0}" --once

Expected output:
    [INFO] [sensor_bridge_node]: Sensor Bridge Node started
    [INFO] [sensor_bridge_node]: Received sensor: 5.0
    [INFO] [sensor_bridge_node]: Agent decision: move_forward
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Import the agent from the same directory
from simple_agent import SimpleAgent


class SensorBridgeNode(Node):
    """
    ROS 2 Node that subscribes to sensor data and bridges to an agent.

    This node demonstrates the first part of agent-ROS 2 integration:
    - It receives sensor data from a ROS 2 topic (asynchronously)
    - It feeds that data to a Python agent
    - It logs the agent's decision for debugging

    In the next example (ControlPublisherNode), we'll see how the agent's
    decisions are published back to ROS 2 as control commands.
    """

    def __init__(self):
        """
        Initialize the Sensor Bridge Node.

        Sets up:
        1. Node name: 'sensor_bridge_node'
        2. Subscription to /sensor_data topic (Float32 messages)
        3. Instance of SimpleAgent for decision-making
        """
        # Call parent class constructor with node name
        super().__init__('sensor_bridge_node')

        # Create a Python agent instance
        # This agent will process sensor data and make decisions
        self.agent = SimpleAgent()

        # Create a subscription to the /sensor_data topic
        # Parameters:
        #   - Message type: Float32 (std_msgs.msg.Float32)
        #   - Topic name: 'sensor_data'
        #   - Callback function: self.sensor_callback (called when message arrives)
        #   - Queue size: 10 (buffer up to 10 messages if processing is slow)
        self.subscription = self.create_subscription(
            Float32,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Prevent unused variable warning from linters
        self.subscription

        # Log that the node has started
        self.get_logger().info('Sensor Bridge Node started')

    def sensor_callback(self, msg):
        """
        Callback function called when a sensor message is received.

        This function is called asynchronously by the ROS 2 runtime whenever
        a message arrives on the /sensor_data topic. The callback pattern
        is central to ROS 2's event-driven architecture.

        Args:
            msg (std_msgs.msg.Float32): The received sensor message
                                       containing a float32 value in msg.data
        """
        # Extract the sensor value from the message
        sensor_value = msg.data

        # Log the received sensor reading
        self.get_logger().info(f'Received sensor: {sensor_value}')

        # Pass sensor data to agent and get decision
        decision = self.agent.process_sensor_data(sensor_value)

        # Log the agent's decision
        self.get_logger().info(f'Agent decision: {decision}')

        # In a full system, we would publish this decision to another ROS 2 topic
        # (see ControlPublisherNode for the complete example)


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

    # Create an instance of the SensorBridgeNode
    node = SensorBridgeNode()

    # Spin: keep the node alive and process callbacks
    # This blocks until the node is shut down (Ctrl+C)
    # While spinning, the subscription callback will be called whenever
    # a message arrives on /sensor_data
    rclpy.spin(node)

    # Cleanup after shutdown
    # Destroy the node (close subscriptions, publishers, etc.)
    node.destroy_node()

    # Shutdown ROS 2
    rclpy.shutdown()


if __name__ == '__main__':
    main()
