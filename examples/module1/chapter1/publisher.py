#!/usr/bin/env python3
"""
ROS 2 Topic Publisher Example

This node publishes String messages to the 'greeting' topic every 1 second.

Demonstrates:
1. Creating a publisher
2. Using a timer to publish at regular intervals
3. Logging messages

Run with:
    source /opt/ros/humble/setup.bash
    python3 publisher.py

Monitor the topic in another terminal:
    ros2 topic echo /greeting

Expected output from publisher:
    [INFO] [publisher_node]: Published: Hello from ROS 2! Message #0
    [INFO] [publisher_node]: Published: Hello from ROS 2! Message #1
    ...
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherNode(Node):
    """Node that publishes messages to a topic."""

    def __init__(self):
        super().__init__('publisher_node')

        # Create a publisher to the 'greeting' topic
        # Parameters:
        #   - Message type: String (std_msgs.msg.String)
        #   - Topic name: 'greeting'
        #   - Queue size: 10 (max messages to buffer)
        self.publisher = self.create_publisher(String, 'greeting', 10)

        # Create a timer to publish every 1 second
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_message)

        # Counter for message numbering
        self.counter = 0

    def publish_message(self):
        """Callback function called by timer every 1 second."""
        # Create a String message
        msg = String()

        # Set the message data
        msg.data = f'Hello from ROS 2! Message #{self.counter}'

        # Publish the message
        self.publisher.publish(msg)

        # Log the published message
        self.get_logger().info(f'Published: {msg.data}')

        # Increment counter for next message
        self.counter += 1


def main(args=None):
    """Entry point for the ROS 2 application."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node
    node = PublisherNode()

    # Spin (keep running until Ctrl+C)
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
