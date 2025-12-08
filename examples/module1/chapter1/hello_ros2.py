#!/usr/bin/env python3
"""
Hello ROS 2 - Minimal Node Example

This is the simplest possible ROS 2 node. It demonstrates:
1. Initializing ROS 2
2. Creating a node
3. Spinning (keeping the node running)
4. Logging a message

Run with:
    source /opt/ros/humble/setup.bash
    python3 hello_ros2.py

Expected output:
    [INFO] [hello_node]: Hello from ROS 2!
"""

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    """A minimal ROS 2 node that logs a message."""

    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('hello_node')

        # Log a message when the node starts
        self.get_logger().info('Hello from ROS 2!')


def main(args=None):
    """Entry point for the ROS 2 application."""
    # Initialize ROS 2 client library
    rclpy.init(args=args)

    # Create and instantiate the node
    node = HelloNode()

    # Keep the node running (spin) until interrupted (Ctrl+C)
    rclpy.spin(node)

    # Cleanup when shutting down
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
