#!/usr/bin/env python3
"""
ROS 2 Topic Subscriber Example

This node subscribes to the 'greeting' topic and prints messages as they arrive.

Demonstrates:
1. Creating a subscriber
2. Using a callback function to handle messages
3. Asynchronous message reception

Run with:
    source /opt/ros/humble/setup.bash
    python3 subscriber.py

In another terminal, run the publisher:
    python3 publisher.py

Expected output from subscriber:
    [INFO] [subscriber_node]: Received: Hello from ROS 2! Message #0
    [INFO] [subscriber_node]: Received: Hello from ROS 2! Message #1
    ...
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    """Node that subscribes to messages from a topic."""

    def __init__(self):
        super().__init__('subscriber_node')

        # Create a subscriber to the 'greeting' topic
        # Parameters:
        #   - Message type: String (std_msgs.msg.String)
        #   - Topic name: 'greeting'
        #   - Callback function: self.message_callback
        #   - Queue size: 10 (max messages to buffer)
        self.subscription = self.create_subscription(
            String,
            'greeting',
            self.message_callback,
            10
        )

        # Prevent unused variable warning
        self.subscription

    def message_callback(self, msg):
        """
        Callback function called automatically when a message is received.

        This is called asynchronously - the node doesn't wait for messages,
        the callback is triggered when a message arrives.

        Args:
            msg: The received message (std_msgs.msg.String)
        """
        # Log the received message
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    """Entry point for the ROS 2 application."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node
    node = SubscriberNode()

    # Spin (keep running until Ctrl+C)
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
