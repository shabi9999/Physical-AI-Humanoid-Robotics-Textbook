#!/usr/bin/env python3
"""
ROS 2 Service Server Example

This node provides a service that adds two integers.

Service Name: /add_two_ints
Service Type: AddTwoInts (example_interfaces.srv)
  Request: int64 a, int64 b
  Response: int64 sum

Demonstrates:
1. Creating a service server
2. Handling service requests
3. Returning responses
4. Synchronous request/response pattern

Run with:
    source /opt/ros/humble/setup.bash
    python3 service_server.py

Call the service in another terminal:
    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

Expected output from server:
    [INFO] [service_server_node]: Request: 5 + 3 = 8
    [INFO] [service_server_node]: Request: 10 + 20 = 30
    ...

Expected response:
    result:
      sum: 8
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceServerNode(Node):
    """Node that provides a service."""

    def __init__(self):
        super().__init__('service_server_node')

        # Create a service called 'add_two_ints'
        # Parameters:
        #   - Service type: AddTwoInts (example_interfaces.srv.AddTwoInts)
        #   - Service name: 'add_two_ints'
        #   - Callback function: self.add_ints_callback
        self.service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_ints_callback
        )

        # Log that service is available
        self.get_logger().info('Service /add_two_ints is ready')

    def add_ints_callback(self, request, response):
        """
        Callback function called when a service request is received.

        This function is synchronous - the client waits for the response.

        Args:
            request: The service request (contains fields a and b)
            response: The service response object (we set the sum field)

        Returns:
            response: The populated response object
        """
        # Perform the addition
        result = request.a + request.b

        # Set the response
        response.sum = result

        # Log the request and response
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')

        # Return the response to the client
        return response


def main(args=None):
    """Entry point for the ROS 2 application."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node
    node = ServiceServerNode()

    # Spin (keep running until Ctrl+C)
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
