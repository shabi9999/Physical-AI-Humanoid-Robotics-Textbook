#!/usr/bin/env python3
"""
ROS 2 Service Client Example

This node calls the 'add_two_ints' service with values 5 and 3.

Service Name: /add_two_ints
Service Type: AddTwoInts (example_interfaces.srv)

Demonstrates:
1. Creating a service client
2. Waiting for service to be available
3. Making an async service call
4. Handling the response

Run with:
    source /opt/ros/humble/setup.bash
    python3 service_client.py

First start the service server in another terminal:
    python3 service_server.py

Expected output from client:
    [INFO] [service_client_node]: Waiting for service...
    [INFO] [service_client_node]: Result: 8
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceClientNode(Node):
    """Node that calls a service."""

    def __init__(self):
        super().__init__('service_client_node')

        # Create a client for the 'add_two_ints' service
        # Parameters:
        #   - Service type: AddTwoInts (example_interfaces.srv.AddTwoInts)
        #   - Service name: 'add_two_ints'
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        # This blocks until the service is ready or timeout occurs
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        # Call the service with values 5 and 3
        self.send_request(5, 3)

    def send_request(self, a, b):
        """
        Send a request to the service.

        Args:
            a: First integer
            b: Second integer
        """
        # Create a request object
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call the service asynchronously
        # This returns a Future object
        future = self.client.call_async(request)

        # Register a callback to handle the response
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        """
        Callback function called when the service responds.

        Args:
            future: The Future object returned by call_async()
        """
        try:
            # Get the response from the future
            response = future.result()

            # Log the result
            self.get_logger().info(f'Result: {response.sum}')

            # Shutdown after getting response (optional)
            rclpy.shutdown()

        except Exception as e:
            # Handle errors
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    """Entry point for the ROS 2 application."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node
    node = ServiceClientNode()

    # Spin (keep running until callback shuts down)
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
