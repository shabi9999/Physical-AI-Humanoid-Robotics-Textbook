#!/usr/bin/env python3
"""
Simple Autonomous Agent Example

This module demonstrates a basic AI agent that makes decisions based on input data.
The agent is separate from ROS 2 and can be tested independently.

Key concepts:
1. Agent as a reusable decision-making unit
2. Agent logic separate from ROS 2 communication
3. Deterministic decision-making based on sensor data

Demonstrates:
- Creating an agent class with decision logic
- Processing sensor readings
- Making decisions based on thresholds
- Generating control commands

Run with:
    python3 simple_agent.py

Expected output:
    Processing sensor: 2.5
    Decision: move_forward
    Processing sensor: 8.5
    Decision: turn_left
    ...
"""

class SimpleAgent:
    """
    A simple autonomous agent with basic decision logic.

    This agent demonstrates how to encapsulate decision-making logic
    separately from ROS 2 communication. The agent receives sensor data
    (floating-point values) and outputs control decisions (strings).
    """

    def __init__(self):
        """
        Initialize the agent with default state.

        Attributes:
            decision_counter: Tracks number of decisions made
            last_sensor_reading: Stores the most recent sensor value
        """
        self.decision_counter = 0
        self.last_sensor_reading = 0.0

    def process_sensor_data(self, sensor_reading):
        """
        Process incoming sensor data and make a decision.

        This method is the core of agent intelligence. It receives sensor
        data (e.g., distance to obstacle, sensor value) and returns a
        decision command.

        Args:
            sensor_reading (float): Raw sensor value from ROS 2 topic
                                   (e.g., distance in meters)

        Returns:
            str: Decision command ("move_forward", "turn_left", "turn_right", "stop")
        """
        # Store the sensor reading for later reference
        self.last_sensor_reading = sensor_reading

        # Make decision based on sensor threshold
        # This is threshold-based logic: a simple example of agent decision-making
        decision = self.make_decision()

        return decision

    def make_decision(self):
        """
        Generate a control decision based on current state.

        This method implements the agent's decision logic. In this simple example,
        we use a counter to demonstrate state-based decision-making. In a real
        application, this would use the sensor data to make intelligent decisions.

        Decision logic:
        - Every 5 decisions: turn left (exploration behavior)
        - Otherwise: move forward (default behavior)

        Returns:
            str: Control command to send to ROS 2
        """
        # Increment decision counter
        self.decision_counter += 1

        # Simple threshold-based decision: alternate behaviors
        if self.decision_counter % 5 == 0:
            # Every 5 decisions, turn left (exploration)
            return "turn_left"
        elif self.last_sensor_reading > 7.0:
            # If sensor reading is high, turn right (avoid obstacle)
            return "turn_right"
        else:
            # Default: move forward
            return "move_forward"

    def reset(self):
        """
        Reset agent state for new episode or scenario.

        Clears internal counters and state variables. Useful when the agent
        needs to start fresh in a new environment or scenario.
        """
        self.decision_counter = 0
        self.last_sensor_reading = 0.0

    def get_state(self):
        """
        Return current agent state for debugging or logging.

        Returns:
            dict: Current state including decision count and last sensor reading
        """
        return {
            "decision_count": self.decision_counter,
            "last_sensor": self.last_sensor_reading,
        }


def main():
    """
    Demonstrate agent functionality standalone (no ROS 2).

    This is how you would test the agent logic independently
    before integrating it with ROS 2 nodes.
    """
    # Create an instance of the agent
    agent = SimpleAgent()

    print("=== Simple Agent Demo ===\n")

    # Simulate sensor readings from an environment
    sensor_readings = [2.5, 3.0, 4.5, 5.2, 6.8, 8.5, 3.0, 2.1, 7.5, 9.2]

    print("Simulating sensor data and agent decisions:\n")

    # Process each sensor reading through the agent
    for reading in sensor_readings:
        # Pass sensor reading to agent
        decision = agent.process_sensor_data(reading)

        # Print the result
        print(f"Sensor: {reading:4.1f} | Decision: {decision:15} | Count: {agent.decision_counter}")

    print("\nFinal Agent State:")
    print(agent.get_state())
    print("\n[SUCCESS] Agent ran successfully without ROS 2 dependency")


if __name__ == "__main__":
    main()
