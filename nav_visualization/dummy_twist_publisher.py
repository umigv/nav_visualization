#!/usr/bin/env python3

"""
Twist Publisher for ROS2

This script publishes Twist messages to a specified ROS2 topic at a fixed interval.
It generates smooth, dynamic linear and angular velocity values using sine and cosine
functions to simulate realistic motion.

Features:
- Publishes to a configurable ROS2 topic (default: `/robot_twist`).
- Generates smooth motion using sine wave functions.
- Publishes messages at a fixed rate (default: 10 Hz).
- Logs each published message for debugging.

Usage:
1. Run a ROS2 environment.
2. Execute this script.
3. The node will start publishing Twist messages.
4. Use `ros2 topic echo /robot_twist` to view the published messages.
5. Stop the script with Ctrl+C.

Dependencies:
- ROS2 (rclpy)
- numpy
- math

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class TwistPublisher(Node):
    """
    A ROS2 node that publishes Twist messages at a fixed rate.

    The node:
    - Publishes linear and angular velocities using a sine function for smooth motion.
    - Runs at a configurable rate (default: 10 Hz).
    """

    def __init__(self):
        super().__init__('twist_publisher')

        # Declare and retrieve ROS2 parameters
        self.declare_parameter('topic', '/robot_twist')
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        # Create a publisher for the Twist message
        self.publisher_ = self.create_publisher(Twist, self.topic, 10)

        # Set the timer to publish messages at a fixed interval
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.publish_twist)

        # Time variable for generating smooth velocity variations
        self.time_elapsed = 0.0

    def publish_twist(self):
        """
        Generates and publishes a Twist message with dynamic linear and angular velocities.
        The values change smoothly over time using sine and cosine functions.
        """
        try:
            msg = Twist()

            # Generate smooth motion using sine wave functions
            msg.linear.x = 0.5 * math.sin(self.time_elapsed)  # Forward motion varies with sine
            msg.linear.y = 0.5 * math.cos(self.time_elapsed)  # Side motion varies with cosine
            msg.angular.z = 0.2 * math.sin(0.5 * self.time_elapsed)  # Rotational motion varies with a slower sine wave

            # Publish the message
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: linear=({msg.linear.x:.3f}, {msg.linear.y:.3f}), angular={msg.angular.z:.3f}')

            # Increment time for smooth variation
            self.time_elapsed += self.timer_period

        except Exception as e:
            self.get_logger().error(f"Error while publishing Twist message: {e}")

def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    node = TwistPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()