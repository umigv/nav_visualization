#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        # Create publisher
        self.publisher_ = self.create_publisher(Twist, '/robot_twist', 10)
        
        # Create timer to publish messages
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_twist)
        self.time_elapsed = 0

    def publish_twist(self):
        """Publish dummy Twist data with varying linear and angular velocities."""
        msg = Twist()
        
        # Example twist values with a sine wave for smooth motion
        msg.linear.x = 0.5 * math.sin(self.time_elapsed)
        msg.linear.y = 0.5 * math.cos(self.time_elapsed)
        msg.angular.z = 0.2 * math.sin(0.5 * self.time_elapsed)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear=({msg.linear.x}, {msg.linear.y}), angular={msg.angular.z}')
        
        self.time_elapsed += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
