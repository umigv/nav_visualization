#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random

class DummyPositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('grid_width', 20),
                ('grid_height', 20),
                ('update_rate', 1.0)
            ]
        )
        
        # Get parameters
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Create publisher
        self.publisher = self.create_publisher(Point, '/robot_position', 10)
        
        # Initial position
        self.current_x = self.grid_width // 2
        self.current_y = self.grid_height // 2
        
        # Create timer
        self.timer = self.create_timer(1.0/self.update_rate, self.publish_position)

    def publish_position(self):
        """Generate and publish new position"""
        new_x = self.current_x + random.randint(-2, 2)
        new_y = self.current_y + random.randint(-2, 2)
        
        new_x = max(0, min(self.grid_width - 1, new_x))
        new_y = max(0, min(self.grid_height - 1, new_y))
        
        self.current_x = new_x
        self.current_y = new_y
        
        msg = Point()
        msg.x = float(new_x)
        msg.y = float(new_y)
        msg.z = 0.0
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing position: ({new_x}, {new_y})')

def main(args=None):
    rclpy.init(args=args)
    publisher = DummyPositionPublisher()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()