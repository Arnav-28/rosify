#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TurtleJoyController(Node):
    def __init__(self):
        super().__init__('turtle_joy_controller')
        
        # Create publisher for turtle velocity commands
        self.turtle_vel_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        
        # Subscribe to joy topic
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Initialize message
        self.last_vel = Twist()
        
        # Controller settings
        self.linear_speed = 2.0
        self.angular_speed = 2.0
        
        # Axis mappings (may need adjustment based on your controller)
        self.linear_axis = 1  # Left stick vertical
        self.angular_axis = 0  # Left stick horizontal
        
        self.get_logger().info('Turtle Joy Controller Node Started')

    def joy_callback(self, joy_msg):
        # Create Twist message
        vel_msg = Twist()
        
        # Linear velocity (up/down)
        vel_msg.linear.x = self.linear_speed * joy_msg.axes[self.linear_axis]
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        
        # Angular velocity (left/right rotation)
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = self.angular_speed * joy_msg.axes[self.angular_axis]
        
        # Publish velocity command
        self.turtle_vel_pub.publish(vel_msg)
        self.last_vel = vel_msg

def main(args=None):
    rclpy.init(args=args)
    
    turtle_joy_controller = TurtleJoyController()
    
    try:
        rclpy.spin(turtle_joy_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the turtle before shutting down
        stop_msg = Twist()
        turtle_joy_controller.turtle_vel_pub.publish(stop_msg)
        
        # Cleanup
        turtle_joy_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()