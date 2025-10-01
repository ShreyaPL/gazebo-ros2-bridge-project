#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DisplayOdometry(Node):
    def __init__(self, linear_vel: float, angular_vel: float):
        super().__init__('display_odom_mode')
        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Prepare message
        msg = Twist()
        msg.linear.x = linear_vel    # forward velocity (m/s)
        msg.angular.z = angular_vel  # yaw rate (rad/s)

        # Publish once
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published cmd_vel: linear={linear_vel}, angular={angular_vel}')

def main(args=None):
    rclpy.init(args=args)

    # Expect 2 command-line arguments
    if len(sys.argv) != 3:
        print("Usage: ros2 run <your_package> move_robot.py <linear_vel> <angular_vel>")
        return

    # Parse arguments
    linear = float(sys.argv[1])
    angular = float(sys.argv[2])

    node = DisplayOdometry(linear, angular)

    # Keep node alive just long enough to ensure publish goes out
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()