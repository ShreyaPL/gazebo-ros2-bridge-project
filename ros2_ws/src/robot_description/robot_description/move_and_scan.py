import math
from typing import List, Optional
import sys
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def _min_valid_range(ranges: List[float]) -> Optional[float]:
    m = None
    for r in ranges:
        if r is None:
            continue
        if math.isfinite(r) and r > 0.0:
            m = r if (m is None or r < m) else m
    return m

class MoveandScan(Node):
    def __init__(self, linear,angular, rate_hz=10.0):
        super().__init__('move_and_scan')
        self.pub_twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_laser_scan = self.create_subscription(LaserScan, 'laser/scan', self.scan_cb, 10)

        # prepare twist
        self.twist = Twist()
        self.twist.linear.x = float(linear)
        self.twist.angular.z = float(angular)

        # publish at steady rate
        self.timer = self.create_timer(1.0/rate_hz, self.publish_cmd)

        self.last_min = None
        self.get_logger().info(
            f'Started Publishing cmd_vel at {rate_hz:0f} Hz '
            f'(linear = {linear:.3f} m/s, angular = {angular:.3f} rad/s '
            "Listening on /laser/scan for minimum distance"
        )
    
    def publish_cmd(self):
        self.pub_twist.publish(self.twist)
    
    def scan_cb(self, msg):
        m = _min_valid_range(msg.ranges)
        if m is not None and (self.last_min is None or abs(m - self.last_min) > 1e-3):
            self.last_min = m
            self.get_logger().info(f'Min laser distance: {m:.3f} m')

def main():
    parser = argparse.ArgumentParser(description="Move robot and print min LaserScan distance.")
    parser.add_argument('linear', type=float, help='Linear velocity (m/s)')
    parser.add_argument('angular', type=float, help='Angular velocity (rad/s)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MoveandScan(args.linear, args.angular)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
