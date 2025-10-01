#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class DisplayImuNode(Node):
    def __init__(self):
        super().__init__('display_imu_node')
    
        self.cmd_publish = self.create_publisher(Twist, 'cmd_vel', 10)
        self.imu_subscribe = self.create_subscription(Imu, 'imu', self.imu_cb, qos_profile_sensor_data)

         # Publish at a steady rate (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_cmd)

        self.twist = Twist()
        self.twist.linear.x = 0.50
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.75
    
    def publish_cmd(self):
        self.cmd_publish.publish(self.twist)
    
    def imu_cb(self, imu_msg):
        orientation = imu_msg.orientation
        quat_x = orientation.x
        quat_y = orientation.y
        quat_z = orientation.z
        quat_w = orientation.w

        #linear acceleration
        linear_acceleration = imu_msg.linear_acceleration
        lin_acc_x = linear_acceleration.x

        # angular velocity
        angular_velocity = imu_msg.angular_velocity
        ang_vel_z = angular_velocity.z

        # print the range value
        self.get_logger().info("qX: %f, qY: %f" % (quat_x, quat_y))
        self.get_logger().info("qZ: %f, qW: %f" % (quat_z, quat_w))
        self.get_logger().info("LinAcc_X: %f" % (lin_acc_x))
        self.get_logger().info("AngVel_Z: %f" % (ang_vel_z))

def main(args=None):
    # initialize ROS2 communication
    rclpy.init(args=args)
    # initialize node
    display_imu_node = DisplayImuNode()
    # spin the node
    rclpy.spin(display_imu_node)
    # destroy the node
    display_imu_node.destroy_node()
    # shutdown ROS2 communication
    rclpy.shutdown()

if __name__ == "__main__":
    main()