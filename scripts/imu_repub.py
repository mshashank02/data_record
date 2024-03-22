#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vesc_msgs.msg import VescImuStamped
from vesc_msgs.msg import VescImu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
import math

class ImuToOdometry(Node):

    def __init__(self):

        super().__init__('imu_repub')

        #Subscriber for Vesc IMU msg
        self.subscription = self.create_subscription(
            VescImuStamped,
            '/sensors/imu',
            self.imu_callback,
            10)

        #Publisher for Odometry 
        self.imu_odom_publisher = self.create_publisher(Odometry, '/odom2',10)

    def imu_callback(self,msg):

            odom_msg = Odometry()
        

            odom_msg.pose.orientation.x = msg.imu.orientation.x
            odom_msg.pose.orientation.y = msg.imu.orientation.y
            odom_msg.pose.orientation.z = msg.imu.orientation.z
            odom_msg.pose.orientation.w = msg.imu.orientation.w

            odom_msg.twist.angular.x = msg.imu.angular_velocity.x
            odom_msg.twist.angular.y = msg.imu.angular_velocity.y
            odom_msg.twist.angular.z = msg.imu.angular_velocity.z

            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            self.imu_odom_publisher.publish(odom_msg)

def main(args=None):
     
    rclpy.init(args=args)
    imu_to_odom = ImuToOdometry()
    rclpy.spin(imu_to_odom)
    imu_to_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()

        

