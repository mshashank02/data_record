#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from phasespace_msgs.msg import Rigid
from phasespace_msgs.msg import Rigids
from rclpy.qos import qos_profile_sensor_data
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import time

class SynchronizeAndPublish(Node):

    def __init__(self):

        super().__init__('synchronise_and_publish')

        # Publisher for AckermannDriveStamped
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/synchronized_ackermann', 10)

        # Publisher for Markers
        self.rigids_publisher = self.create_publisher(Rigids, '/synchronized_rigids', 10)

        # Subscribers for Ackerman Command and Phasespace Markers
        ackermann_sub = Subscriber(self, AckermannDriveStamped, '/ackermann_cmd', qos_profile=qos_profile_sensor_data)
        rigids_sub = Subscriber(self, Rigids, '/phasespace/rigids', qos_profile=qos_profile_sensor_data)

        # Synchronize messages based on ROS time
        self.sync = ApproximateTimeSynchronizer([ackermann_sub, rigids_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

    def callback(self, ackermann_msg, rigids_msg):
        # Process Ackermann message
        steering_angle = ackermann_msg.drive.steering_angle
        self.get_logger().info(f"Received Ackermann message - Steering Angle: {steering_angle}")
        self.ackermann_publisher.publish(ackermann_msg)

        #Process Rigids message
        rigid_array = rigids_msg.rigids
        for rigid in rigid_array:
            print(rigid.id)
            print(rigid.x)
            print(rigid.y)
            print(rigid.qz)

        # Publish the synchronized Markers message
        self.rigids_publisher.publish(rigids_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SynchronizeAndPublish()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

