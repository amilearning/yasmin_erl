#!/usr/bin/env python3

import time
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import random

class MockPublisher(Node):
    def __init__(self):
        super().__init__("mock_demo_publisher")

        self.goal_x = 2.0
        self.goal_y = 3.0
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.temp_pub = self.create_publisher(Float32, "/soil_temp", 10)

        self.timer = self.create_timer(0.1, self.publish_all)

        self.step = 0
        self.joint_pos = [0.0] * 6
        self.temp = 5.0

    def publish_all(self):
        # self.publish_odom()
        # self.publish_joints()
        self.publish_temp()

    def publish_odom(self):
        # Simulate moving toward the goal
        dist_step = 0.1
        angle = math.atan2(self.goal_y, self.goal_x)
        dx = dist_step * math.cos(angle)
        dy = dist_step * math.sin(angle)

        pos_x = min(self.goal_x, dx * self.step)
        pos_y = min(self.goal_y, dy * self.step)

        msg = Odometry()
        msg.pose.pose.position.x = pos_x
        msg.pose.pose.position.y = pos_y
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        self.odom_pub.publish(msg)

    def publish_joints(self):
        # Simulate arm joints reaching a fixed pose
        if self.step < 5:
            self.joint_pos = [0.0 + 0.02 * self.step] * 6
        elif self.step < 10:
            self.joint_pos = [0.2] * 6

        msg = JointState()
        msg.name = [f'joint_{i}' for i in range(6)]
        msg.position = self.joint_pos
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(msg)

    def publish_temp(self):
        # Every 5 seconds (0.1s * 50 steps), publish exactly 15.0
        if self.step % 30 == 0:
            self.temp = 10.0
        else:
            self.temp = random.uniform(10.0, 20.0)

        msg = Float32()
        msg.data = self.temp
        self.temp_pub.publish(msg)

        self.step += 1



def main(args=None):
    rclpy.init(args=args)
    node = MockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
