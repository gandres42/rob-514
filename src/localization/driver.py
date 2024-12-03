
"""
This file holds the driver node which reads in the current pose and goal point, then drives the pupper towards the brightest point
"""
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
from constants import aruco_position, mtx, dist
import rclpy

ALPHA = 2
BETA = 0.8

class Driver(Node):
    def __init__(self):
        super().__init__('driver') # type: ignore
        self.point = None

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_subscription = self.create_subscription(
            Point,
            '/bright_point',
            self.goal_callback,
            10)
        self.goal_subscription      
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/camera_pose',
            self.position_callback,
            10)
        self.pose_subscription

    
    def position_callback(self, msg):
        # Adjust the proportional drive with every callback
        pass

    def goal_callback(self, msg):
        # set a point and rememeber it? Then repeat after some amount of time has elapsed?
        pass

if __name__ == "__main__":

    driver_node = Driver()
    rclpy.spin(driver_node)

    driver_node.destroy_node()
    rclpy.shutdown()
    



