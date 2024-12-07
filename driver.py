
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
        self.current_point = None

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_subscription = self.create_subscription(
            Point,
            '/bright_point1',
            self.goal_callback,
            10)
             
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/camera_pose',
            self.position_callback,
            10)
    
    def position_callback(self, msg):
        # Adjust the proportional drive with every callback
        self.current_point = msg

        if self.point is not None:
            # go to goal
            self.go_to_light()

    def goal_callback(self, msg):
        # set a point and remember it? Then repeat after some amount of time has elapsed?
        self.point = msg

    def go_to_light(self):
        #take in positional data and calculate angle and distance. Send twist commands until
        #both are zero or under a given margin

        if self.point is None or self.current_point is None:
            return

        cur_pose = self.current_point.pose.position
        cur_orientation = self.current_point.pose.orientation

        self.get_logger().info(f"Current Position: {cur_pose.x}, {cur_pose.y}, {cur_pose.z}")
        
        self.get_logger().info(f"Current Orientation: {cur_orientation.x}, {cur_orientation.y}, {cur_orientation.z}, {cur_orientation.w}")

        self.get_logger().info(f"Goal Position: {self.point.x}, {self.point.y}, {self.point.z}")

if __name__ == "__main__":
    rclpy.init()
    driver_node = Driver()
    rclpy.spin(driver_node)
    driver_node.destroy_node()
    rclpy.shutdown()
    



