
"""
This file holds the driver node which reads in the current pose and goal point, then drives the pupper towards the brightest point
"""
import numpy as np

import rclpy

from constants import aruco_position, mtx, dist
from geometry_msgs.msg import PoseStamped, Point, Twist
from networktables import NetworkTables
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from networktables.util import ntproperty

# Gross Hardcoded Server IP
IP = "10.214.154.192"
NetworkTables.initialize(server=IP)


class Driver(Node):
    def __init__(self):
        super().__init__('driver') # type: ignore
        self.point = None
        self.current_point = None

        self.goal_subscription = self.create_subscription(
            Point,
            '/bright_point',
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

        angles = Rotation.from_quat(cur_orientation).as_euler('zxy')

        sd = NetworkTables.getTable("PositionTable")
        sd.putNumber("pos_x", cur_pose.x)
        sd.putNumber("pos_y", cur_pose.y)
        sd.putNumber("pos_z", cur_pose.z)
        sd.putNumber("angle_z", angles[0])
        sd.putNumber("goal_x", self.point.x)
        sd.putNumber("goal_y", self.point.y)
        sd.putNumber("goal_z", self.point.y)

        self.get_logger().info(f"Current Position: {cur_pose.x}, {cur_pose.y}, {cur_pose.z}")
        
        self.get_logger().info(f"Current Orientation: {angles}")

        self.get_logger().info(f"Goal Position: {self.point.x}, {self.point.y}, {self.point.z}")
        
        

if __name__ == "__main__":
    rclpy.init()
    driver_node = Driver()
    rclpy.spin(driver_node)
    driver_node.destroy_node()
    rclpy.shutdown()
    



