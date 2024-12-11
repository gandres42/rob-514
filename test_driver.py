"""
This file holds the driver node which reads in the current pose and goal point, then drives the pupper towards the brightest point.
"""
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
import rclpy
import math
import time

# Control parameters
ALPHA = 2.0  # Proportional control for rotation
BETA = 0.8   # Proportional control for forward motion
PIXEL_THRESHOLD = 20  # Distance threshold in pixels

class Driver(Node):
    def __init__(self):
        super().__init__('driver')  # Initialize the node
        self.goal_point = None  # Goal point in pixels
        self.current_point = None  # Current pose in pixels and orientation
        self.current_pose = None 

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions for goal and current position
        self.goal_subscription = self.create_subscription(
            Point,
            '/bright_point',  # Placeholder for centroid topic
            self.goal_callback,
            10)

        self.current_point_subscription = self.create_subscription(
            Point,
            '/dog_point',  # Placeholder for pose topic
            self.point_callback,
            10)
        
        self.current_pose_subscription = self.create_subscription(
            PoseStamped,
            '/dog_pose',  # Placeholder for pose topic
            self.pose_callback,
            10)

    def point_callback(self, msg):
        """Callback for current position updates."""
        self.current_point = msg
        self.go_to_light()

    def pose_callback(self, msg):
        """Callback for current position updates."""
        self.current_pose = msg

    def goal_callback(self, msg):
        """Callback for goal updates."""
        self.goal_point = msg

    def go_to_light(self):
        """Navigate towards the light centroid."""
        if self.goal_point is None or self.current_point is None or self.current_pose is None:
            return

        # Extract current position and orientation
        yaw = self.current_pose.pose.orientation.z

        # Calculate distance to the light centroid
        dx = self.goal_point.x - self.current_point.x
        dy = self.goal_point.y - self.current_point.y
        distance_to_goal = np.sqrt(dx**2 + dy**2)

        # Calculate the angle to the light centroid
        angle_to_goal = np.arctan2(dx, dy)

        # Calculate the angle difference
        # angle_diff = self.normalize_angle(angle_to_goal - yaw)
        angle_diff = (angle_to_goal - yaw) + .8

        # Create Twist message
        twist = Twist()

        if distance_to_goal > PIXEL_THRESHOLD:
            if abs(angle_diff) > 1:  # Rotate towards the goal
                twist.angular.z = ALPHA * angle_diff
                twist.linear.x = 0.0
            else:  # Move towards the goal
                twist.linear.x = BETA * distance_to_goal
                twist.angular.z = 0.0
        else:
            # Stop when within the threshold
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish the Twist message
        self.publisher_.publish(twist)

        # time.sleep(10)

        # Log current status
        self.get_logger().info(f"Current position: {self.current_point.x}, {self.current_point.y}")
        self.get_logger().info(f"Distance to goal: {distance_to_goal}")
        self.get_logger().info(f"Angle: {yaw}")
        self.get_logger().info(f"Angle to goal: {angle_to_goal}")
        self.get_logger().info(f"Angle difference: {angle_diff}")

    def normalize_angle(self, angle):
        """Normalize angle to the range [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle + .8

def main(args=None):
    rclpy.init(args=args)
    driver_node = Driver()
    rclpy.spin(driver_node)
    driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
