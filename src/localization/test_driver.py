"""
This file holds the driver node which reads in the current pose and goal point, then drives the pupper towards the brightest point.
"""
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
import rclpy

# Control parameters
ALPHA = 2.0  # Proportional control for rotation
BETA = 0.8   # Proportional control for forward motion
PIXEL_THRESHOLD = 100  # Distance threshold in pixels

class Driver(Node):
    def __init__(self):
        super().__init__('driver')  # Initialize the node
        self.point = None  # Goal point in pixels
        self.current_point = None  # Current pose in pixels and orientation

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions for goal and current position
        self.goal_subscription = self.create_subscription(
            Point,
            '/bright_point',  # Placeholder for centroid topic
            self.goal_callback,
            10)

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/camera_pose',  # Placeholder for pose topic
            self.position_callback,
            10)

    def position_callback(self, msg):
        """Callback for current position updates."""
        self.current_point = msg
        if self.point is not None:
            self.go_to_light()

    def goal_callback(self, msg):
        """Callback for goal updates."""
        self.point = msg

    def go_to_light(self):
        """Navigate towards the light centroid."""
        if self.point is None or self.current_point is None:
            return

        # Extract current position and orientation
        cur_pose = self.current_point.pose.position
        cur_orientation = self.current_point.pose.orientation

        # Convert quaternion to yaw
        yaw = self.quaternion_to_yaw(cur_orientation)

        # Calculate distance to the light centroid
        dx = self.point.x - cur_pose.x
        dy = self.point.y - cur_pose.y
        distance_to_goal = np.sqrt(dx**2 + dy**2)

        # Calculate the angle to the light centroid
        angle_to_goal = np.arctan2(dy, dx)

        # Calculate the angle difference
        angle_diff = self.normalize_angle(angle_to_goal - yaw)

        # Create Twist message
        twist = Twist()

        if distance_to_goal > PIXEL_THRESHOLD:
            if abs(angle_diff) > 0.1:  # Rotate towards the goal
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

        # Log current status
        self.get_logger().info(f"Distance to goal: {distance_to_goal}")
        self.get_logger().info(f"Angle to goal: {angle_to_goal}")
        self.get_logger().info(f"Angle difference: {angle_diff}")

    @staticmethod
    def quaternion_to_yaw(orientation):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
        return np.arctan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to the range [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    driver_node = Driver()
    rclpy.spin(driver_node)

    driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
