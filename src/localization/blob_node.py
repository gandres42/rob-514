import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from constants import aruco_position, mtx, dist
import rclpy
from nav_msgs.msg import OccupancyGrid, MapMetaData

from blob import find_target, find_target_gradient



class Blob(Node):

    def __init__(self):
        super().__init__('blob_finder') # type: ignore
        self.rover_size = (20,20)
        self.rover_pos = None

        self.publisher_ = self.create_publisher(Point, '/bright_point', 10)
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/camera_pose',
            self.position_callback,
            10)
        self.pose_subscription  # prevent unused variable warning
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/light_grid',
            self.map_subscriber_callback,
            10)
        self.map_subscription  # prevent unused variable warning

    def position_callback(self, msg):
        self.rover_pos = (msg.pose.position.x, msg.pose.position.y)

    def map_subscriber_callback(self, msg):
        if self.rover_pos:
            img = np.array(msg.data).reshape((msg.info.width, msg.info.height))
            target = find_target(img, self.rover_pos, self.rover_size)
            point = Point()
            point.x = float(target[0])
            point.y = float(target[1])
            point.z = 0.0
            self.publisher_.publish(point)
        

if __name__ == "__main__":
    # ros setup
    rclpy.init()
    blob_node = Blob()

    rclpy.spin(blob_node)

    # Destroy the node
    blob_node.destroy_node()
    rclpy.shutdown()

