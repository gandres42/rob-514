import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import rclpy


class BlobDetector(Node):
    def __init__(self):
        super().__init__('blob_detector')  # Initialize the node
        self.bridge = CvBridge()  # For converting ROS Image to OpenCV format
        self.publisher_ = self.create_publisher(Point, '/bright_point1', 10)  # Publisher for blob centroid
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Assuming this topic publishes the camera feed
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        """Process the incoming camera frame to detect the largest light blob."""
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Threshold to create a binary image
            _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

            # Find contours
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Find the largest contour
                largest_contour = max(contours, key=cv2.contourArea)

                # Calculate the moments of the largest contour
                moments = cv2.moments(largest_contour)

                if moments["m00"] != 0:  # Avoid division by zero
                    # Calculate the centroid of the blob
                    cx = int(moments["m10"] / moments["m00"])
                    cy = int(moments["m01"] / moments["m00"])

                    # Publish the centroid as a Point message
                    point = Point()
                    point.x = cx
                    point.y = cy
                    point.z = 0.0  # Assuming a 2D plane
                    self.publisher_.publish(point)

                    # Log the centroid
                    self.get_logger().info(f"Brightest Point Centroid: x={cx}, y={cy}")
                else:
                    self.get_logger().warning("Moments calculation failed, blob area is zero.")
            else:
                self.get_logger().warning("No blobs detected in the frame.")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    blob_detector = BlobDetector()
    rclpy.spin(blob_detector)

    blob_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
