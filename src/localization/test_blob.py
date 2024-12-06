import cv2
import numpy as np
from scipy import ndimage
from rclpy.node import Node
from geometry_msgs.msg import Point
import rclpy


class BlobDetector(Node):
    def __init__(self):
        super().__init__('blob_detector')  # Initialize the node
        self.publisher_ = self.create_publisher(Point, '/bright_point1', 10)  # Publisher for blob centroid

        # Camera Initialization
        self.cap = cv2.VideoCapture(6)  # Use your camera index
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 100)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

        self.timer = self.create_timer(0.1, self.process_frame)  # Process frames at 10 Hz

    def process_frame(self):
        """Process a single frame from the camera to detect the largest blob."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read from camera.")
            return

        # Convert frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Threshold to create a binary image (adjust threshold as needed)
        _, binary_frame = cv2.threshold(gray_frame, 200, 255, cv2.THRESH_BINARY)

        # Detect blobs using connected components
        labeled_array, num_spots = ndimage.label(binary_frame)

        if num_spots > 0:
            # Find the largest blob
            blob_sizes = ndimage.sum(binary_frame, labeled_array, range(1, num_spots + 1))
            largest_blob_idx = np.argmax(blob_sizes) + 1  # Labels are 1-based
            centroid = ndimage.center_of_mass(binary_frame, labeled_array, largest_blob_idx)

            # Publish the centroid as a Point message
            point = Point()
            point.x = float(centroid[1])  # X-coordinate in image space
            point.y = float(centroid[0])  # Y-coordinate in image space
            point.z = 0.0  # Assuming 2D

            self.publisher_.publish(point)

            # Log the detected centroid
            self.get_logger().info(f"Detected Brightest Blob Centroid: x={point.x}, y={point.y}")

            # Optional: Visualize the results
            cv2.circle(frame, (int(centroid[1]), int(centroid[0])), 5, (0, 0, 255), -1)
            cv2.imshow("Blob Detection", frame)
        else:
            self.get_logger().warning("No blobs detected in the frame.")

        # Display the frame (press 'q' to quit)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def destroy_node(self):
        """Release resources."""
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    blob_detector = BlobDetector()
    rclpy.spin(blob_detector)

    blob_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
