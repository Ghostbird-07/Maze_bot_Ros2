import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np


class MazeSolver(Node):
    def __init__(self):
        super().__init__("maze_solving_node")
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.videofeed_subscriber = self.create_subscription(
            Image, '/upper_camera/image_raw', self.get_video_feed_cb, 10)
        self.timer = self.create_timer(0.2, self.maze_solving)
        self.bridge = CvBridge()
        self.vel_msg = Twist()
        self.frame = None  # Store the latest frame

    def get_video_feed_cb(self, data):
        """ Callback to receive and process the video feed """
        self.frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.imshow("Maze View", self.frame)
        cv2.waitKey(1)

    def maze_solving(self):
        """ Maze-solving logic using line-following on a black path """

        if self.frame is None:
            return  # No frame received yet

        # Convert the frame to grayscale
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # Apply threshold to detect black maze path
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

        # Find contours of the path
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour (assumed to be the path)
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            if M["m00"] != 0:
                # Compute the centroid of the path
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Image center
                height, width = binary.shape
                center_x = width // 2

                # Steering logic: Adjust angular velocity based on centroid deviation
                error = center_x - cx  # Error is positive if the path is left
                self.vel_msg.linear.x = 0.1  # Move forward slowly
                self.vel_msg.angular.z = 0.002 * error  # Proportional control for turning

                # Debugging output
                self.get_logger().info(f"Centroid: ({cx}, {cy}), Error: {error}, Angular Z: {self.vel_msg.angular.z}")

                # Display centroid on the image
                cv2.circle(self.frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.imshow("Processed Frame", self.frame)
                cv2.waitKey(1)

        else:
            # Stop if no path is detected
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0

        # Publish velocity command
        self.velocity_publisher.publish(self.vel_msg)


def main(args=None):
    rclpy.init()
    node_obj = MazeSolver()
    rclpy.spin(node_obj)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
