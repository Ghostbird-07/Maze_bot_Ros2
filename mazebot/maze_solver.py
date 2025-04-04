import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import math
import time

class MazeSolver(Node):
    def __init__(self):
        super().__init__("maze_solving_node")
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.videofeed_subscriber = self.create_subscription(
            Image, '/upper_camera/image_raw', self.get_video_feed_cb, 10)
        
        # Add parameters for easy tuning
        self.declare_parameter("linear_speed", 0.1)
        self.declare_parameter("angular_gain", 0.002)
        self.declare_parameter("threshold_value", 100)
        self.declare_parameter("debug_view", True)
        self.declare_parameter("bot_color_lower", [0, 0, 100])  # Default red lower bound in HSV
        self.declare_parameter("bot_color_upper", [10, 255, 255])  # Default red upper bound in HSV
        self.declare_parameter("junction_wait_time", 2.0)  # Time to wait at junctions in seconds
        
        # Create a timer for maze solving logic
        self.timer = self.create_timer(0.1, self.maze_solving)  # 10Hz for control loop
        
        # Initialize variables
        self.bridge = CvBridge()
        self.vel_msg = Twist()
        self.frame = None  # Store the latest frame
        self.processed_frame = None  # Store processed frame for visualization
        
        # Path tracking variables
        self.last_error = 0
        self.integral_error = 0
        self.path_detected = False
        self.consecutive_no_path = 0
        self.max_consecutive_no_path = 10  # Number of consecutive frames without path before stopping
        
        # Target point lookahead
        self.lookahead_ratio = 0.7  # Look ahead 70% of the frame height
        
        # Initialize state variables
        self.junction_detected = False
        self.at_junction = False
        self.junction_start_time = None
        self.direction_at_junction = None
        self.visited_junctions = set()  # Track visited junctions
        
        # Bot tracking
        self.bot_position = None
        self.bot_orientation = 0.0
        
        self.get_logger().info("Maze Solver Node initialized")
    
    def get_video_feed_cb(self, data):
        """ Callback to receive and process the video feed """
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            
            # Show raw feed if debug view is enabled
            if self.get_parameter("debug_view").value:
                cv2.imshow("Maze View", self.frame)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing video feed: {e}")
    
    def preprocess_image(self, frame):
        """Process image to extract the maze path"""
        # Make a copy of the frame
        processed = frame.copy()
        
        # Convert the frame to grayscale
        gray = cv2.cvtColor(processed, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply threshold to detect black maze path
        threshold_value = self.get_parameter("threshold_value").value
        _, binary = cv2.threshold(blurred, threshold_value, 255, cv2.THRESH_BINARY_INV)
        
        # Apply morphological operations to clean up the binary image
        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        return binary, processed
    
    def detect_bot(self, frame):
        """Detect the robot in the frame using color segmentation"""
        try:
            # Convert to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Get the color thresholds from parameters
            lower_bound = np.array(self.get_parameter("bot_color_lower").value)
            upper_bound = np.array(self.get_parameter("bot_color_upper").value)
            
            # Create a mask for the bot color
            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            
            # Apply morphological operations to clean the mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                return None, None, None
            
            # Find the largest contour (assumed to be the bot)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # If contour area is too small, ignore it
            if cv2.contourArea(largest_contour) < 50:
                return None, None, None
            
            # Calculate centroid
            M = cv2.moments(largest_contour)
            if M["m00"] == 0:
                return None, None, None
                
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Calculate orientation (using minimum enclosing rectangle)
            rect = cv2.minAreaRect(largest_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            # Return the contour, position, and box points for visualization
            return largest_contour, (cx, cy), box
            
        except Exception as e:
            self.get_logger().error(f"Error detecting bot: {e}")
            return None, None, None
    
    def detect_path(self, binary):
        """Detect the maze path and extract its properties"""
        height, width = binary.shape
        center_x = width // 2
        
        # Find contours of the path
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Check if we have any contours
        if not contours:
            self.path_detected = False
            return None, center_x, height, width
        
        # Find the largest contour (assumed to be the path)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # If the contour area is too small, consider it noise
        if cv2.contourArea(largest_contour) < 100:
            self.path_detected = False
            return None, center_x, height, width
        
        self.path_detected = True
        return largest_contour, center_x, height, width
    
    def detect_junctions(self, binary):
        """Detect junctions in the maze path"""
        # Check if cv2.ximgproc is available
        try:
            # Try using skeletonization if available
            skeleton = cv2.ximgproc.thinning(binary)
            
            # Find junction points (points with more than 2 neighbors)
            junction_points = []
            for i in range(1, skeleton.shape[0]-1):
                for j in range(1, skeleton.shape[1]-1):
                    if skeleton[i, j] == 255:
                        # Check 8-connected neighbors
                        neighbors = np.sum(skeleton[i-1:i+2, j-1:j+2] == 255) - 1
                        if neighbors > 2:
                            junction_points.append((j, i))  # x, y format
            
        except AttributeError:
            # Alternative method if ximgproc is not available
            # Use Harris corner detector to find potential junction points
            corners = cv2.cornerHarris(binary, 5, 3, 0.04)
            corners = cv2.dilate(corners, None)
            threshold = 0.01 * corners.max()
            junction_points = []
            
            # Get coordinates of points where corners > threshold
            y_coords, x_coords = np.where(corners > threshold)
            
            # Create clusters of corner points (junctions)
            if len(x_coords) > 0:
                # Use a simple distance-based clustering
                from sklearn.cluster import DBSCAN
                points = np.column_stack((x_coords, y_coords))
                clustering = DBSCAN(eps=10, min_samples=3).fit(points)
                
                # Get cluster centers
                unique_labels = set(clustering.labels_)
                for label in unique_labels:
                    if label == -1:  # Skip noise
                        continue
                    mask = clustering.labels_ == label
                    x_mean = int(np.mean(points[mask, 0]))
                    y_mean = int(np.mean(points[mask, 1]))
                    junction_points.append((x_mean, y_mean))
        
        return junction_points
    
    def is_bot_at_junction(self, junctions, bot_position):
        """Check if the bot is at a junction"""
        if not junctions or not bot_position:
            return False
        
        # Check if the bot is close to any junction
        for junction in junctions:
            distance = math.dist(junction, bot_position)
            if distance < 30:  # Adjust threshold based on your setup
                return True
        
        return False
    
    def calculate_target_point(self, contour, center_x, height, width):
        """Calculate target point on the path using lookahead"""
        # Create a mask from the contour
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(mask, [contour], 0, 255, -1)
        
        # Find points on the path at the lookahead distance
        target_y = int(height * self.lookahead_ratio)
        
        # Find all white points in the row at target_y
        white_points = np.where(mask[target_y, :] == 255)[0]
        
        if len(white_points) == 0:
            # No path at the lookahead distance, use the centroid
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return cx, cy
            return center_x, height // 2
        
        # Find the point closest to the center
        target_x = white_points[np.abs(white_points - center_x).argmin()]
        return target_x, target_y
    
    def calculate_steering(self, target_x, center_x):
        """Calculate steering using PID control"""
        # Calculate error (positive if path is to the left)
        error = center_x - target_x
        
        # PID control parameters
        kp = self.get_parameter("angular_gain").value
        kd = 0.0002  # Derivative gain
        ki = 0.00001  # Integral gain
        
        # Calculate PID terms
        p_term = kp * error
        d_term = kd * (error - self.last_error)
        self.integral_error += error
        i_term = ki * self.integral_error
        
        # Calculate angular velocity
        angular_z = p_term + d_term + i_term
        
        # Update last error
        self.last_error = error
        
        return error, angular_z
    
    def handle_junction(self, junctions, binary, bot_position):
        """Handle maze junctions with stopping"""
        if not junctions or not bot_position:
            self.junction_detected = False
            return
        
        # Check if bot is at a junction
        bot_at_junction = self.is_bot_at_junction(junctions, bot_position)
        
        if bot_at_junction and not self.at_junction:
            # Bot just arrived at a junction
            self.at_junction = True
            self.junction_start_time = time.time()
            
            # Stop the robot
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(self.vel_msg)
            
            self.get_logger().info(f"Bot stopped at junction at position {bot_position}")
            
            # Find the closest junction
            closest_junction = min(junctions, key=lambda j: math.dist(j, bot_position))
            
            # Convert junction to tuple for set operations
            junction_tuple = (closest_junction[0], closest_junction[1])
            
            # Check if this is a new junction
            is_new_junction = True
            for visited in self.visited_junctions:
                # Check if we're close to a previously visited junction
                if math.dist(junction_tuple, visited) < 20:
                    is_new_junction = False
                    break
            
            if is_new_junction:
                self.visited_junctions.add(junction_tuple)
                self.junction_detected = True
                
                # Apply right-hand rule (or other maze solving algorithm)
                # For right-hand rule, we prioritize: Right, Straight, Left
                
                # Get available directions at junction
                height, width = binary.shape
                directions = []
                
                # Check for paths in different directions (right, straight, left)
                if closest_junction[0] + 20 < width and np.any(binary[closest_junction[1], closest_junction[0]+10:closest_junction[0]+20] == 255):
                    directions.append("right")
                if closest_junction[1] + 20 < height and np.any(binary[closest_junction[1]+10:closest_junction[1]+20, closest_junction[0]] == 255):
                    directions.append("straight")
                if closest_junction[0] - 20 >= 0 and np.any(binary[closest_junction[1], closest_junction[0]-20:closest_junction[0]-10] == 255):
                    directions.append("left")
                
                # Choose direction based on right-hand rule
                if "right" in directions:
                    self.direction_at_junction = "right"
                    self.get_logger().info("Taking right turn at junction")
                elif "straight" in directions:
                    self.direction_at_junction = "straight"
                    self.get_logger().info("Going straight at junction")
                elif "left" in directions:
                    self.direction_at_junction = "left"
                    self.get_logger().info("Taking left turn at junction")
        
        # Check if we should resume movement after waiting at the junction
        if self.at_junction:
            junction_wait_time = self.get_parameter("junction_wait_time").value
            if time.time() - self.junction_start_time >= junction_wait_time:
                self.at_junction = False
                self.get_logger().info(f"Resuming movement after junction wait with direction: {self.direction_at_junction}")
    
    def maze_solving(self):
        """ Maze-solving logic using line-following on a black path """
        if self.frame is None:
            self.get_logger().info("No frame received yet")
            return

        try:
            # Preprocess the image to extract the path
            binary, processed_frame = self.preprocess_image(self.frame)
            
            # Detect the bot
            bot_contour, bot_position, bot_box = self.detect_bot(self.frame)
            self.bot_position = bot_position
            
            # Detect the path
            contour, center_x, height, width = self.detect_path(binary)
            
            # Detect junctions
            junctions = self.detect_junctions(binary)
            
            # Handle junctions and check if bot is at a junction
            if bot_position:
                self.handle_junction(junctions, binary, bot_position)
            
            # If we're at a junction, don't move until wait time is up
            if self.at_junction:
                # Keep robot stopped
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(self.vel_msg)
                
                # Display debugging info
                if self.get_parameter("debug_view").value:
                    self.display_debug_info(processed_frame, binary, contour, bot_contour, bot_position, bot_box, junctions, None, None)
                return
            
            # Check if path is detected
            if not self.path_detected:
                self.consecutive_no_path += 1
                if self.consecutive_no_path >= self.max_consecutive_no_path:
                    # Stop the robot if path is lost for too many frames
                    self.vel_msg.linear.x = 0.0
                    self.vel_msg.angular.z = 0.0
                    self.velocity_publisher.publish(self.vel_msg)
                    self.get_logger().warn("Path lost, stopping robot")
                
                # Display debugging info
                if self.get_parameter("debug_view").value:
                    self.display_debug_info(processed_frame, binary, contour, bot_contour, bot_position, bot_box, junctions, None, None)
                return
            
            # Reset the counter since path is detected
            self.consecutive_no_path = 0
            
            # Calculate target point on the path
            target_x, target_y = self.calculate_target_point(contour, center_x, height, width)
            
            # Calculate steering
            error, angular_z = self.calculate_steering(target_x, center_x)
            
            # Set velocity based on path following
            linear_speed = self.get_parameter("linear_speed").value
            
            # Adjust speed based on curve sharpness
            # Slow down in sharp curves
            curve_factor = 1.0 - min(1.0, abs(angular_z) / 0.5)
            adjusted_speed = linear_speed * max(0.3, curve_factor)
            
            self.vel_msg.linear.x = adjusted_speed
            self.vel_msg.angular.z = angular_z
            
            # Override control if at a junction and we have a direction
            if self.junction_detected and self.direction_at_junction:
                if self.direction_at_junction == "right":
                    self.vel_msg.linear.x = linear_speed * 0.7
                    self.vel_msg.angular.z = -0.5  # Turn right
                elif self.direction_at_junction == "left":
                    self.vel_msg.linear.x = linear_speed * 0.7
                    self.vel_msg.angular.z = 0.5  # Turn left
                # For "straight", use the normal control
            
            # Publish velocity command
            self.velocity_publisher.publish(self.vel_msg)
            
            # Debugging output
            self.get_logger().debug(f"Target: ({target_x}, {target_y}), Error: {error}, Angular Z: {self.vel_msg.angular.z}")
            
            # Display debugging info
            if self.get_parameter("debug_view").value:
                self.display_debug_info(processed_frame, binary, contour, bot_contour, bot_position, bot_box, junctions, target_x, target_y)
                
        except Exception as e:
            self.get_logger().error(f"Error in maze_solving: {e}")
            # Stop the robot if there's an error
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(self.vel_msg)
    
    def display_debug_info(self, processed_frame, binary, path_contour, bot_contour, bot_position, bot_box, junctions, target_x, target_y):
        """Display debug information"""
        # Create a copy of the frame for visualization
        debug_frame = processed_frame.copy()
        height, width = binary.shape
        center_x = width // 2
        
        # Draw center line
        cv2.line(debug_frame, (center_x, 0), (center_x, height), (255, 255, 0), 1)
        
        # Draw path contour
        if path_contour is not None:
            cv2.drawContours(debug_frame, [path_contour], 0, (0, 255, 0), 2)
        
        # Draw bot contour and position
        if bot_contour is not None:
            cv2.drawContours(debug_frame, [bot_contour], 0, (255, 0, 0), 2)
        
        if bot_position is not None:
            cv2.circle(debug_frame, bot_position, 5, (255, 0, 255), -1)
            
        if bot_box is not None:
            cv2.drawContours(debug_frame, [bot_box], 0, (0, 0, 255), 2)
        
        # Draw junctions
        for junction in junctions:
            # Draw a larger circle for junctions
            cv2.circle(debug_frame, junction, 8, (255, 0, 0), -1)
            
            # Check if this junction has been visited
            for visited in self.visited_junctions:
                if math.dist(junction, visited) < 20:
                    # Draw an X for visited junctions
                    cv2.drawMarker(debug_frame, junction, (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
        
        # Draw target point
        if target_x is not None and target_y is not None:
            cv2.circle(debug_frame, (int(target_x), int(target_y)), 5, (0, 0, 255), -1)
            
        # Add status text
        status_text = f"Path Detected: {self.path_detected}"
        cv2.putText(debug_frame, status_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if self.at_junction:
            junction_text = f"At Junction! Waiting: {time.time() - self.junction_start_time:.1f}s"
            cv2.putText(debug_frame, junction_text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        if self.direction_at_junction:
            direction_text = f"Direction: {self.direction_at_junction}"
            cv2.putText(debug_frame, direction_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Show the processed frames
        cv2.imshow("Processed Frame", debug_frame)
        cv2.imshow("Binary View", binary)
        cv2.waitKey(1)
    
    def destroy_node(self):
        # Clean up OpenCV windows
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node_obj = MazeSolver()
    try:
        rclpy.spin(node_obj)
    except KeyboardInterrupt:
        pass
    finally:
        node_obj.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()