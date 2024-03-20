import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv_bridge import CvBridgeError

class LineDetectorNode(Node):
    def __init__(self):
        super().__init__('line_detector_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/oakd/rgb/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
        self.get_logger().info("Line detector node has been started.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))

        # Process the image and detect lines
        processed_image = self.detect_lines(cv_image)

        # Convert the processed image back to a ROS2 message and publish it
        try:
            processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            self.publisher.publish(processed_image_msg)
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))

    def detect_lines(self, cv_image):
        frame = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the HSV range for the color you want to detect
        lower_yellow = np.array([18, 94, 140])
        upper_yellow = np.array([18, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        edges = cv2.Canny(mask, 75, 150)

        # HoughLinesP parameters might need tuning
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap=50)

        # Draw lines on the original image
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
        
        return cv_image

def main(args=None):
    rclpy.init(args=args)
    line_detector_node = LineDetectorNode()
    rclpy.spin(line_detector_node)
    line_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
