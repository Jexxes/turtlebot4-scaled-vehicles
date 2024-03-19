import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/oakd/rgb/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Create a publisher for processed images
        self.publisher = self.create_publisher(Image, '/processed_image', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Process the image
        processed_image = self.process_image(cv_image)
        # Convert the processed image back to a ROS message and publish it
        processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
        self.publisher.publish(processed_image_msg)



def process_image(cv_image):

    # Example processing: Convert to grayscale
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # Convert to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # Apply Gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # Edge detection
    edges = cv2.Canny(blur, 50, 150, apertureSize=3)
    # Hough Line Transform
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=10, maxLineGap=250)

    # Visualize the detected lines
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Show the result
    cv2.imshow("Road Line Detection", cv_image)
    cv2.waitKey(1)  # A brief pause, allowing OpenCV to render the image

        # Convert back to BGR for visualization in RViz
    return cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
