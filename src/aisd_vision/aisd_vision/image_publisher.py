import rclpy
from rclpy.node import Node

# Imports for the Image message type
from sensor_msgs.msg import Image

# Imports for converting between ROS and OpenCV images
from cv_bridge import CvBridge

# Imports for OpenCV functions
import cv2


class ImagePublisher(Node):
    """
    A ROS2 node that captures video from the camera (index 0)
    and publishes it as Image messages.
    """
    def __init__(self):
        # Call the parent Node constructor with the node name 'image_publisher'
        super().__init__('image_publisher')

        # Create a publisher for the Image message type on the 'video_frames' topic
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)

        # Define the period for the timer callback (e.g., 0.1 seconds)
        timer_period = 0.1

        # Create a timer that calls the timer_callback function every 'timer_period' seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize video capture object for camera index 0
        self.cap = cv2.VideoCapture(0)

        # Initialize the CvBridge object for conversion
        self.br = CvBridge()

    def timer_callback(self):
        # Read a frame from the video capture device
        ret, frame = self.cap.read()

        # Check if the frame was read successfully
        if ret:
            # Convert the OpenCV image (frame) to a ROS Image message and publish it
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

            # Log the message for debugging (optional, but good practice)
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().warning('Failed to read frame from camera')


def main(args=None):
    # Initialize the ROS 2 communication layer
    rclpy.init(args=args)

    # Create an instance of the ImagePublisher node
    image_publisher = ImagePublisher()

    # Keep the node running until it is manually shut down (e.g., with Ctrl+C)
    rclpy.spin(image_publisher)

    # Clean up and shut down the node
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
