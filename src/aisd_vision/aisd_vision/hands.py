import rclpy
from rclpy.node import Node

# Imports for Image messages (subscriber)
from sensor_msgs.msg import Image

# Imports for converting between ROS and OpenCV images
from cv_bridge import CvBridge

# Imports for OpenCV functions
import cv2

# Imports for the MediaPipe library
import mediapipe as mp

# Initialize the MediaPipe hands solution
mp_hands = mp.solutions.hands

# Imports for the custom Hand message type (publisher)
from aisd_msgs.msg import Hand


class Hands(Node):
    """
    A ROS2 node that processes Image messages to detect hands
    and publishes Hand messages with finger coordinates.
    """
    def __init__(self):
        # Call the parent Node constructor with the node name 'hands'
        super().__init__('hands')

        # Create a subscriber for Image messages on the 'video_frames' topic
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10)

        # Initialize the CvBridge object for conversion
        self.br = CvBridge()

        # Create a publisher for Hand messages on the 'cmd_hand' topic
        self.hand_publisher = self.create_publisher(Hand, 'cmd_hand', 10)

    def listener_callback(self, msg: Image):
        # Convert the ROS Image message to an OpenCV image
        image = self.br.imgmsg_to_cv2(msg)

        # Landmark index for the Pinky Finger Tip
        PINKY_FINGER_TIP = 20
        # Landmark index for the Index Finger Tip
        INDEX_FINGER_TIP = 8

        # Analyse the image for hands using MediaPipe
        with mp_hands.Hands(
            # Set model complexity to 0 (fastest)
            model_complexity=0,
            # Minimum confidence value for detection to be successful
            min_detection_confidence=0.5,
            # Minimum confidence value for tracking to be successful
            min_tracking_confidence=0.5) as myhands:

            # To improve performance, optionally mark the image as not writeable
            # so it can be passed by reference.
            image.flags.writeable = False
            # Convert the image from BGR (OpenCV default) to RGB (MediaPipe requirement)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # Process the image to find hand landmarks
            results = myhands.process(image)

            # Check if any hand landmarks were detected
            if results.multi_hand_landmarks:
                # Initialize a new Hand message
                hand_msg = Hand()
                # Get the x-coordinate of the pinky finger tip from the first detected hand
                hand_msg.xpinky = results.multi_hand_landmarks[0].landmark[PINKY_FINGER_TIP].x
                # Get the x-coordinate of the index finger tip from the first detected hand
                hand_msg.xindex = results.multi_hand_landmarks[0].landmark[INDEX_FINGER_TIP].x

                # Check if there are any subscribers to the hand publisher
                if self.hand_publisher.get_subscription_count() > 0:
                    # Publish the Hand message
                    self.hand_publisher.publish(hand_msg)
                else:
                    # Log a message if no one is listening
                    self.get_logger().info('waiting for subscriber')


def main(args=None):
    # Initialize the ROS 2 communication layer
    rclpy.init(args=args)

    # Create an instance of the Hands node
    hands_node = Hands()

    # Keep the node running
    rclpy.spin(hands_node)

    # Clean up and shut down
    hands_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
