#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from aisd_msgs.srv import Speak


class SpeakClient(Node):
    """
    Subscribes to recognized words from words_publisher,
    then sends them to the speak service (gTTS → audio output).
    """

    def __init__(self):
        super().__init__('speak_client')

        # Subscribe to transcribed text
        self.subscription = self.create_subscription(
            String,
            'words',
            self.words_callback,
            10
        )

        # Create a client for the speak service
        self.client = self.create_client(Speak, 'speak')

        self.get_logger().info("SpeakClient ready and waiting for words...")

    def words_callback(self, msg):
        """
        Called whenever new recognized text arrives.
        Sends a service request to speak the words.
        """
        text_to_speak = msg.data.strip()

        if not text_to_speak:
            self.get_logger().warning("Empty text received, skipping.")
            return

        self.get_logger().info(f"Sending text to speak service: {text_to_speak}")

        # Wait for service to appear
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Speak service not available.")
            return

        # Create the request
        req = Speak.Request()
        req.words = text_to_speak

        # Call the service asynchronously
        future = self.client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """
        Called when the speak service responds.
        """
        try:
            response = future.result()
            self.get_logger().info(f"Speak service response: {response.response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SpeakClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
