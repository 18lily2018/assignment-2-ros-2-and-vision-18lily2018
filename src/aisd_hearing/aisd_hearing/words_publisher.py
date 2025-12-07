import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import os
import whisper


class WordsPublisher(Node):
    """
    Subscribes to filenames published by RecordingPublisher,
    runs Whisper speech-to-text on the audio file,
    and publishes the recognized text on the 'heard_words' topic.
    """

    def __init__(self):
        # Initialize base Node with name 'words_publisher'
        super().__init__('words_publisher')

        # Subscribe to the WAV filename topic from recording_publisher
        self.subscription = self.create_subscription(
            String,
            'recording_filename',   # topic name
            self.listener_callback, # callback method
            10                      # queue size
        )

        # Publisher that sends out the recognized words as text
        self.publisher_ = self.create_publisher(
            String,
            'heard_words',          # output topic name
            10
        )

        # Load Whisper model once (small model is a good compromise)
        self.get_logger().info('Loading Whisper model (small)...')
        self.model = whisper.load_model("small")
        self.get_logger().info('WordsPublisher node started.')

    def listener_callback(self, msg: String):
        """
        Called whenever a new filename is published on 'recording_filename'.
        Uses Whisper to transcribe the WAV file and publishes the text.
        """
        wav_file = msg.data
        self.get_logger().info(f"Received WAV file: {wav_file}")

        # Check the file exists
        if not os.path.exists(wav_file):
            self.get_logger().error("File does not exist!")
            return

        # Transcribe using Whisper
        self.get_logger().info("Transcribing with Whisper...")
        result = self.model.transcribe(wav_file)
        text = result.get('text', '').strip()

        self.get_logger().info(f"Heard: {text}")

        # Publish the recognized text
        out_msg = String()
        out_msg.data = text
        self.publisher_.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WordsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
