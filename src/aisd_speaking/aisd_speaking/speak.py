#!/usr/bin/env python3

import io

from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play

import rclpy
from rclpy.node import Node

from aisd_msgs.srv import Speak


class SpeakService(Node):
    def __init__(self):
        super().__init__('speak_service')

        # Create service named "speak" using Speak.srv
        self.srv = self.create_service(
            Speak,
            'speak',
            self.speak_callback
        )

        self.get_logger().info("Speak service is ready.")

    def speak_callback(self, request, response):
        """
        Convert request.words → speech and play it.
        """

        # In-memory buffer for MP3 data
        with io.BytesIO() as f:

            # Convert text → speech using Google Text-to-Speech
            tts = gTTS(text=request.words, lang='en')
            tts.write_to_fp(f)

            # Reset pointer to start of buffer
            f.seek(0)

            # Load MP3 into AudioSegment
            song = AudioSegment.from_file(f, format="mp3")

            # Play the audio
            play(song)

        # Reply to client that it worked
        response.response = "OK"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SpeakService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
