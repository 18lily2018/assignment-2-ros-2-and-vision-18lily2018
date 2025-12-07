#!/usr/bin/env python3

import os
import wave
import time
import pyaudio

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RecordingPublisher(Node):
    """
    This node records audio from the microphone for a short duration
    and publishes the filename of the saved audio as a String message.
    """

    def __init__(self):
        super().__init__('recording_publisher')

        # Publisher sends file paths to words_publisher
        self.publisher_ = self.create_publisher(String, 'recording', 10)

        # Timer: record once every 8 seconds
        self.timer = self.create_timer(8.0, self.timer_callback)

        # Folder to save recordings
        self.recordings_dir = os.path.join(
            os.path.expanduser('~'),
            'ros2_ws_clean',
            'recordings'
        )
        os.makedirs(self.recordings_dir, exist_ok=True)

        self.get_logger().info(
            f"RecordingPublisher started. Saving WAV files to: {self.recordings_dir}"
        )

    def timer_callback(self):
        """
        Called every 8 seconds:
        - Records audio
        - Saves WAV file
        - Publishes filepath
        """
        filename = self.record_audio()
        msg = String()
        msg.data = filename

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published audio file: {filename}")

    def record_audio(self, duration=3, rate=16000, chunk=1024):
        """
        Records a few seconds of microphone audio and saves to a WAV file.
        """
        audio = pyaudio.PyAudio()

        stream = audio.open(format=pyaudio.paInt16,
                            channels=1,
                            rate=rate,
                            input=True,
                            frames_per_buffer=chunk)

        self.get_logger().info("Recording...")

        frames = []
        for _ in range(0, int(rate / chunk * duration)):
            data = stream.read(chunk, exception_on_overflow=False)
            frames.append(data)

        self.get_logger().info("Recording finished.")

        stream.stop_stream()
        stream.close()
        audio.terminate()

        # Save WAV
        timestamp = int(time.time())
        wav_path = os.path.join(
            self.recordings_dir,
            f"recording_{timestamp}.wav"
        )

        wf = wave.open(wav_path, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(audio.get_sample_size(pyaudio.paInt16))
        wf.setframerate(rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return wav_path


def main(args=None):
    rclpy.init(args=args)
    node = RecordingPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
