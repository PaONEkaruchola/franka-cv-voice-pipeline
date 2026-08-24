#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        self.publisher_ = self.create_publisher(String, '/voice_commands', 10)
        self.recognizer = sr.Recognizer()
        
        self.get_logger().info("Voice Node Active. Calibrating microphone...")
        
        # Run listening loop in a background thread so it doesn't block ROS 2 spin
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def listen_loop(self):
        with sr.Microphone() as source:
            # Calibrate for ambient noise for 1 second before starting
            self.recognizer.adjust_for_ambient_noise(source)
            self.get_logger().info("Ready! Say a command (e.g., 'pick up the blue block').")
            
            while rclpy.ok():
                try:
                    # Listen for audio (times out after 5s of silence)
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                    self.get_logger().info("Processing speech...")
                    
                    # Send to Google Speech Recognition
                    text = self.recognizer.recognize_google(audio).lower()
                    self.get_logger().info(f"Recognized: '{text}'")
                    
                    # Publish to ROS 2 network
                    msg = String()
                    msg.data = text
                    self.publisher_.publish(msg)
                    
                except sr.WaitTimeoutError:
                    pass # Loop back and keep listening
                except sr.UnknownValueError:
                    self.get_logger().warn("Could not understand the audio.")
                except sr.RequestError as e:
                    self.get_logger().error(f"API request failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()