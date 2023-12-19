import rclpy
from rclpy.node import Node
from techshare_ros_pkg2.srv import TextToSpeech  # Replace with your package name
import pyttsx3

class TextToSpeechNode(Node):
    def __init__(self):
        super().__init__('text_to_speech_node')
        self.service = self.create_service(TextToSpeech, 'speak', self.speak_callback)
        self.engine = pyttsx3.init()
        # Set up voices
        self.japanese_voice_id = "HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Speech\Voices\Tokens\TTS_MS_JA-JP_HARUKA_11.0"  # Replace with the actual ID
        self.english_voice_id = "HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Speech\Voices\Tokens\TTS_MS_EN-US_ZIRA_11.0"  # Replace with the actual ID

    def speak_callback(self, request, response):
        self.get_logger().info('Received request: "%s"' % request.text)
        
        if request.is_japanese:
            self.engine.setProperty('voice', self.japanese_voice_id)
        else:
            self.engine.setProperty('voice', self.english_voice_id)

        try:
            self.engine.say(request.text)
            self.engine.runAndWait()
            response.success = True
        except Exception as e:
            self.get_logger().error('Error in speaking: %s' % str(e))
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
