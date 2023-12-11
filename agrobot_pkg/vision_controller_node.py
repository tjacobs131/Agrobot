import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from agrobot_msgs.msg import VisionPublishClosestCrop

class VisionControllerNode(Node):

    def __init__(self):
        super().__init__('vision_controller_node')

        self.publisher = self.create_publisher(VisionPublishClosestCrop, 'detected_crop', 1)

        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.logger = self.get_logger() # Set up logger
        self.logger.info("Start vision controller")

    def timer_callback(self): # Runs periodically
        # Get image data
        
        msg = VisionPublishClosestCrop()
        msg.crop_type = ""
        msg.crop_x = 0.0
        msg.crop_y = 0.0

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    vision_node = VisionControllerNode()

    rclpy.spin(vision_node)

    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()