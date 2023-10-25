import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraProcessingNode(Node):

    def __init__(self):
        super().__init__('camera_processing_node')

        # Subscribe to camera data
        self.subscription = self.create_subscription(
            Image, 'raw_images', self.listener_callback, 10)

    def listener_callback(self, msg):
        # Process image data here
        pass

def main(args=None):
    rclpy.init(args=args)
    CameraProcessingNode = CameraProcessingNode()

    rclpy.spin(CameraProcessingNode)

    CameraProcessingNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()