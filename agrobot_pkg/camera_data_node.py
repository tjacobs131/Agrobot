import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraDataNode(Node):

    def __init__(self):
        super().__init__('camera_data_node')

        self.publisher = self.create_publisher(Image, 'raw_images', 10)

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self): # Runs periodically
        # Get image data
        image_data = '...'  # Replace this with code to get image data

        msg = Image()
        # msg.data = image_data

        print('Publishing data')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    camera_node = CameraDataNode()

    rclpy.spin(camera_node)

    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()