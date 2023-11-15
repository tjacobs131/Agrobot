import rclpy
from sensor_msgs.msg import Image

class CameraDriverNode:
    def __init__(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__camera = self.__robot.getDevice('detection_camera')
        self.__camera.enable(64)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('camera_driver_node')
        self.__publisher = self.__node.create_publisher(Image, 'camera_image', 10)

        self.logger = self.__node.get_logger()
        self.logger.info("Start camera driver")

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        image = self.__camera.getImage()
        # Convert the image to a ROS Image message
        msg = Image()
        # Fill in the details of the Image message
        # ...

        self.__publisher.publish(msg)