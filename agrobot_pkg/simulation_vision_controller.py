import string
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from timeit import default_timer
from std_msgs.msg import String
from agrobot_msgs.msg import VisionPublishClosestCrop
from agrobot_msgs.srv import SimCropLocation
from rclpy.callback_groups import ReentrantCallbackGroup


class SimulationVisionController(Node):
    def __init__(self):
        self.publish_crop_data = self.create_publisher(VisionPublishClosestCrop, 'detected_crop', 1)

        self.objects_subscription = self.create_subscription(SimCropLocation, '')

def main(args=None):
    rclpy.init(args=args)

    visionController = SimulationVisionController()

    rclpy.spin(visionController)

    visionController.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()