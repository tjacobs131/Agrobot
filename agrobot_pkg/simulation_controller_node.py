import string
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class SimulationControllerNode:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        self.__camera = self.__robot.getDevice('detection_camera')
        self.__camera.enable(64)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('simulation_controller_node')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

        self.__node.camera_publisher = self.__node.create_publisher(Image, 'camera_image', 10)

        self.logger = self.__node.get_logger()
        self.logger.info("Start movement driver")


    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

         # Handle camera
        image = self.__camera.getImage()
        # Convert the image to a ROS Image message
        msg = Image()
        # Fill in the details of the Image message
        msg.data = image
        self.__node.camera_publisher.publish(msg)