import string
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image
from controller import Camera, CameraRecognitionObject, Robot

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

        self.__camera = self.__robot.getDevice('detection_camera') # Get camera attached to the robot
        self.__camera.enable(64)
        self.__camera.recognitionEnable(10) # Enable object recognition
        self.__camera.enableRecognitionSegmentation()

        # Node setup
        rclpy.init(args=None)
        self.__node = rclpy.create_node('simulation_controller_node')

        # Set up subscribers
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

        # Set up publishers
        self.__node.camera_publisher = self.__node.create_publisher(Image, 'camera_image', 10)

        self.logger = self.__node.get_logger() # Logger setup
        self.logger.info("Start movement driver")

    # Runs when the topic cmd_vel recieves a message
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        if not rclpy.ok(): # If node is terminated
            return

        rclpy.spin_once(self.__node, timeout_sec=0) # Run node

        # Apply recieved cmd_vel to motors
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
        msg.data = image
        self.__node.camera_publisher.publish(msg)

        # Get and process detected objects
        detected_objects = self.__camera.getRecognitionObjects()
        self.__camera.getRecognitionSegmentationImage()
        self.__camera.saveRecognitionSegmentationImage("img.png", 0)
        for obj in detected_objects:
            self.logger.info(obj)