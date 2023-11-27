import string
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image
from controller import Camera, CameraRecognitionObject, Robot, Supervisor

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025



class SimulationControllerNode:
    def init(self, webots_node, properties):        
        self.detected_object_ids = []

        self.supervisor = Supervisor()

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
        self.__camera.recognitionEnable(64) # Enable object recognition

        # Node setup
        rclpy.init(args=None)
        self.__node = rclpy.create_node('simulation_controller_node')

        # Set up publishers
        self.__node.camera_publisher = self.__node.create_publisher(Image, 'camera_image', 10)
        self.__node.detected_objects_publisher = self.__node.create_publisher(String, 'detected_objects', 10)

        # Set up subscribers
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(String, 'detected_objects', self.__update_objects, 10)

        self.logger = self.__node.get_logger() # Logger setup
        self.logger.info("Start movement driver")

    # Runs when the topic cmd_vel recieves a message
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __update_objects(self, objects):
        self.detected_object_ids = []
        detected_objects = objects.data.split(',')
        for object in detected_objects:
            if (object != "" and object != "0"):
                self.detected_object_ids.append(int(object))



    def step(self):
        if not rclpy.ok(): # If node is not running
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

        # Publish detected objects
        detected_objects = self.__camera.getRecognitionObjects()
        msg = String()
        for obj in detected_objects:
            msg.data += str(obj.getId()) + ","
        self.__node.detected_objects_publisher.publish(msg)

        if len(self.detected_object_ids) > 0:
            self.logger.info("Amount of objects: " + str(len(self.detected_object_ids)))