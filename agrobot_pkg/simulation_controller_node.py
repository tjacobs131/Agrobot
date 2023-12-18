import math
import string
import rclpy
import ctypes
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from agrobot_msgs.srv import SimCropLocation
from agrobot_msgs.msg import VisionPublishClosestCrop
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image
from controller import Camera, CameraRecognitionObject, Robot, Supervisor

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class SimulationControllerNode:
    def init(self, webots_node, properties):        
        self.detected_objects = {}

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
        self.__node.closest_crop_publisher = self.__node.create_publisher(VisionPublishClosestCrop, 'detected_crop', 1)

        # Set up subscribers
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(String, 'detected_objects', self.__update_objects, 10)

        # Set up services
        self.__node.create_service(SimCropLocation, 'get_object_position', self.__get_object_position_callback)

        self.logger = self.__node.get_logger() # Logger setup
        self.logger.info("Start movement driver")

    # Runs when the topic cmd_vel recieves a message
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __update_objects(self, objects):
        detected_objects = {}
        detected_objects = objects.data.split(';')
        for object in detected_objects:
            if(object != ""):
                obj_id, obj_x, obj_y = object.split(",")
                if obj_id and obj_id != "0":
                    self.detected_objects[obj_id] = [float(obj_x), float(obj_y)]

    def __get_object_position_callback(self, request, response):
        object_id = request.object_id
        self.logger.info("Get object position: " + str(object_id))
        if str(object_id) in self.detected_objects.keys():
            response.position = self.detected_objects[str(object_id)]
            self.logger.info("Object found: " + str(response.position))
        else:
            raise Exception('Object not found')
        
        self.logger.info("Returning")
        return response

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

        if len(detected_objects) > 0:
            # Find closest object
            target_x = 1.15
            closest_obj = None
            for obj in detected_objects:
                current_x = obj.getPosition()[2]
                if closest_obj == None:
                    closest_obj = obj
                    continue;
                elif abs(math.dist(closest_obj.getPosition()[2], target_x)) > abs(math.dist(obj.getPosition()[2], target_x)):
                    # Current object is closer to target than closest
                    closest_obj = obj
            
            if closest_obj != None:
                msg = VisionPublishClosestCrop()
                msg.crop_type = "lettuce"
                msg.crop_x = closest_obj.getPosition()[2]
                msg.crop_y = closest_obj.getPosition()[1]

                self.__node.closest_crop_publisher.publish(msg)

            msg = String()
            for obj in detected_objects:
                msg.data += str(obj.getId()) + ","
                position_list = [obj.getPosition()[i] for i in range(3)]
                msg.data += str(position_list[0]) + ","
                msg.data += str(position_list[2]) + ";"
                
            self.__node.detected_objects_publisher.publish(msg)