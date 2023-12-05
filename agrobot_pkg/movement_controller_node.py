import string
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from timeit import default_timer
from std_msgs.msg import String
from agrobot_msgs.srv import SimCropLocation
from rclpy.callback_groups import ReentrantCallbackGroup


class MovementControllerNode(Node):

    plantsCollected = 0
    detected_objects = {}

    def __init__(self):
        # Set up node
        super().__init__('movement_controller_node')

        # Set up publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        # Set up subscribers
        self.create_subscription(String, 'detected_objects', self.__update_objects, 10)
        
        self.logger = self.get_logger() # Set up logger
        self.logger.info("Start movement controller")

        # Set up service clients with a new callback group
        service_callback_group = ReentrantCallbackGroup()
        self.get_object_position_service = self.create_client(SimCropLocation, 'get_object_position', callback_group=service_callback_group)
        self.get_object_position_request = SimCropLocation.Request()

        self.start() # Start pathfinding script

    def __update_objects(self, objects):
        detected_objects_str = {}
        detected_objects_str = objects.data.split(';')
        for object in detected_objects_str:
            if(object != ""):
                obj_id, obj_x, obj_y = object.split(",")
                if obj_id and obj_id != "0":
                    self.detected_objects[obj_id] = [float(obj_x), float(obj_y)]

    def start(self):

        # Robot starts in front of planter box'

        # Wait for simulation to finish setting up
        start_time = default_timer()
        while(True):
            if(default_timer() - start_time > 8):
                break
            
        self.move_cmd = Twist()
        self.move_cmd.linear.x = -80.0
        self.cmd_vel_pub.publish(self.move_cmd)

        self.logger.info("Start moving")
        
        self.harvestRowOfPlanters()

        self.deliverCrops()

        # Position self to drive over row two

        self.harvestRowOfPlanters()   

        # Drive backwards until planter boxes cleared

        # Position self to deliver plants

        self.deliverCrops()

        # Drive forward until boxes cleared


    def harvestRowOfPlanters(self):
        self.harvested_object_ids = [] # Needed for simulation
        self.harvest_timer = self.create_timer(0.1, self.__loopRowOfPlanters) # Harvesting loop

    def __loopRowOfPlanters(self):
        # Move until plant is detected

        detected_set = set(self.detected_objects.keys())
        harvested_set = set(self.harvested_object_ids)

        # Find elements in detected_set not in harvested_set
        new_elements = detected_set.difference(harvested_set)

        for item in new_elements:
            self.logger.info("New object: " + str(item))
            

        # If new_elements is not empty, append its elements to harvested_object_ids
        if new_elements:
            self.harvested_object_ids.extend(list(new_elements))

            self.logger.info("Detected crop, stopping")
            self.harvest_timer.cancel()

            self.move_cmd = Twist()
            self.cmd_vel_pub.publish(self.move_cmd)

            # Find nearest object
            target_x = 7.8
            closest_obj_id = None
            closest_x_diff = None

            for object_id in detected_set:
                position = self.detected_objects[object_id]
                x_diff = target_x - position[0]  # Calculate the difference between target_x and the object's x position

                # If this object is closer to target_x than the current closest object, update closest_obj_id and closest_x_diff
                if closest_obj_id is None or x_diff < closest_x_diff:
                    closest_obj_id = object_id
                    closest_x_diff = x_diff
            
            self.logger.info("Closest object: " + str(closest_obj_id))
            self.logger.info("Closest x diff: " + str(closest_x_diff))

            # Wait for robot to stop moving
            start_time = default_timer()
            while True:
                if default_timer() - start_time > 8:
                    break
            
            # Adjust position
            self.adjust_position(closest_obj_id=closest_obj_id)

            # Start gripper service

            self.harvest_timer.reset()

    def adjust_position(self, closest_obj_id):
            # Get object position
            crop_location = self.get_crop_position(int(closest_obj_id))

            self.logger.info("Adjusting position")

            # Calculate time to move
            target_time = abs(crop_location.position[1]) * 60

            if(crop_location.position[1] < 0):
                # Move forward
                self.move_cmd = Twist()
                self.move_cmd.linear.x = 20.0
                self.cmd_vel_pub.publish(self.move_cmd)

                # Calculate time to move forward
                start_time = default_timer()
                self.logger.info("Target time: " + str(target_time))
                while True:
                    if default_timer() - start_time > target_time:
                        break
            else: 
                # Move backwards
                self.move_cmd = Twist()
                self.move_cmd.linear.x = -20.0
                self.cmd_vel_pub.publish(self.move_cmd)

                # Calculate time to move backwards
                start_time = default_timer()
                self.logger.info("Target time: " + str(target_time))
                while True:
                    if default_timer() - start_time > target_time:
                        break
            
            # Stop
            self.move_cmd = Twist()
            self.cmd_vel_pub.publish(self.move_cmd)

            # Wait for robot to stop moving
            start_time = default_timer()
            while True:
                if default_timer() - start_time > 2:
                    break



    def get_crop_position(self, object_id):
        self.get_object_position_request.object_id = object_id
        self.future = self.get_object_position_service.call_async(self.get_object_position_request)
        # Spin until future complete with a new executor
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=5)

        return self.future.result()
    
    def deliverCrops(self):
        # while (not detected unseen bin marker):

            # Move backwards
            
        # Open appropriate sorting box

        # Wait

        # Add marker to seen markers

        # if(seen marker count == 4):

            # Drive forward until delivery boxes cleared

            # empty seen marker list

            return


def main(args=None):
    rclpy.init(args=args)

    movementController = MovementControllerNode()

    rclpy.spin(movementController)

    movementController.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()