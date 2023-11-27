import string
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from timeit import default_timer
from std_msgs.msg import String

class MovementControllerNode(Node):

    plantsCollected = 0
    detected_object_ids = []

    def __init__(self):
        # Set up node
        super().__init__('movement_controller_node')

        # Set up publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        # Set up subscribers
        self.create_subscription(String, 'detected_objects', self.__update_objects, 10)
        
        self.logger = self.get_logger() # Set up logger
        self.logger.info("Start movement controller")

        self.start() # Start pathfinding script

    def __update_objects(self, objects):
        detected_objects = objects.data.split(',')
        for object in detected_objects:
            if (object != "" and object != "0"):
                self.logger.info("Detected object: " + object)
                self.detected_object_ids.append(int(object))

    def start(self):

        # Robot starts in front of planter box'

        # Wait for simulation to finish setting up
        start_time = default_timer()
        while(True):
            if(default_timer() - start_time > 10):
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
        self.last_object_ids = [] # Needed for simulation
        self.harvest_timer = self.create_timer(0.1, self.__loopRowOfPlanters) # Harvesting loop

    def __loopRowOfPlanters(self):

        self.logger.info("Harvest loop")
        self.logger.info("Last object: " + str(self.last_object_ids))

        # Move until plant is detected

        if len(self.detected_object_ids) == 0:
            self.logger.info("No objects")
            return

        if len(self.detected_object_ids) > 0 and not self.detected_object_ids in self.last_object_ids:
            self.last_object_ids += self.detected_object_ids[0]
            self.logger.info("Stopping")

            self.harvest_timer.cancel()

            self.move_cmd = Twist()
            self.cmd_vel_pub.publish(self.move_cmd)

            # Start gripper service
            start_time = default_timer()
            while True:
                if default_timer() - start_time > 10:
                    self.move_cmd = Twist()
                    self.move_cmd.linear.x = -40.0
                    self.cmd_vel_pub.publish(self.move_cmd)
                    break
            
            self.harvest_timer.reset()

        for i in range(3):
            # Correct position based on detected crop

            # Harvest plant

            pass
        
        pass

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