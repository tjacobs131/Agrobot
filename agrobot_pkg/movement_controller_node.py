import string
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from timeit import default_timer
from std_msgs.msg import String

class MovementControllerNode(Node):

    plantsCollected = 0

    def __init__(self):
        super().__init__('movement_controller_node')

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        
        self.logger = self.get_logger()
        self.logger.info("Start movement controller")

        self.start()

    def start(self):

        # Robot starts in front of planter box'

        start_time = default_timer()
        while(True):
            if(default_timer() - start_time > 10):
                break

        self.logger.info("Done waiting")
        
        self.move_cmd = Twist()
        self.move_cmd.linear.x = -50.0
        self.cmd_vel_pub.publish(self.move_cmd)

        self.logger.info("Start moving")
        
        timer_period = 10
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.harvestRowOfPlanters()

        self.deliverCrops()

        # Position self to drive over row two

        self.harvestRowOfPlanters()   

        # Drive backwards until planter boxes cleared

        # Position self to deliver plants

        self.deliverCrops()

        # Drive forward until boxes cleared


    def harvestRowOfPlanters(self):
        # while( End row marker not detected )):

        # Move until plant is detected

        for i in range(3):
            # Slow down and keep moving until nearest plant is in correct place

            # Stop when nearest plant is in correct place

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
    
    def timer_callback(self):
        self.move_cmd = Twist()
        self.cmd_vel_pub.publish(self.move_cmd)


def main(args=None):
    rclpy.init(args=args)

    movementController = MovementControllerNode()

    rclpy.spin(movementController)

    movementController.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()