import rclpy
from rclpy.node import Node

class MovementControllerNode(Node):

    plantsCollected = 0

    def __init__(self):
        super().__init__('movement_controller_node')

        # Set up listeners

        pass

    def start(self):

        # Robot starts in front of planter box

        self.harvestRowOfPlanters(self)

        self.deliverCrops(self)

        # Position self to drive over row two

        self.harvestRowOfPlanters(self)   

        # Drive backwards until planter boxes cleared

        # Position self to deliver plants

        self.deliverCrops(self)

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


def main(args=None):
    rclpy.init(args=args)

    movementController = MovementControllerNode()

    rclpy.spin(movementController)

    movementController.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()