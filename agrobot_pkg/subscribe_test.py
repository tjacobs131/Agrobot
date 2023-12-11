import rclpy
from std_msgs.msg import Int64MultiArray

def callback(msg):
    crop_type, crop_x, crop_y = msg.data
    print('Received: crop_type={}, crop_x={}, crop_y={}'.format(crop_type, crop_x, crop_y))

def subscribe_to_crop_info():
    rclpy.init()

    node = rclpy.create_node('crop_info_subscriber')

    # Create subscriber with the specified message type and callback function
    subscription = node.create_subscription(Int64MultiArray, 'detected_objects', callback, 10)

    try:
        # Keep the script running
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    subscribe_to_crop_info()
