from ultralytics import YOLO
import cv2
import math
import rclpy
from std_msgs.msg import String, Int64MultiArray
from time import sleep


def detect_object():
    # start webcam
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)

    # model
    model = YOLO("/home/duarte/ros2_ws/src/Agrobot/agrobot_pkg/weights/best.pt")

    # object classes
    classNames = ["carrot", "beetroot", "lettuce", "radish"]

    # last detected object details
    last_object_details = None

    def get_last_object_details():
        return last_object_details

    while True:
        success, img = cap.read()
        results = model(img, stream=True)

        # coordinates
        for r in results:
            boxes = r.boxes

            for box in boxes:
                if math.ceil((box.conf[0] * 100)) / 100 <= 0.85:
                    print("Confidence --->", math.ceil((box.conf[0] * 100)) / 100)
                    continue

                # bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # convert to int values
                #print("Bounding Box --->", (x1, y1, x2, y2))

                # update last detected object details
                last_object_details = {
                    "bounding_box": (x1, y1, x2, y2),
                    "confidence": math.ceil((box.conf[0] * 100)) / 100,
                    "class_name": classNames[int(box.cls[0])]
                }

                # put box in cam
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

                # confidence
                confidence = math.ceil((box.conf[0] * 100)) / 100
                publish_crop_info(crop_type=int(confidence), crop_x=x1, crop_y=y1)

                # class name
                cls = int(box.cls[0])
                #print("Class name -->", classNames[cls])

                # object details
                org = [x1, y1]
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (255, 0, 0)
                thickness = 2

                cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)

        cv2.imshow('Webcam', img)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def publish_crop_info(crop_type, crop_x, crop_y):
    rclpy.init()

    node = rclpy.create_node('detected_objects')

    # Create publisher with the specified message type
    publisher = node.create_publisher(Int64MultiArray, 'detected_objects', 10)

    # Create message
    msg = Int64MultiArray()
    msg.data = [crop_type, crop_x, crop_y]

    try:
        # Publish the message
        node.get_logger().info('Publishing: crop_type={}, crop_x={}, crop_y={}'.format(
            msg.data[0], msg.data[1], msg.data[2]))
        publisher.publish(msg)

        # Sleep to allow time for the message to be published
        #sleep(1)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    detect_object()
