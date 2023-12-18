import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray

def detect_crop():
    cap = cv2.VideoCapture(0) # Open the camera
    ret, frame = cap.read()  # Read a frame from the camera
    segmented_frame, crop_x, crop_y = apply_segmentation(frame) # Apply segmentation to the frame
    
    # Display the segmented frame
    cv2.imshow('Color Segmentation', segmented_frame)
    
    # Publish the x, y coordinates of the crop
    publish_crop_info(crop_x, crop_y)
    
    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
    
    
def publish_crop_info(crop_x, crop_y):
    rclpy.init()
    
    node = rclpy.create_node('detected_crop')
    
    # Creating a publisher to pass the x, y coordinates for gripping
    publisher = node.create_publisher(Int64MultiArray, 'detected_crop', 10)
    
    # Creating a message
    msg = Int64MultiArray()
    msg.data = [crop_x, crop_y]
    
    try:
        # Publish the message
        node.get_logger().info('Publishing: crop_x={}, crop_y={}'.format(
            msg.data[0], msg.data[1]))
        publisher.publish(msg)
        
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


def apply_segmentation(frame):
    img = cv2.imread(frame) # Read the frame
    
    # function for image processing and converting to grayscale
    canned, img = cannyImg(img)
    
    # dilate the image to fill in holes 
    contours, kernel, mask = findBiggestContourProcessing(canned)
    
    # function to find and leave only the biggest contour on the img
    biggest_cntr = findBiggestContour(contours)
    
    # function to smoothen out the contour to remove jaggies
    # and make it easier to draw a bounding box around the crop
    crop_mask = smoothenOutContours(mask, kernel, biggest_cntr)
    
    # function that returns the cropped image 
    # and the final mask for the bounding box to be drawn
    crop, mask = remove_blurred_background(img, crop_mask)
    
    # find contours on the masked image
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # create bounding box
    x, y, w, h = create_bounding_box(img, contours)
    
    point_x, point_y = center_point_in_box(x, y, w, h)
    line_length = 10
    cv2.line(img, (int(point_x) - line_length, int(point_y) - line_length), 
            (int(point_x) + line_length, int(point_y) + line_length), (0, 0, 255), 2)
    cv2.line(img, (int(point_x) + line_length, int(point_y) - line_length), 
            (int(point_x) - line_length, int(point_y) + line_length), (0, 0, 255), 2)
    print(f"Center point coordinates: ({point_x}, {point_y})")
    
    # Returns the final processed img + the center point of the crop (x, y)
    return img, point_x, point_y


def cannyImg(img):
    # Convert the original image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define range of green color in HSV
    lower_green = np.array([30, 50, 50])
    upper_green = np.array([90, 255, 255])

    # Threshold the HSV image to get only green colors
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # Bitwise-AND mask and original image
    img = cv2.bitwise_and(img, img, mask=mask_green)

    # canny
    canned = cv2.Canny(img, 0, 100)
    return canned, img

def findBiggestContourProcessing(canned):
    # dilate to close holes in lines
    kernel = np.ones((3,3),np.uint8)
    mask = cv2.dilate(canned, kernel, iterations = 1)
    
    # find all contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE);
    return contours, kernel, mask

def findBiggestContour(contours):
    biggest_cntr = None;
    biggest_area = 0;
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > biggest_area:
            biggest_area = area
            biggest_cntr = contour
            
    return biggest_cntr

def smoothenOutContours(mask, kernel, biggest_cntr):
    # Draw contours on the image
    crop_mask = np.zeros_like(mask)
    cv2.drawContours(crop_mask, [biggest_cntr], -1, (255), -1)

    # opening + median blur to smooth jaggies
    crop_mask = cv2.erode(crop_mask, kernel, iterations = 5)
    crop_mask = cv2.dilate(crop_mask, kernel, iterations = 5)
    crop_mask = cv2.medianBlur(crop_mask, 21)
    
    return crop_mask

def remove_blurred_background(image_path, crop_mask):
    crop = np.zeros_like(image_path)
    crop[crop_mask == 255] = image_path[crop_mask == 255]
    crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    
    # Apply thresholding to create a binary image
    _, binary_image = cv2.threshold(crop, 127, 255, cv2.THRESH_BINARY)

    # Connected component analysis to label each connected region
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary_image, connectivity=4)

    # Find the largest object (excluding the background)
    largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])

    # Create a mask to remove other objects except the largest one
    mask = np.where(labels == largest_label, 255, 0).astype(np.uint8)

    # Apply the mask to the crop image
    crop = cv2.bitwise_and(crop, crop, mask=mask)
    
    return crop, mask

# Returns only the largest bounding box found in the image
def create_bounding_box(image, contours):
    max_area = 0
    max_contour = None

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = w * h  # Calculate area using width and height

        # Check if the current contour has a larger area than the max area found so far
        if area > max_area:
            max_area = area
            max_contour = contour

    if max_contour is not None:
        x, y, w, h = cv2.boundingRect(max_contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)
    
    return x, y, w, h

def center_point_in_box(box_x, box_y, box_width, box_height):
    point_x = box_x + box_width / 2  # Center x-coordinate of the box
    point_y = box_y + 0.9 * box_height  # 90% down from the top of the box

    return point_x, point_y