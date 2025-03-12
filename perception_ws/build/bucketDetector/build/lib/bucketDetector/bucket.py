#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import String

class BucketDetector(Node):
    def __init__(self):
        super().__init__('bucket')
        self.get_logger().info('Bucket Detector Node has started.')        

        self.publisher = self.create_publisher(String, '/bucket/movement_instructions', 10)
        
        # Open webcam
        self.cap = cv2.VideoCapture(0)  
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera.")
            return  # Exit constructor if camera fails to open

        # Define color range for detection
        self.lower_orange = np.array([5, 100, 100], dtype=np.uint8)
        self.upper_orange = np.array([15, 255, 255], dtype=np.uint8)
        
        self.timer = self.create_timer(0.1, self.detect_and_publish)
    
    def detect_bucket(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            return x, y, w, h, mask
        
        return None, None, None, None, mask
    
    def get_movement_instructions(self, x, y, w, h, frame_width, frame_height):
        center_x, center_y = x + w // 2, y + h // 2
        frame_center_x, frame_center_y = frame_width // 2, frame_height // 2
        
        move_x, move_y = "", ""
        
        if center_x < frame_center_x - 20:
            move_x = "Move Left"
        elif center_x > frame_center_x + 20:
            move_x = "Move Right"
        
        if center_y < frame_center_y - 20:
            move_y = "Move Up"
        elif center_y > frame_center_y + 20:
            move_y = "Move Down"
        
        if not move_x and not move_y:
            return "Centered, Now Move Closer"
        
        return f"{move_x} {move_y}".strip()
    
    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().error("Failed to capture frame from camera.")
            return  # Skip processing if no valid frame
        
        frame_height, frame_width, _ = frame.shape
        x, y, w, h, mask = self.detect_bucket(frame)
        
        if x is not None:
            instruction = self.get_movement_instructions(x, y, w, h, frame_width, frame_height)
        else:
            instruction = "Bucket Not Found"
        
        self.publisher.publish(String(data=instruction))
        self.get_logger().info(instruction)
        
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()
    

def main(args=None):
    rclpy.init(args=args)
    node = BucketDetector()

    if node.cap is None or not node.cap.isOpened():
        node.get_logger().error("Exiting due to camera initialization failure.")
        return
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
