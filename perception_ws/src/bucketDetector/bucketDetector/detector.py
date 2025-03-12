#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import String

class BucketDetector(Node):
    def __init__(self):
        super().__init__('bucketDetector')
        self.get_logger().info('Bucket Detector Node has started.')
        self.publisher = self.create_publisher(String, '/bucket/movement_instructions', 10)

        # Ensure NumPy arrays are correctly initialized
        self.lower_orange = np.array([5, 100, 100], dtype=np.uint8)
        self.upper_orange = np.array([15, 255, 255], dtype=np.uint8)

        self.cap = cv2.VideoCapture(0)  # Open default webcam

        if not self.cap.isOpened():
            self.get_logger().error("❌ Could not open webcam. Check if it's in use or properly connected.")

    def detect_bucket(self, frame):
        if frame is None:
            self.get_logger().error("❌ Received empty frame. Skipping processing.")
            return None, None, None, None, None

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
            move_x = "MoveLeft"
        elif center_x > frame_center_x + 20:
            move_x = "MoveRight"

        if center_y < frame_center_y - 20:
            move_y = "MoveUp"
        elif center_y > frame_center_y + 20:
            move_y = "MoveDown"

        if not move_x and not move_y:
            return "Closer"

        return f"{move_x} {move_y}".strip()

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            
            if not ret or frame is None:
                self.get_logger().error("❌ Failed to capture frame from webcam. Retrying...")
                continue  # Skip this iteration and retry

            frame_height, frame_width, _ = frame.shape
            x, y, w, h, mask = self.detect_bucket(frame)

            if x is not None:
                instruction = self.get_movement_instructions(x, y, w, h, frame_width, frame_height)
            else:
                instruction = "Bucket Not Found"

            self.publisher.publish(String(data=instruction))
            self.get_logger().info(instruction)

            # Display frames
            cv2.imshow("Bucket Detection", frame)
            cv2.imshow("Mask", mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = BucketDetector()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
