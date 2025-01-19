import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
 
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        # Create a publisher for the 'camera_image' topic
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        # OpenCV bridge to convert between OpenCV images and ROS image messages
        self.bridge = CvBridge()
        # Capture from the default camera (ID 0)
        self.cap = cv2.VideoCapture(0)
        # Timer to publish frames at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS Image message and publish
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(image_message)
            self.get_logger().info("Publishing image frame")
    def destroy_node(self):
        super().destroy_node()
        self.cap.release()
 
def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()