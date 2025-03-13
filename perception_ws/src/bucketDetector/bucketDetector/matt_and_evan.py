"""
BSD 2-Clause License
Copyright (c) 2023, Allied Vision Technologies GmbH
All rights reserved.
"""

import sys
import cv2
import os
import time
import csv
import rclpy
from rclpy.node import Node
from queue import Queue
from typing import Optional
from vmbpy import *
from px4_msgs.msg import CameraCapture, VehicleLocalPosition, VehicleAttitude
import numpy as np


def get_next_filename(base_name="flight", extension=".avi"):
    index = 1
    while os.path.exists(f"{base_name}{index}{extension}"):
        index += 1
    return f"{base_name}{index}{extension}"

# Pixel format for OpenCV processing
opencv_display_format = PixelFormat.Mono8  
VIDEO_FILENAME = get_next_filename()
CSV_FILENAME = VIDEO_FILENAME.replace(".avi", ".csv")  # Corresponding CSV file


def print_preamble():
    print('///////////////////////////////////////////////////')
    print('/// VmbPy Asynchronous Grab and Record with PX4 Data Logging ///')
    print('///////////////////////////////////////////////////\n')


def abort(reason: str, return_code: int = 1):
    print(reason + '\n')
    sys.exit(return_code)


def get_camera(camera_id: Optional[str]) -> Camera:
    with VmbSystem.get_instance() as vmb:
        if camera_id:
            try:
                return vmb.get_camera_by_id(camera_id)
            except VmbCameraError:
                abort(f"Failed to access Camera '{camera_id}'. Abort.")
        else:
            cams = vmb.get_all_cameras()
            if not cams:
                abort("No Cameras accessible. Abort.")
            return cams[0]

def setup_camera(cam: Camera):
    with cam:
        try:
            cam.ExposureAuto.set('Continuous')
        except (AttributeError, VmbFeatureError):
            pass
        try:
            cam.BalanceWhiteAuto.set('Continuous')
        except (AttributeError, VmbFeatureError):
            pass
        try:
            stream = cam.get_streams()[0]
            stream.GVSPAdjustPacketSize.run()
            while not stream.GVSPAdjustPacketSize.is_done():
                pass
        except (AttributeError, VmbFeatureError):
            pass


def setup_pixel_format(cam: Camera):
    cam_formats = cam.get_pixel_formats()
    if opencv_display_format in cam_formats:
        cam.set_pixel_format(opencv_display_format)
    else:
        abort("Camera does not support an OpenCV compatible format. Abort.")


class DroneDataLogger(Node):
    def __init__(self):
        super().__init__('drone_data_logger')

        # Open CSV file and write the header
        with open(CSV_FILENAME, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Frame Timestamp", "X Position", "Y Position", "Z Position", 
                             "Roll (deg)", "Pitch (deg)", "Yaw (deg)"])

        # ROS2 Subscriptions
        self.create_subscription(CameraCapture, '/fmu/out/camera_capture', self.camera_callback, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, 10)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, 10)

        # Data storage
        self.last_camera_timestamp = None
        self.last_x = None
        self.last_y = None
        self.last_z = None
        self.last_roll = None
        self.last_pitch = None
        self.last_yaw = None

    def camera_callback(self, msg):
        """ Store camera frame timestamp """
        self.last_camera_timestamp = msg.timestamp
        self.save_data()

    def position_callback(self, msg):
        """ Store drone position """
        self.last_x = msg.x
        self.last_y = msg.y
        self.last_z = msg.z

    def attitude_callback(self, msg):
        """ Store drone orientation """
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        roll, pitch, yaw = self.quaternion_to_euler(q)
        self.last_roll = np.degrees(roll)
        self.last_pitch = np.degrees(pitch)
        self.last_yaw = np.degrees(yaw)

    def quaternion_to_euler(self, q):
        """ Convert quaternion to Euler angles """
        w, x, y, z = q
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def save_data(self):
        """ Save synchronized frame metadata to CSV """
        if self.last_camera_timestamp is not None and self.last_x is not None:
            with open(CSV_FILENAME, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([self.last_camera_timestamp, 
                                 self.last_x, self.last_y, self.last_z, 
                                 self.last_roll, self.last_pitch, self.last_yaw])
            self.get_logger().info(f"Logged Frame {self.last_camera_timestamp}")

class VideoHandler:
    def __init__(self, video_filename):
        self.display_queue = Queue(3)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video_writer = cv2.VideoWriter(video_filename, fourcc, 30, (640, 480), isColor=True)

    def __call__(self, cam: Camera, stream: Stream, frame: Frame):
        if frame.get_status() == FrameStatus.Complete:
            opencv_image = frame.as_opencv_image()
            if frame.get_pixel_format() == PixelFormat.Mono8:
                bgr_frame = cv2.cvtColor(opencv_image, cv2.COLOR_GRAY2BGR)
            else:
                bgr_frame = opencv_image

            if not self.display_queue.full():
                self.display_queue.put(bgr_frame, True)

            self.video_writer.write(bgr_frame)
            cam.queue_frame(frame)

    def release(self):
        self.video_writer.release()

#   Main Function (Runs ROS2 & Camera Streaming)
def main():
    rclpy.init()
    logger_node = DroneDataLogger()
    print_preamble()
    
    with VmbSystem.get_instance():
        with get_camera(None) as cam:
            setup_camera(cam)
            setup_pixel_format(cam)
            handler = VideoHandler(VIDEO_FILENAME)

            try:
                cam.start_streaming(handler=handler, buffer_count=50)
                rclpy.spin(logger_node)
            finally:
                cam.stop_streaming()
                handler.release()
                logger_node.destroy_node()
                rclpy.shutdown()
                print(f"Recording saved as {VIDEO_FILENAME} and {CSV_FILENAME}")

if __name__ == '__main__':
    main()
