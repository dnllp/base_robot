#!/usr/bin/env python
# -*- coding: utf-8 -*-

# MIT License
# Copyright (c) 2019-2022 JetsonHacks

# A simple code snippet
# Using two  CSI cameras (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit with two CSI ports (Jetson Nano, Jetson Xavier NX) via OpenCV
# Drivers for the camera and OpenCV are included in the base image in JetPack 4.3+

# This script will open a window and place the camera stream from each camera in a window
# arranged horizontally.
# The camera streams are each read in their own thread, as when done sequentially there
# is a noticeable lag

import cv2
import threading
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CSI_Camera:
    def __init__(self):
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # The thread where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    def open(self, gstreamer_pipeline_string):
        try:
            self.video_capture = cv2.VideoCapture(
                gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )
            # Grab the first frame to start the video capturing
            self.grabbed, self.frame = self.video_capture.read()

        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)

    def start(self):
        if self.running:
            print('Video capturing is already running')
            rospy.loginfo("Captura de video esta ejecutandose")
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.running = True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
        return self

    def stop(self):
        self.running = False
        # Kill the thread
        self.read_thread.join()
        self.read_thread = None

    def updateCamera(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Could not read image from camera")
        # FIX ME - stop and cleanup thread
        # Something bad happened

    def read(self):
        with self.read_lock:
            if self.frame is not None:
                frame = self.frame.copy()
                grabbed = self.grabbed
                return grabbed, frame
            else:
                return False, None

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=320,
    capture_height=240,
    display_width=320,
    display_height=240,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def run_cameras():
    rospy.init_node('Nodo de Camaras')
    bridge = CvBridge()

    front_topic = rospy.get_param('~front_camera_topic', '/camera/front/image_raw')
    rear_topic = rospy.get_param('~rear_camera_topic', '/camera/rear/image_raw')

    pub_front = rospy.Publisher(front_topic, Image, queue_size=10)
    pub_rear = rospy.Publisher(rear_topic, Image, queue_size=10)

    left_camera = CSI_Camera()
    left_camera.open(
        gstreamer_pipeline(
            sensor_id=1,
            capture_width=640, # Adjusted for better resolution
            capture_height=480,
            flip_method=2,
        )
    )
    left_camera.start()

    right_camera = CSI_Camera()
    right_camera.open(
        gstreamer_pipeline(
            sensor_id=0,
            capture_width=640, # Adjusted for better resolution
            capture_height=480,
            flip_method=2,
        )
    )
    right_camera.start()

    rate = rospy.Rate(30) # Publish at 30 Hz

    try:
        while not rospy.is_shutdown():
            _, front_image = left_camera.read()
            _, rear_image = right_camera.read()

            if front_image is not None:
                try:
                    front_msg = bridge.cv2_to_imgmsg(front_image, "bgr8")
                    pub_front.publish(front_msg)
                except Exception as e:
                    rospy.logerr("Error converting front image: %s", str(e))

            if rear_image is not None:
                try:
                    rear_msg = bridge.cv2_to_imgmsg(rear_image, "bgr8")
                    pub_rear.publish(rear_msg)
                except Exception as e:
                    rospy.logerr("Error converting rear image: %s", str(e))

            rate.sleep()

    finally:
        left_camera.stop()
        left_camera.release()
        right_camera.stop()
        right_camera.release()

if __name__ == "__main__":
    run_cameras()
