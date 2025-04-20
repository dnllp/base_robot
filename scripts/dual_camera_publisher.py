#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import numpy as np

class CSICameraPublisher:
    def __init__(self, camera_id, topic_name, capture_width=640, capture_height=480, display_width=640, display_height=480, framerate=30):
        self.camera_id = camera_id
        self.topic_name = topic_name
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.display_width = display_width
        self.display_height = display_height
        self.framerate = framerate
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(self.topic_name, Image, queue_size=10)
        self.capture = self._gstreamer_pipeline(
            capture_width=self.capture_width,
            capture_height=self.capture_height,
            display_width=self.display_width,
            display_height=self.display_height,
            framerate=self.framerate,
            flip_method=0,  # Puedes ajustar esto según la orientación de tu cámara
            camera_id=self.camera_id
        )
        self.thread = threading.Thread(target=self.publish_frame)
        self.running = True
        self.thread.start()

    def _gstreamer_pipeline(self, capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=30, flip_method=0, camera_id=0):
        return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                camera_id,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )

    def publish_frame(self):
        while self.running and self.capture.isOpened():
            ret, frame = self.capture.read()
            if ret:
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.publisher.publish(image_msg)
                except CvBridgeError as e:
                    rospy.logerr("CvBridge error: {}".format(e))
            else:
                rospy.logwarn("Could not read frame from camera {}".format(self.camera_id))
                rospy.sleep(0.1)
        self.capture.release()
        rospy.loginfo("Camera {} capture stopped.".format(self.camera_id))

    def stop(self):
        self.running = False
        self.thread.join()

if __name__ == '__main__':
    rospy.init_node('dual_camera_publisher')

    # Configuración de la cámara frontal (sensor-id=0 por defecto)
    front_camera_topic = rospy.get_param('~front_camera_topic', 'camera/front/image_raw')
    front_camera_width = rospy.get_param('~front_camera_width', 640)
    front_camera_height = rospy.get_param('~front_camera_height', 480)
    front_camera_fps = rospy.get_param('~front_camera_fps', 30)
    front_camera_publisher = CSICameraPublisher(
        camera_id=0,
        topic_name=front_camera_topic,
        capture_width=front_camera_width,
        capture_height=front_camera_height,
        framerate=front_camera_fps
    )
    rospy.loginfo("Publishing front camera on topic: {}".format(front_camera_topic))

    # Configuración de la cámara posterior (sensor-id=1, puede variar)
    rear_camera_topic = rospy.get_param('~rear_camera_topic', 'camera/rear/image_raw')
    rear_camera_width = rospy.get_param('~rear_camera_width', 640)
    rear_camera_height = rospy.get_param('~rear_camera_height', 480)
    rear_camera_fps = rospy.get_param('~rear_camera_fps', 30)
    rear_camera_publisher = CSICameraPublisher(
        camera_id=1,
        topic_name=rear_camera_topic,
        capture_width=rear_camera_width,
        capture_height=rear_camera_height,
        framerate=rear_camera_fps
    )
    rospy.loginfo("Publishing rear camera on topic: {}".format(rear_camera_topic))

    rospy.spin()

    front_camera_publisher.stop()
    rear_camera_publisher.stop()