#!/usr/bin/env python2
# coding=utf8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CaptureImages:
    def __init__(self, server_ip, server_port):
        self.bridge = CvBridge()
        self.rgb_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.server_ip = server_ip
        self.server_port = server_port
        self.saved_rgb_image = False
        self.saved_depth_image = False
        self.capture_images = False
        self.cv_image = None
        self.depth_image_color = None

    def rgb_callback(self, data):
        if not self.capture_images:
            return

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.saved_rgb_image = True
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: {}".format(e))

    def depth_callback(self, data):
        if not self.capture_images:
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
            depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            self.depth_image_color = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_normalized), cv2.COLORMAP_JET)
            self.saved_depth_image = True
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: {}".format(e))

    def capture_and_save(self, filename):
        if self.cv_image is not None:
            cv2.imwrite("./" +"rgb"+ filename, self.cv_image)
            print("save_rgb")
        if self.depth_image_color is not None:
            cv2.imwrite("./" +"depth"+filename, self.depth_image_color)
            print("save_depth")
        rospy.loginfo("Saved images as {}".format(filename))

    def reset_image_flags(self):
        self.saved_rgb_image = False
        self.saved_depth_image = False

    def set_capture_images(self, status):
        self.capture_images = status

