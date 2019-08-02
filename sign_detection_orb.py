#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan, Joy, Image
from ackermann_msgs.msg import AckermannDriveStamped
from color_segmentation_sign import cd_color_segmentation, signIdentify
from newZed import Zed_converter
from cv_bridge import CvBridge, CvBridgeError
import cv2
from scipy import stats
from turnRectNoFeature import orb_det



class SignDetectionOrb:
    SCAN_TOPIC = '/scan'
    DRIVE_TOPIC = '/drive'
    IMAGE_TOPIC = '/zed/zed_node/color_seg_output'
    DRIVE_OUT_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/default'
    
    def __init__(self):
        self.data = None
        self.cmd = AckermannDriveStamped()
        self.count = 0
        
        #write your publishers and subscribers here; they should be the same as the wall follower's
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.drive_pub2 = rospy.Publisher(self.DRIVE_OUT_TOPIC, AckermannDriveStamped, queue_size=1)
        self.image_pub = rospy.Publisher(self.IMAGE_TOPIC, Image, queue_size = 2)
        
        self.camera_data = Zed_converter(False, save_image = False)
        self.imagePush = None
        self.bridge = CvBridge()
        
        self.mode = 0
        self.signs = []
        self.last_dir = 0
        self.dir = 0
        self.certainty = 0
        self.turn_dir = 0
    
    def scan_callback(self, data):
        '''Checks LIDAR data'''
        self.data = data.ranges

        self.drive_callback()
    
    def drive_callback(self):
        """laser scan callback function"""
        #checks if the image is valid first
        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")
        
        #outputs the image to the IMAGE_TOPIC
#        try:
#            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.imagePush, "bgr8"))
#        except CvBridgeError as e:
#            print("Error bridging Zed image", e)
        speed = 0.5
        angle = 0
        
        if self.turn_dir == 0:
            front = self.data[19*len(self.data)/40:21*len(self.data)/40]
            rospy.loginfo("min: {}".format(min(front)))
            if min(front) > 0.57:
                speed = 0.4
            elif min(front) <0.43:
                speed = -0.4
            else:
                speed = 0
            frame = None
            dir, frame = orb_det(self.camera_data.cv_image)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            rospy.loginfo("dir: {}".format(dir))

            if dir != 'None':
                if dir == self.last_dir:
                    self.certainty += 1
                    if self.certainty > 5: #10
                        self.turn_dir = dir
                else:
                    self.certainty = 0
                    self.last_dir = dir
            else:
                self.last_dir = 0
                self.certainty = 0
            rospy.loginfo("turn_dir: {} last_dir: {}".format(self.turn_dir, self.last_dir))
        if self.turn_dir != 0:
            angle = -self.last_dir
        self.cmd.drive.speed = speed
        self.cmd.drive.steering_angle = angle
        self.drive_pub2.publish(self.cmd)

if __name__ == "__main__":
    rospy.init_node('sign_detection_orb')
    sign_detection_orb = SignDetectionOrb()
    rospy.spin()
