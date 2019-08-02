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



class SignDetection:
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
        self.dir = 0
    
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

        speed = 0.7
        angle = 0
        
        if self.mode == 0:
            d_front = sum(self.data[520:560]) / 40
            print(d_front)
            
            if d_front < 0.7:
                self.mode = 1
        elif self.mode == 1:
            speed = 0.5
            d_front = sum(self.data[520:560]) / 40
            
            dir = self.detect_sign()
            
            print(d_front, dir)
            if dir == 'left':
                self.signs.append(0)
            else:
                self.signs.append(1)
            
            if d_front < 0.5:
                if float(sum(self.signs)) / len(self.signs) < 0.5:
                    self.dir = 1
                else:
                    self.dir = -1
                self.mode = 2
        
        elif self.mode == 2:
            angle = self.dir
        
        
        self.cmd.drive.speed = speed
        self.cmd.drive.steering_angle = angle
        self.drive_pub2.publish(self.cmd)
    
    def detect_sign(self):
        dir = signIdentify(self.camera_data.cv_image)#, './images/onewayarrow.png')
        return dir

if __name__ == "__main__":
    rospy.init_node('sign_detection')
    sign_detection = SignDetection()
    rospy.spin()
