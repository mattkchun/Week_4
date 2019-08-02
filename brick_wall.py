#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan, Joy, Image
from ackermann_msgs.msg import AckermannDriveStamped
from color_segmentation import cd_color_segmentation, signIdentify
from newZed import Zed_converter
from cv_bridge import CvBridge, CvBridgeError
import cv2
from scipy import stats



class BrickWall:
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
        
        self.box_x = 0
        self.last_x = 0
        
        self.new_angle = 0
    
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
        
        #applies the current filter to the image and stores the image in imagePush
        self.flag_box, self.imagePush = cd_color_segmentation(self.camera_data.cv_image, color="b")
        
        self.size_calc(self.flag_box)
        
        #outputs the image to the IMAGE_TOPIC
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.imagePush, "bgr8"))
        except CvBridgeError as e:
            print("Error bridging Zed image", e)
        
        print(self.flag_box)

        self.cmd.drive.steering_angle = self.new_angle
        self.cmd.drive.speed = 0.7
        self.drive_pub2.publish(self.cmd)
    
    def size_calc(self, box):
        """ calculate the x and y size of the box in pixels"""
        width = box[1][0] - box[0][0]
        height = box[1][1] - box[0][1]
        #self.box_size = width*height
        #rospy.loginfo("box_size: {}".format(self.box_size))
        self.box_x = (box[0][0]+box[1][0])/2
        #rospy.loginfo("x: {} , y: {}".format(self.box_x, box[0][1]))
        if self.box_x == 0:
            self.box_x = self.last_x
        else:
            self.last_x = self.box_x
        self.box_left = box[0][0]
        
        offset = 0.05 + (width/2000) #adjust for left camera offset, adjust more when closer to wall
        
        self.new_angle = offset -(self.box_x-336)/500.0 #1000.0 336

if __name__ == "__main__":
    rospy.init_node('BrickWall')
    potential_field = BrickWall()
    rospy.spin()

