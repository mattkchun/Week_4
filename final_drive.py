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
from ar_track_alvar_msgs.msg import AlvarMarkers
from PickWall_Wall_Follower import wall_follower
#wall_follower(dir (1:left, 0:both, -1: right), velocity, distance, data (lidar))


class PotentialField:
    SCAN_TOPIC = '/scan'
    DRIVE_TOPIC = '/drive'
    IMAGE_TOPIC = '/zed/zed_node/color_seg_output'
    AR_TOPIC = '/ar_pose_marker'
    JOY_TOPIC = '/vesc/joy'
    
    def __init__(self):
        self.data = None
        self.cmd = AckermannDriveStamped()
        self.count = 0
        
        #write your publishers and subscribers here; they should be the same as the wall follower's
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.image_pub = rospy.Publisher(self.IMAGE_TOPIC, Image, queue_size = 2)
        self.ar_sub = rospy.Subscriber(self.AR_TOPIC, AlvarMarkers, self.arCallback)
        self.joy_sub = rospy.Subscriber(self.JOY_TOPIC, Joy, self.joyCallback)
        
        self.camera_data = Zed_converter(False, save_image = False)
        self.imagePush = None
        self.bridge = CvBridge()
        
        #cartesian points -- to be filled (tuples)
        self.cartPoints = [None for x in range(1081)]
    
        self.state = "waiting"
        self.states = ["waiting", "start_area", "highway1", "highway-windmill", "windmill", "windmill-graveyard", "graveyard", "castle", "highway2", "prebridge", "bridge", "roundabout", "commons", "underbridge", "finish", "finish2"]
        self.additional_states = ["straight", "stopped"]
        
        ''' waiting: wait for light, then switch to start area when car sees green
            start area: left wall follower (fast and close)
            highway1: right wall follower (fast and far)
            highway-windmill: corner with two sets of ar tags
            windmill: left wall follower (medium speed and closeish)
            graveyard: red line follower (medium speed)
            castle: left wall follower (fast and close)
            highway2: right wall follower (fast and far)'''
        
        self.timer = 0
        self.certainty = 0
    
        self.last_x = 0
    
    
    def scan_callback(self, data):
        '''Checks LIDAR data'''
        self.data = data.ranges
        self.drive_callback()
    
    def drive_callback(self):
        '''Publishes drive commands'''
        #outputs the image to the IMAGE_TOPIC
        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")
        velocity = 0
        angle = 0
        rospy.loginfo("state: {}".format(self.state))
        
        if self.state == "waiting":
            self.flag_box, self.imagePush = cd_color_segmentation(self.camera_data.cv_image, color="g")
            green_light_size = self.size_calc(self.flag_box)
            if green_light_size <= 250 and green_light_size >= 100:
                self.certainty += 1
            else:
                self.certainty = 0
            if self.certainty >= 7:
                self.state = "start_area"
                self.certainty = 0
        elif self.state == "start_area":
            velocity = 1.5
            angle = wall_follower(1,0.7,self.data)
        elif self.state == "highway1":
            velocity = 2.5 #2
            angle = wall_follower(-1,1.7,self.data)
        elif self.state == "highway-windmill":
            velocity = 1.5
            angle = wall_follower(-1,0.8,self.data)
        elif self.state == "windmill":
            velocity = 1.0
            angle = wall_follower(-1,0.9,self.data)
        elif self.state == "windmill-graveyard":
            velocity = 1.5
            angle = wall_follower(-1,1.0,self.data)
            #cropped_image = self.camera_data.cv_image[0:50,:]
            #flag_boxR, imagePush = cd_color_segmentation(self.camera_data.cv_image, color="r")
            #if self.size_calc(flag_boxR) >= 50:
                #self.certainty += 1
                #if self.certainty >= 5:
                    #self.state = "graveyard"
                    #self.certainty = 0
            #else:
                #self.certainty = 0
        elif self.state == "pregraveyard":
            velocity = 0.7
            angle = wall_follower(-1,0.8,self.data)
            cropped = self.camera_data.cv_image[150:360,:]
            flag_boxR, imagePush = cd_color_segmentation(cropped, color="r")
            if self.size_calc(flag_boxR) >= 150:
                self.state = "graveyard"
        elif self.state == "graveyard":
            velocity = 0.85#0.5
            cropped_image = self.camera_data.cv_image[150:360,:]
#            print(self.camera_data.cv_image.shape)
            flag_boxR, imagePush = cd_color_segmentation(cropped_image, color="r")
            self.follow_line(flag_boxR)
            angle = self.follow_line(flag_boxR)
        elif self.state == "castle":
            velocity = 1.5
            angle = wall_follower(1,1.0,self.data)
        elif self.state == "highway2":
            velocity = 2.5 #2
            angle = wall_follower(-1,1.0,self.data)
        elif self.state == "prebridge":
            velocity = 1
            angle = wall_follower(-1,0.3,self.data)
        elif self.state == "bridge":
            velocity = 0.7
            angle = wall_follower(-1,0.5,self.data)
        elif self.state == "roundabout":
            velocity = 1.0
            angle = wall_follower(-1,0.7,self.data)
        elif self.state == "commons":
            velocity = 1.5
            angle = wall_follower(1,0.7,self.data)
        elif self.state == "underbridge":
            velocity = 1
            angle = wall_follower(1,0.7,self.data)
        elif self.state == "finish":
            velocity = 1.5
            angle = wall_follower(-1,0.8,self.data)
        elif self.state == "finish2":
            velocity = 2.5
            angle = wall_follower(-1,0.8,self.data)
        elif self.state == "straight":
            velocity = 2
            angle = 0
        elif self.state == "stopped":
            velocity = 0
            angle = 0
        rospy.loginfo("velocity: {} angle: {}".format(velocity, angle))
        self.drive(velocity, angle)
    
    def arCallback(self, tags):
        '''Callback when an AR tag is detected'''
        ids = []
        for tag in tags.markers:
            ids.append(tag.id)
        if self.state == "start_area":
            if 1 in ids:
                self.state = "highway1"
        elif self.state == "highway1":
            if 3 in ids:
                self.state = "windmill"
            elif 2 in ids:
                self.state = "highway-windmill"
        elif self.state == "highway-windmill":
            if 3 in ids:
                self.state = "windmill"
        elif self.state == "windmill":
            if 4 in ids:
                self.state = "windmill-graveyard"
        elif self.state == "windmill-graveyard":
            if 5 in ids:
                self.state = "pregraveyard"
        elif self.state == "graveyard":
            if 6 in ids:
                self.state = "castle"
        elif self.state == "castle":
            if 7 in ids:
                self.state = "highway2"
        elif self.state == "highway2":
            if 8 in ids:
                self.state = "prebridge"
        elif self.state == "prebridge":
            if 9 in ids:
                self.state = "bridge"
        elif self.state == "bridge":
            if 10 in ids:
                self.state = "roundabout"
        elif self.state == "roundabout":
            if 11 in ids:
                self.state = "commons"
        elif self.state == "commons":
            if 13 in ids:
                self.state = "underbridge"
        elif self.state == "underbridge":
            if 14 in ids:
                self.state = "finish"
        elif self.state == "finish":
            if 17 in ids:
                self.state = "finish2"

#        for tag in tags.markers:
    
    def size_calc(self, box):
        """ calculate the x and y size of the box in pixels"""
        #rospy.loginfo("box_size: {}".format(self.box_size))
        width = box[1][0] - box[0][0]
        height = box[1][1] - box[0][1]
        box_area = width*height
        rospy.loginfo("box area: {}".format(box_area))
        
        return box_area

    def follow_line(self,box):
        box_x = (box[0][0]+box[1][0])/2
        width = box[1][0] - box[0][0]
        
        if box_x == 0:
            box_x = self.last_x
        else:
            self.last_x = box_x

#        offset = 0.05 + (width/2000) #adjust for left camera offset, adjust more when closer to line
        new_angle = -(box_x-336)/250.0

        return new_angle
    
    def joyCallback(self, command):
        #reset for lights
        if command.buttons[0] == 1:
            if self.last_button == 0:
                self.state = self.states[self.states.index(self.state)-1]
                self.last_button = 1
        else:
            self.last_button = 0

    def drive(self, speed, angle):
        self.cmd.drive.speed = speed
        self.cmd.drive.steering_angle = angle
        self.drive_pub.publish(self.cmd)

if __name__ == "__main__":
    rospy.init_node('potential_field')
    potential_field = PotentialField()
    rospy.spin()

