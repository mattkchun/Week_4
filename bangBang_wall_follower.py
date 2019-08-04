#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan, Joy, Image
from ackermann_msgs.msg import AckermannDriveStamped
from color_segmentation import cd_color_segmentation
from newZed import Zed_converter
from cv_bridge import CvBridge, CvBridgeError
import cv2
from scipy import stats



class wall_follower:
    SCAN_TOPIC = '/scan'
    DRIVE_TOPIC = '/drive'
    IMAGE_TOPIC = '/zed/zed_node/color_seg_output'

    def __init__(self):
        self.data = None
        self.cmd = AckermannDriveStamped()
        self.count = 0

        #write your publishers and subscribers here; they should be the same as the wall follower's
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.image_pub = rospy.Publisher(self.IMAGE_TOPIC, Image, queue_size = 2)

        self.camera_data = Zed_converter(False, save_image = False)
        self.imagePush = None
        self.bridge = CvBridge()

        #cartesian points -- to be filled (tuples)
        self.cartPoints = [None for x in range(1081)]

        #[speed, angle]
        self.finalVector = [1, 0]
        self.timeSinceLastAction = 0

        self.direction = 1 # "1" is right and "2" is left

        #HIHIHIHI
        self.des_dist_middle = 1
        self.des_dist_front = self.des_dist_middle / math.sin(math.pi/4)
        self.cmd.drive.speed = 0.75
    
    
        self.timer = 0
        self.dir = 0

    def scan_callback(self, data):
        if self.timeSinceLastAction != 0:
            self.timeSinceLastAction -= 1
            return
        '''Checks LIDAR data'''
        self.data = data.ranges
        self.drive_callback()

    def drive_callback(self):
        '''Publishes drive commands'''
        #outputs the image to the IMAGE_TOPIC

        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")
        self.cmd.drive.speed = 2 #speed
        self.middle_right = self.data[160:201]
        self.front_right = self.data[340:380]
        self.top = self.data[520:560]
        self.middle_left = self.data[700:740]
        self.front_left = self.data[680:720]

        self.middle_right_point = np.median(self.middle_right)
        self.front_right_point = np.median(self.front_right)
        self.top_point = np.median(self.top)
        self.middle_left_point = np.median(self.middle_left)
        self.front_left_point = np.median(self.front_left)

        print("Top point: ", self.top_point)
        print("Middle Right Point: ", self.middle_right_point)
        print("Middle Left Point: ", self.middle_left_point)
        print("Front Right Point: ", self.front_right_point)
        print("Front Left Point: ", self.front_left_point)



        self.delta = 0.2

        "Compare the two points are turn to the left or right based on the orientation of the points compared to the wall."
        print("hi3")
        if self.front_right_point < self.des_dist_middle + (self.delta * 2): #was self.des.dis + delta before
            print("hi")
            l = (self.middle_left_point + self.front_left_point)/2
            r = (self.middle_right_point + self.front_right_point)/2
            if l > r:
                self.direction = 2
                self.left_wall_follower()

            else:
                self.direction = 1
                self.right_wall_follower()

        else:
            print("hi2")
            if self.direction == 1:
                self.right_wall_follower()
            else:
                self.left_wall_follower()
                    #self.right_wall_follower()



    def right_wall_follower(self):
        print("right")
        self.des_dist_middle = min((self.middle_right_point + self.middle_left_point)/2, 1)
        print("desired_distance", self.des_dist_middle)
        self.des_dist_front = self.des_dist_middle / math.sin(math.pi/4)

        if self.middle_right_point > self.des_dist_middle + self.delta and self.front_right_point > self.des_dist_front + self.delta:
            self.cmd.drive.steering_angle = -1
            self.cmd.drive.steering_angle = (float((self.middle_right_point/self.des_dist_middle)) - 1)*-1
            print("LOOP 1")
        elif self.middle_right_point < self.des_dist_middle - self.delta and self.front_right_point > self.des_dist_front + self.delta:
            self.cmd.drive.steering_angle = (1 - float((self.middle_right_point/self.des_dist_middle)))*1
            #self.cmd.drive.steering_angle = -1
            print("LOOP 2")
        elif (self.middle_right_point > self.des_dist_middle + self.delta) and (self.front_right_point < self.des_dist_front - self.delta):
            self.cmd.drive.steering_angle =  float((self.middle_right_point/self.des_dist_middle)) - 1
            print("LOOP 3")
            #self.cmd.drive.steering_angle = 1
        elif self.middle_right_point < self.des_dist_middle - self.delta and self.front_right_point < self.des_dist_front - self.delta:
            self.cmd.drive.steering_angle = (1 - float((self.middle_right_point/self.des_dist_middle)))
            print("LOOP 4")
            #self.cmd.drive.steering_angle = 1
        else:
            self.cmd.drive.steering_angle = 0
            print("LOOP 5")
        #self.check_safe()
        self.drive_pub.publish(self.cmd)
        print("angle", self.cmd.drive.steering_angle)

    def left_wall_follower(self):
        print("left")
        self.des_dist_middle = min((self.middle_right_point + self.middle_left_point)/2 + 0.5,1)
        print("desired_distance", self.des_dist_middle)
        self.des_dist_front = self.des_dist_middle / math.sin(math.pi/4)
        if self.middle_left_point > self.des_dist_middle + self.delta and self.front_left_point > self.des_dist_front + self.delta:
            #self.cmd.drive.steering_angle = 1
            self.cmd.drive.steering_angle = min((float((self.middle_left_point/self.des_dist_middle)) - 1)*1, 1)

            print("LOOP 1")
        elif self.middle_left_point < self.des_dist_middle - self.delta and self.front_left_point > self.des_dist_front + self.delta:
            self.cmd.drive.steering_angle = min((1 - float((self.middle_left_point/self.des_dist_middle)))*-1, 1)
            #self.cmd.drive.steering_angle = 1
            print("LOOP 2")
        elif (self.middle_left_point > self.des_dist_middle + self.delta) and (self.front_left_point < self.des_dist_front - self.delta):
            self.cmd.drive.steering_angle =  max((float((self.middle_left_point/self.des_dist_middle)) - 1)*-1,-1)
            print("LOOP 3")
            #self.cmd.drive.steering_angle = -1
        elif self.middle_left_point < self.des_dist_middle - self.delta and self.front_left_point < self.des_dist_front - self.delta:
            self.cmd.drive.steering_angle = max(((1 - float((self.middle_left_point/self.des_dist_middle))))*-1, -1)
            print("LOOP 4")
            #self.cmd.drive.steering_angle = -1
        else:
            self.cmd.drive.steering_angle = 0
            print("LOOP 5")
        #self.check_safe()
        self.drive_pub.publish(self.cmd)
        print("angle", self.cmd.drive.steering_angle)
    '''
    def check_safe(self):
        front = self.data[19*len(self.data)/40:21*len(self.data)/40]
        if self.timer <= 0:
            if min(front) < 0.3+(self.cmd.drive.speed/5.0):
                self.timer = 10
#                index = front.index(min(front))
#                if index > 540:
#                    self.dir = -1
#                else:
#                    self.dir = 1
        else:
            self.timer -= 1
#            self.cmd.drive.steering_angle = self.dir
            self.cmd.drive.speed = -0.5
        print(self.timer,self.dir)
    '''


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    potential_field = wall_follower()
    rospy.spin()
