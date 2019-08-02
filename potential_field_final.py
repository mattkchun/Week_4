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



class PotentialField:
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
    
        self.state = "wall_follower"
        self.states = ["waiting", "potential_field", "found_sign", "wall_follower",]
    
        self.box_x = 0
        self.last_x = 0

        self.turnDirection = 0
        self.timer = 0
        self.certainty = 0
    
    def scan_callback(self, data):
        if self.timeSinceLastAction != 0:
            self.timeSinceLastAction -= 1
            return
        '''Checks LIDAR data'''
        self.data = data.ranges
#        dir = self.signdir(self.camera_data.cv_image)
#        max = len(self.data)
#        front = self.data[(9*max)/20:(11*max)/20]
#        rospy.loginfo("min: {}".format(min(front)))
#        if dir != 0 and dir == self.last_dir:
#            self.certainty += 1
#            #self.state = "found_sign"
#            self.timer = 30
#        else:
#            self.certainty = 0
#            self.last_dir = dir
##            if min(front)<1.9:
##                self.cmd.drive.speed = 0
##                self.cmd.drive.steering_angle = 0
#        if self.certainty >= 2:
#            self.turnDirection = dir
#            self.state = "found_sign"

        self.drive_callback()
    
    def drive_callback(self):
        '''Publishes drive commands'''
        #outputs the image to the IMAGE_TOPIC
        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")
        
        self.convertPoints(self.data)
        
        print(self.state)
        
        if self.state == "waiting":
            self.cmd.drive.speed = 0
            self.cmd.drive.steering_angle = 0
            self.flag_box, self.imagePush = cd_color_segmentation(self.camera_data.cv_image, color="g")
            #print(self.flag_box)
            self.size_calc()
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.imagePush, "bgr8"))
            except CvBridgeError as e:
                print("Error bridging Zed image", e)
            
            if self.count >= 3:
                self.state = "potential_field"
                #self.state = "waiting"
        elif self.state == "potential_field":
            self.convertPoints(self.data)
            self.calcFinalVector(self.cartPoints)

            self.cmd.drive.speed = self.finalVector[0]
            self.cmd.drive.steering_angle = self.finalVector[1]
        elif self.state == "found_sign":
            self.cmd.drive.speed = 1
            self.cmd.drive.steering_angle = -self.turnDirection
            if self.timer <= 0:
                self.state = "potential field"
            else:
                self.timer -= 1
        elif self.state == "wall_follower":
            turn_angle = self.wall_follower(1)
            self.cmd.drive.speed = 1
            self.cmd.drive.steering_angle = turn_angle
        self.drive_pub.publish(self.cmd)
    
    def convertPoints(self, points):
        '''Convert all current LIDAR data to cartesian coordinates'''
        for index in range(len(points)): # + left, - right
            currentAngle = (3*math.pi*index)/(2*len(points))
            adjustedAngle = (3.0*math.pi/4) - currentAngle
            x = math.cos(adjustedAngle)*points[index]
            y = math.sin(adjustedAngle)*points[index]
            self.cartPoints[index] = (x,y)

    def calcFinalVector(self, points):
        '''Calculate the final driving speed and angle'''
        x_vector = 0
        y_vector = 0
        for index in range(len(points)):
#            if abs(points[index][0]) > 0.001:
#            angle_diff = (1 - (abs(index-((index/2)+1))/((index/2)+1))) + 0.25
#            angle_diff = (1 - (abs(index-((index/2)+1))/((len(points)/2)+1))) + 0.25
            angle_diff = 1
#            if index != 0 and index != len(points)-1 and (self.data[index]-self.data[index-1]>0.2 or self.data[index]-self.data[index+1]>0.2):
#                angle_diff = 10
#            angle_diff = 1
            angle_diff = math.pow(angle_diff, 2)
            x_change = 1/(points[index][0])
            x_vector -= x_change * angle_diff
#            x_vector += 1/(points[index][0])
#            if abs(points[index][1]) > 0.001:
            y_change = 1/(points[index][1])
            y_vector += y_change * angle_diff
#            y_vector += 1/(points[index][1])
        #rospy.loginfo("x: {} y: {}".format(x_vector, y_vector))
#        x_vector += 1000
#        y_vector *= 300
        speed = 1
        angle = math.atan2(y_vector,x_vector)
        #print(angle)
#        angle /= 9
        angle = min(0.33, max(-0.33, angle))
#        front = self.data[(len(self.data)*19)/40:(len(self.data)*21)/40] #9,11
#        front_dist = min(front)
#        if front_dist<0.4:
#            self.timeSinceLastAction = 15
#            speed = -1
#            if new_angle<0:
#                new_angle = 0.34
#            else:
#                new_angle = -0.34
        self.finalVector=[speed,angle]
        #print(self.finalVector)
    
    def size_calc(self):
        """ calculate the x and y size of the box in pixels"""
        #rospy.loginfo("box_size: {}".format(self.box_size))
        width = self.flag_box[1][0] - self.flag_box[0][0]
        height = self.flag_box[1][1] - self.flag_box[0][1]
        #        self.box_size = width*height
        #print("AREA", width*height)
        box_area = width*height
        if box_area <= 320 and box_area >= 250:
            self.count += 1
        else:
            self.count == 0
            print("COUNT", self.count)
        self.box_x = (self.flag_box[0][0]+self.flag_box[1][0])/2
        #rospy.loginfo("x: {} , y: {}".format(self.box_x, box[0][1]))
#        if self.box_x == 0:
#            self.box_x = self.last_x
#        else:
#            self.last_x = self.box_x
#        self.box_left = self.flag_box[0][0]

#        offset = 0.05 + (width/2000) #adjust for left camera offset, adjust more when closer to cone

#        self.new_angle = offset -(self.box_x-336)/500.0 #1000.0 336
    def signdir(self,img,threshold=.6):
        img_rgb = img
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
        bw = cv2.threshold(img_gray, 90, 255, cv2.THRESH_BINARY)[1]
#        template = cv2.imread("images/LEFT.png",0)
#        lres = cv2.matchTemplate(bw,template,cv2.TM_CCOEFF_NORMED)
#        template = cv2.imread("images/RIGHT.png",0)
#        rres = cv2.matchTemplate(bw,template,cv2.TM_CCOEFF_NORMED)
        template = cv2.imread("images/LEFTS.png",0)
        lress = cv2.matchTemplate(bw,template,cv2.TM_CCOEFF_NORMED)
        template = cv2.imread("images/RIGHTS.png",0)
        rress = cv2.matchTemplate(bw,template,cv2.TM_CCOEFF_NORMED)
#        template = cv2.imread("images/LEFTxS.png",0)
#        lresxs = cv2.matchTemplate(bw,template,cv2.TM_CCOEFF_NORMED)
#        template = cv2.imread("images/RIGHTxS.png",0)
#        rresxs = cv2.matchTemplate(bw,template,cv2.TM_CCOEFF_NORMED)
        output=0
#            if(np.max(lres) >= threshold) or (np.max(lress)>= threshold) or (np.max(lresxs)>= threshold):
#                output-=1
#            if(np.max(rres) >= threshold) or (np.max(rress)>= threshold) or (np.max(rresxs)>= threshold):
#                output+=1
        if np.max(lress)>= threshold:
            output-=1
        if np.max(rress)>= threshold:
            output+=1
        rospy.loginfo("output: {}".format(output))
        return output

    def wall_follower(self, dir, target_dist = 0.5):
#        x = np.array([i[0] for i in self.cartPoints])
#        x = x[int((0.25+(dir/4))*len(x)):int((0.75+(dir/4))*len(x))]
        for index in range(len(self.data)): # + left, - right
            currentAngle = (3*math.pi*index)/(2*len(self.data))
            adjustedAngle = (3.0*math.pi/2) - currentAngle
            x = math.cos(adjustedAngle)*self.data[index]
            y = math.sin(adjustedAngle)*self.data[index]
            self.cartPoints[index] = (x,y)
        if dir == 1:
            x = np.array([i[0] for i in self.cartPoints])
            cutoff = len(x)/4
            x = x[:cutoff]
            y = np.array([i[1] for i in self.cartPoints])
            y = y[:cutoff]
        else:
            x = np.array([i[0] for i in self.cartPoints])
            cutoff = 3*len(x)/4
            x = x[cutoff:]
            y = np.array([i[1] for i in self.cartPoints])
            y = y[cutoff:]
#        y = np.array([i[1] for i in self.cartPoints])
#        y = y[int((0.25+(dir/4))*len(y)):int((0.75+(dir/4))*len(y))]
#        x = np.array([0, 3, 6, 9])
#        y = np.array([0, 45, 100, 120])
        slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)
        #print(slope, intercept)

#        x_int = -(intercept)/(slope + (1/slope))
#        y_int = slope*x_int + intercept
#        dist = math.hypot(x_int,y_int)

        target_y = 3
        target_x = (target_y-intercept)/slope
        theta = math.atan2(-1,slope)
        if theta < 0:
            theta += math.pi
        target_x += math.cos(theta)*target_dist
        target_y += math.sin(theta)*target_dist
        target_angle = math.atan2(target_y,target_x) - (math.pi/2)
        #print(target_x, target_y, target_angle)
        rospy.loginfo("slope: {} target_angle: {}".format(slope, target_angle))
        return target_angle/9


if __name__ == "__main__":
    rospy.init_node('potential_field')
    potential_field = PotentialField()
    rospy.spin()

