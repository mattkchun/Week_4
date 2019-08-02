#/usr/bin/env python

#import libraries and color segmentation code
import rospy
import cv2
import time
import numpy as np
import sys, math, random, copy
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image

#initalize global variables
DRIVE_TOPIC = "/drive"
IMAGE_TOPIC = "/zed/zed_node/color_seg_output"
CHECK = True

AUTONOMOUS_MODE = True

class driveStop(object):
    """class that will help the robot drive and stop at certain conditions"""
    def __init__(self):
        """initalize the node"""
        rospy.init_node("driveStop")
        self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
        self.image_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size = 2)
        rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)
        
        """initialize the box dimensions"""
        self.empty_box = ((0,0),(0,0))
        self.boxY = self.empty_box
        self.certainty = 0
        
        self.flag_center = (0,0)
        self.flag_size = 0
        
        """driving stuff"""
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0
        self.cmd.drive.steering_angle = 0
        
        """get the camera data from the class Zed_converter in Zed"""
        self.camera_data = Zed_converter(False, save_image = False)
        self.imagePush = None
        self.imagePushB = None
        self.imagePushY = None
        
        self.bridge = CvBridge()
        self.min_value=0
        self.last_x = 0
        self.state = 0
        self.follow_color = "red"
        self.find_color = "yellow"
        self.timer = 0
        self.dir=0
        self.stopped = False
    
    #    def size_calc(self):
    #        """ calculate the x and y size of the box in pixels"""
    #        pix_width = self.flag_box[1][0] - self.flag_box[0][0]
    #        pix_height = self.flag_box[1][1] - self.flag_box[0][1]
    #        #print("point", self.flag_box[0][0], ":", self.flag_box[0][1])
    #        #print("point", self.flag_box[1][0], ":", self.flag_box[1][1])
    #        self.box_size = pix_width*pix_height
    
    def size_calc(self, box):
        """ calculate the x and y size of the box in pixels"""
        #rospy.loginfo("box_size: {}".format(self.box_size))
        width = box[1][0] - box[0][0]
        height = box[1][1] - box[0][1]
        self.boxY = width*height
        print("size of box", self.boxY)

        
        
    
    #print("driving angle: ", self.new_angle)
    #rospy.loginfo("a: {} w: {}".format(self.new_angle, width))
    
    def driveStop_car_callback(self,data):
        """laser scan callback function"""
        #checks if the image is valid first
        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")
        
        #applies the current filter to the image and stores the image in imagePush
        self.boxY, self.imagePushY = cd_color_segmentation(self.camera_data.cv_image, color="y")
        
        #finds the size of the box
        self.size_calc(self.boxY)
        if self.boxY != self.empty_box:
            print("Switching to pot field")
            CHECK = False

        #        self.size_calc(self.flag_boxB)
        #        self.size_calc(self.flag_boxY)
        
        #outputs the image to the IMAGE_TOPIC
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.imagePush, "bgr8"))
        except CvBridgeError as e:
            print("Error bridging Zed image", e)
        

    
  
class PotentialField:
    SCAN_TOPIC = '/scan'
    DRIVE_TOPIC = '/drive'
    
    def __init__(self):
        self.data = None
        self.cmd = AckermannDriveStamped()
        
        #write your publishers and subscribers here; they should be the same as the wall follower's
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        
        #cartesian points -- to be filled (tuples)
        self.cartPoints = [None for x in range(1081)]
        
        #[speed, angle]
        self.finalVector = [1, 0]
        self.timeSinceLastAction = 0
    
    def scan_callback(self, data):
        if self.timeSinceLastAction != 0:
            self.timeSinceLastAction -= 1
            return
        '''Checks LIDAR data'''
        self.data = data.ranges
        self.drive_callback()
    
    def drive_callback(self):
        '''Publishes drive commands'''
        self.convertPoints(self.data)
        self.calcFinalVector(self.cartPoints)
#        if

        self.cmd.drive.speed = self.finalVector[0]
        self.cmd.drive.steering_angle = self.finalVector[1]
        
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
            angle_diff = (1 - (abs(index-((index/2)+1))/((len(points)/2)+1))) + 1
#            angle_diff = 1
            angle_diff = math.pow(angle_diff, 2)
            x_change = 1/(points[index][0])
            x_vector -= x_change * angle_diff
#            x_vector += 1/(points[index][0])
#            if abs(points[index][1]) > 0.001:
            y_change = 1/(points[index][1])
            y_vector += y_change * angle_diff
#            y_vector += 1/(points[index][1])
        rospy.loginfo("x: {} y: {}".format(x_vector, y_vector))
#        x_vector += 1000
#        y_vector *= 300
        speed = 2
        angle = math.atan2(y_vector,x_vector)
        print(angle)
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
        print(self.finalVector)


def main():
    global AUTONOMOUS_MODE
    try: 
        while(CHECK):
            ic = driveStop()
            rate = rospy.Rate(100)
            while not rospy.is_shutdown():
                if AUTONOMOUS_MODE:
                    ic.pub.publish(ic.cmd)
        if CHECK == False:
            print("initalize pot field")
            rospy.init_node('potential_field')
            potential_field = PotentialField()
            rospy.spin()

    except rospy.ROSInterruptException:
        exit()


if __name__ == "__main__":
    main()


