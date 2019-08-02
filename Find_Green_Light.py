#/usr/bin/env python

#import libraries and color segmentation code
import rospy
import cv2
import time
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image

#initalize global variables
DRIVE_TOPIC = "/drive"
IMAGE_TOPIC = "/zed/zed_node/color_seg_output"

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
        #self.camera_data = Zed_converter(False, save_image = False)
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
            return False

        #        self.size_calc(self.flag_boxB)
        #        self.size_calc(self.flag_boxY)
        
        #outputs the image to the IMAGE_TOPIC
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.imagePush, "bgr8"))
        except CvBridgeError as e:
            print("Error bridging Zed image", e)





