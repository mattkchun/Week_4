#/usr/bin/env python

import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String



DRIVE_TOPIC = "/drive"
SCAN_TOPIC = "/scan"
AR_TOPIC = "/ar_pose_marker"

class ARDrive(object):
    def __init__(self):
        rospy.init_node("ar")
        #initialize publishers and subscribers
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
        self.scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.driveCallback)
        self.ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, self.arCallback)
        self.sound_pub = rospy.Publisher("state", String, queue_size=1)

        #initialize cmd object
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0
        self.cmd.drive.steering_angle = 0

        self.speed_state = "Stop"
        self.turn_state = "Straight"

        self.states = {"Forward": 1,
        "Backward": -1,
        "Stop": 0,
        "Right": -1,
        "Left": 1,
        "Straight": 0}

    def driveCallback(self, data):
        '''LIDAR callback, sets drive commands'''
        self.cmd.drive.speed = self.states[self.speed_state]
        self.cmd.drive.steering_angle = self.states[self.turn_state]

    def arCallback(self, tags):
        '''Callback when an AR tag is detected'''
        changeSpeed = False
        changeTurn = False
        for tag in tags.markers:
            if tag.id == 0:
                if not self.speed_state == "Forward":
                    self.speed_state = "Forward"
                    changeSpeed = True
            if tag.id == 1:
                if not self.speed_state == "Backward":
                    self.speed_state = "Backward"
                    changeSpeed = True
            if tag.id == 2:
                if not self.speed_state == "Stop":
                    self.speed_state = "Stop"
                    changeSpeed = True
            if tag.id == 3:
                if not self.speed_state == "Right":
                    self.turn_state = "Right"
                    changeTurn = True
            if tag.id == 4:
                if not self.speed_state == "Left":
                    self.turn_state = "Left"
                    changeTurn = True
            if tag.id == 5:
                if not self.speed_state == "Straight":
                    self.turn_state = "Straight"
                    changeTurn = True
        if changeSpeed and self.turn_state == "Straight" and not self.speed_state == "Stop":
            self.sound_pub.publish(self.speed_state)
        elif changeTurn and not self.speed_state == "Stop":
            self.sound_pub.publish(self.turn_state)



def main():
    try:
        ic = ARDrive()
        rospy.Rate(100)
        while not rospy.is_shutdown():
            ic.drive_pub.publish(ic.cmd)         
    except rospy.ROSInterruptException:
        exit()

if __name__ == "__main__":
    main()



    

