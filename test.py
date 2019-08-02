#!/usr/bin/env python

import numpy as np
import math
import time
sin = math.sin
cos= math.cos
pi = math.pi

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = '/scan'
    DRIVE_TOPIC = '/drive'
    #SIDE = rospy.get_param("wall_follower/side")
    SIDE = 1 # -1 is RIGHT, 1 is LEFT
    
    #VELOCITY = rospy.get_param("wall_follower/velocity")
    VELOCITY = 0.5
    DESIRED_DISTANCE = 1

    def __init__(self):
        # Initialize your publishers and
        # subscribers
        self.previous = [0,0,0]
        self.previousTime = time.time()
        self.data = None
        self.angle = 0
        self.cmd = AckermannDriveStamped()
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)

    def scan(self, data):
        #stores the lidar data so you can work with it
        self.data = data

        #calls function that controls driving
        self.drive()

    def drive(self):
        """controls driving"""

        #gets the angle required
        self.angle = self.find_wall()

        #sets speed and driving angle
        self.cmd.drive.speed = self.VELOCITY
        self.cmd.drive.steering_angle = self.angle

        #publishes the command
        self.drive_pub.publish(self.cmd)

    def reverseToIndex(self,ranges,target_ang,angMin,angMax,angIncre):
        offset = target_ang - angMin
        if offset < 0 or target_ang > angMax:
            print "Illegal angle", target_ang
        return -1
        castedix = offset / angIncre
           #print "reading from index: ",castedix
        if (offset%angIncre != 0):
            lowerbound = min(int(castedix),len(ranges))
            upperbound = lowerbound + 1
            return (ranges[lowerbound] + ranges[upperbound]) / 2
        else:
            return ranges[int(castedix)]
    def getReading(self,radians,message):
           return self.reverseToIndex(message.ranges,radians,message.angle_min,message.angle_max,message.angle_increment)

    def sampleAroundAngle(self,radians,tolerance,message,measurefunc):
        samples = []
        for a in range(radians-tolerance,radians + tolerance):
            samples.append(measurefunc(a)  * self.getReading(a * (pi/180),message))
        samples.sort()
        return samples[len(samples)/2]

    def find_wall(self):
        # 1 = bang-bang
        # 2 = bang-bang with correct distance
        #

        alg = 2
        tempAngle = 0
        if alg is 1:
        # if lidar data has not been received, do nothing
            if self.data == None:
                return 0

            if self.SIDE is -1:
                if self.data.ranges[16] > self.DESIRED_DISTANCE:
                    # to turn left angle is positive
                    tempAngle = -math.pi/4
                elif self.data.ranges[16] < self.DESIRED_DISTANCE:
                    tempAngle = math.pi/4
                else:
                    tempAngle = 0

                if self.data.ranges[45] < self.DESIRED_DISTANCE + 0.5:
                    tempAngle = math.pi/2
            elif self.SIDE is 1:
                if self.data.ranges[82] > self.DESIRED_DISTANCE:
                    # to turn left angle is positive
                    tempAngle = -math.pi/4
                elif self.data.ranges[82] < self.DESIRED_DISTANCE:
                    tempAngle = math.pi/4
                else:
                    tempAngle = 0

                if self.data.ranges[55] < self.DESIRED_DISTANCE + 0.5:
                    tempAngle = -math.pi/2
            else:
                tempAngle = 0

        elif alg is 2:
        # if lidar data has not been received, do nothing
            if self.data == None:
                return 0
            left = self.sampleAroundAngle(90,20,self.data, lambda x: sin(x * (pi/180)))
            right = self.sampleAroundAngle(-90,20,self.data, lambda x: -sin(x * (pi/180)))
            forward = self.sampleAroundAngle(0,20,self.data, lambda x: abs(cos(x * (pi/180))))
            leftderiv = left - self.previous[0]
            leftderiv /= (time.time()-self.previousTime)
            rightderiv = right - self.previous[2]
            rightderiv /= (time.time()-self.previousTime)

            kD = 0.10
            standardAngle = math.pi/6
            if self.SIDE is -1:
                if right > self.DESIRED_DISTANCE:
                    # to turn left angle is positive
                    tempAngle = -(standardAngle - min(kD * rightderiv,math.pi/6))
                elif right < self.DESIRED_DISTANCE:
                    tempAngle = standardAngle - min(kD * rightderiv,math.pi/6)
                else:
                    tempAngle = 0

                if forward < self.DESIRED_DISTANCE + 1:
                    tempAngle = math.pi/2
            elif self.SIDE is 1:
                if left > self.DESIRED_DISTANCE:
                    # to turn left angle is positive
                    tempAngle = -(standardAngle - min(kD * leftderiv,math.pi/6))
                elif left < self.DESIRED_DISTANCE:
                    tempAngle = standardAngle - min(kD * leftderiv,math.pi/6)
                else:
                    tempAngle = 0

                if forward < self.DESIRED_DISTANCE + 1:
                    tempAngle = -math.pi/2
            else:
                tempAngle = 0
            #print left, right, forward
            print "dampen: ",str(kD*100)+"%%", round(leftderiv,4),round(rightderiv,4)
            self.previous = [left,forward,right]
            self.previousTime = time.time()
        else:
            return 0
        ## TO DO: Find Alg for Wall Following ##

        """Lidar data is now stored in self.data, which can be accessed
        using self.data.ranges (in simulation, returns an array).
        Lidar data at an index is the distance to the nearest detected object
        self.data.ranges[0] gives the leftmost lidar point
        self.data.ranges[100] gives the rightmost lidar point
        self.data.ranges[50] gives the forward lidar point
        """

        #returns the output of your alg, the new angle to drive in

        return tempAngle






if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
