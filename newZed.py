#!/usr/bin/env python

# Basically, pass in any function and this will call it on the image
# Pushes image post-filter into topic "/zed/image_topic" so that rviz can listen 
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image  

from color_segmentation import cd_color_segmentation
ZED_TOPIC = "/zed/zed_node/rgb/image_rect_color"

class Zed_converter:
    """ inspired by https://is.gd/AsVXLH 
        Zed_converter publishes a Zed topic and uses OpenCV to stream it
    """
    def __init__(self, show_image=False, save_image=False): 
        self.show_image = show_image
        #self.output_path = output_path
        self.save_image = save_image

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(ZED_TOPIC, Image, self.callback)
        self.cv_image = None

        self.out =  None
        self.arr = []
        self.counter = 0

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print("Error bridging Zed image, ", e)


        if self.show_image: 
            cv2.imshow("Image window", self.cv_image)
            cv2.waitKey(3)

        if self.save_image:
            cv2.imwrite("./images/img"+str(self.counter).zfill(6)+".jpg", self.cv_image)
            self.counter += 1


    def image_save(self):
        if len(self.arr) == 0:
            print "ERROR, NOTHING IN ARRAY"
            exit
        print "Number of frames ", len(self.arr)

        h, w, layers = self.arr[0].shape
        print "The size is h w", h, w
        fourcc = cv2.VideoWriter_fourcc("M","4","S","2")
        video = cv2.VideoWriter(self.output_path, fourcc, 25, (w, h))

        for i in self.arr:
            video.write(i.astype('uint8'))
        video.release()

def main():
    ic = Zed_converter(show_image=False)
    rospy.init_node("image_converter", anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        exit()
    cv2.destroyAllWindows()

if __name__ == "__main__" :
    main()
