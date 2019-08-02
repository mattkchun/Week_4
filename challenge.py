#!/usr/bin/env python

import cv2
import time
import random  
import numpy as np
#from sensor_msgs.msg import Image 

class Data():
    def __init__(self):
        self.threshold_keys = ['low_red', 'high_red', 'low_green', 'high_green',
                               'low_blue', 'high_blue', 'low_hue', 'high_hue',
                               'low_sat', 'high_sat', 'low_val', 'high_val'] 

        self.thresholds = {}
        self.createThresholds()
        
        self.key_dictionary =  {}
        """{ord('u'): ,
                               ord('i'): D.green,
                               ord('o'): D.blue,
                               ord('j'): D.red_threshed,
                               ord('k'): D.green_threshed,
                               ord('l'): D.blue_threshed,
                               ord('a'): D.hue,
                               ord('s'): D.sat,
                               ord('d'): D.val,
                               ord('z'): D.hue_threshed,
                               ord('x'): D.sat_threshed,
                               ord('c'): D.val_threshed,
                              }
         """
    def createThresholds(self):
        start_values = [0, 255]*6
        i = 0
        for key in self.threshold_keys:
            self.thresholds[key] = start_values[i]
            i += 1

class Sliders:
    """ Sliders to apply color segmentation to the code """
    def __init__(self):
        self.name = "sliders"
        self.answer = "None"	

        self.img = cv2.imread("resources/rainbow.jpg")
        self.data = Data()
        self.show_answer = False

    def update_single_band(self):
        """ color segments the single band image """        
        self.image_copy_ran = cv2.resize(self.img, (0,0), fx=0.5, fy=0.5)
        
        d = {'g': 'green', 'b':'blue', 'r': 'red', 'h': 'hue', 's': 'saturation', 'v': 'value'}
        cspace_keys = {'r': 2, 'g': 1, 'b': 0, 'h': 0, 's': 1, 'v': 2}
        if self.random_cspace:
            if self.random_cspace in 'hsv':
                self.image_copy_ran = cv2.cvtColor(self.image_copy_ran, cv2.COLOR_BGR2HSV)
            self.single_band_ran = cv2.split(self.image_copy_ran)[cspace_keys[self.random_cspace]]
        if self.show_answer:
            cv2.putText(self.single_band_ran, d[self.random_cspace], (100,100), cv2.FONT_HERSHEY_SIMPLEX,4, (0, 255, 0))
            self.answer = d[self.random_cspace]
        cv2.imshow("Random: Single band", self.single_band_ran)


    def print_(self):
        """ prints necessary information """
        d = {'g': 'green', 'b':'blue', 'r': 'red', 'h': 'hue', 's': 'saturation', 'v': 'value'}
        print("------------------------------------------------")
        if self.random_cspace:
            print("Random: Single band window displaying", d[self.random_cspace])
 
    def show(self, verbose=True):
        """ shows the images and waits for key presses """
        self.verbose = verbose
        cv2.namedWindow("Random: Single band")
        print('Press [q] or [esc] to close the window')
        if verbose:
            print('------------------------------------------------------------')
            print(" q    : quit")
            print(" ESC  : quit")
		
            print(" z    : show random single band image")

        # display the image and wait for a keypress or trackbar update
        self.single_band_random = None
        
        choices = ['r', 'g', 'b', 'h', 's', 'v']

        self.random_cspace = random.choice(choices)
        self.update_single_band()
        while(True):
            k = cv2.waitKey(9) & 0xFF
            if k == ord('q') or k == 27:  # 27 is [esc]
                self.print_()
                cv2.destroyAllWindows()
                exit()
                break
            elif k == ord('z'):
                print("Pressed z")
                self.random_cspace = random.choice(choices)
                self.update_single_band()

def main():
    sliding = Sliders()
    sliding.show()

    cv2.destroyAllWindows()

if __name__ == "__main__" :
    main()
