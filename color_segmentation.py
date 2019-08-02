import cv2
#import imutils
import numpy as np
import pdb
import rospy

def cd_color_segmentation(img, show_image=False, color="r"):
    """
        Implement the cone detection using color segmentation algorithm
        Input:
        img: np.3darray; the input image with a cone to be detected
        Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
        (x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
        """
    # convert from rgb to hsv color space (it might be BGR)
    new_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # define lower and upper bound of image values
    if color=="r":
        low_range  = np.array( [ 0, 100, 150] ) #105, 200, 140
        high_range = np.array( [ 8, 250, 255] ) #130, 255, 255
    elif color=="b":
        low_range  = np.array( [ 255, 93, 55] ) #100, 110, 150
        high_range = np.array( [ 255, 255, 255] ) #115, 255, 255
    elif color=="y":
        low_range  = np.array( [ 20, 150, 150] )
        high_range = np.array( [ 30, 255, 255] )
    elif color == "g":
        low_range  = np.array( [ 50, 100, 0] )# [ 50, 100, 22]
        high_range = np.array( [ 80, 255, 255] ) # [ 70, 255, 155]
    elif color == "bl":
        low_range  = np.array( [ 0, 0, 0] )
        high_range = np.array( [ 200, 200, 30] )
    elif color == "rb":   #red bricks
        low_range  = np.array( [ 95, 49, 180] )
        high_range = np.array( [ 136, 167, 242] )
    elif color == "cw":   #car washer
        low_range  = np.array( [ 0, 42, 90] )
        high_range = np.array( [ 26, 216, 203] )
    else:
        low_range  = np.array( [ 0, 0, 0] )
        high_range = np.array( [ 255, 255, 255] )

    rospy.loginfo("low: {} high: {}".format(low_range, high_range))
    # create mask for image with overlapping values
    mask = cv2.inRange(new_img, low_range, high_range)
    
    # filter the image with bitwise and
    filtered = cv2.bitwise_and(new_img, new_img, mask=mask)
    
    # find the contours in the image
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    x1, y1, x2, y2 = 0, 0, 0, 0
    if len(contours) != 0:
        # find contour with max area, which is most likely the cone
        # Solution note: max uses an anonymous function in this case, we can also use a loop...
        contours_max = max(contours, key = cv2.contourArea)
        
        # Find bounding box coordinates
        x1, y1, x2, y2 = cv2.boundingRect(contours_max)
        
        # Draw the bounding rectangle
        cv2.rectangle(filtered, (x1, y1), (x1 + x2, y1 + y2), (0, 255, 0), 2)

    # Return bounding box
    return ((x1, y1), (x1 + x2, y1 + y2)), filtered #img

def signIdentify(img):
    # Python program to illustrate
    # multiscaling in template matching
    
    # Read the main image
    # [100:200, 200:472]
    im = img[120:180, 100:572]
    box, img_rgb = cd_color_segmentation(im, color="bl")
    
    sign = img_rgb[box[0][1]:box[1][1], box[0][0]:box[1][0]]
#    cv2.rectangle(img_rgb, box[0], box[1], (0, 255, 0))
    cv2.normalize(sign, sign, 0, 255, cv2.NORM_MINMAX)
    
    h, w, d= sign.shape
    
    sign_left = sign[:, 0:w/2]
    
    sign_right = sign[:, w/2:w]
    print(np.count_nonzero(sign_left), np.count_nonzero(sign_right))
    if np.count_nonzero(sign_left) > np.count_nonzero(sign_right):
        return 'left', img#, im, sign
    else:
        return 'right', img#, im, sign
