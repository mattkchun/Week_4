import cv2
import imutils
import numpy as np
import pdb
import rospy

def cd_color_segmentation(img, low_range, high_range, show_image=False):
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
        contours_max = max(contours, key=cv2.contourArea)
        
        # Find bounding box coordinates
        x1, y1, x2, y2 = cv2.boundingRect(contours_max)
        
        # Draw the bounding rectangle
        cv2.rectangle(img, (x1, y1), (x1 + x2, y1 + y2), (0, 255, 0), 2)
    
    if show_image:
        cv2.imshow("Color segmentation", img)
        key = cv2.waitKey()
        if key == 'q':
            cv2.destroyAllWindows()

    # Return bounding box
    return ((x1, y1), (x1 + x2, y1 + y2)), filtered


def signIdentify(img):
    # Python program to illustrate
    # multiscaling in template matching
    
    # Read the main image
    # [100:200, 200:472]
    im = img[120:180, 100:572]
    box, img_hsv = cd_color_segmentation(im, np.array([0, 0, 0]), np.array([200, 200, 30]))
#    print(box)
#    bgrImage = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)
#    grayImage = grayImage = cv2.cvtColor(bgrImage, cv2.COLOR_BGR2GRAY)
#    (thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)
#    print(img_hsv)
#    bgrImage = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)
#    img_gray = cv2.cvtColor(bgrImage, cv2.COLOR_BGR2GRAY)
#    bw = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY)[1]
#    img_hsv = cv2.normalize(img_hsv, None, 0, 255, cv2.NORM_MINMAX)

    sign = img_hsv[box[0][1]:box[1][1], box[0][0]:box[1][0]]
    cv2.rectangle(img_hsv, box[0], box[1], (0, 255, 0))
#    sign = sign[:][:][2]
#    print(bw)
#    print(sign)

    h, w, d = sign.shape
    
    sign_left = sign[:, 0:w/2]
    
    sign_right = sign[:, w/2:w]
    rospy.loginfo("left: {} right: {}".format(np.count_nonzero(sign_left), np.count_nonzero(sign_right)))
    if np.count_nonzero(sign_left) > np.count_nonzero(sign_right):
        return 'left'#, im, sign
    else:
        return 'right'#, im, sign
