#/usr/bin/env python
import numpy as np
import cv2
import argparse
import rospy

# python turnRectangle.py -i ./images/oneway.jpg -l "One-Way-Sign" -s 0

# Construct the argument parser and parse the arguments
#ap = argparse.ArgumentParser()
# Image size is proportional to algorithm frame processing speed - bigger pictures cause more frame lag
#ap.add_argument("-i",
#                "--image",
#                required=True,
#                help="path to the static image that will be processed for keypoints")
#ap.add_argument("-l",
#                "--label",
#                help="string to label the object found in the camera feed" )
#ap.add_argument("-s",
#                "--source",
#                type=int,
#                help="optional argument for choosing a specific camera source (-1,0,1,2,3)")
#args = vars(ap.parse_args())

def orb_det(source):
    MIN_MATCH_COUNT = 1
    cropCoor = ((0,0),(0,0))
    
    # Reads source for cam feed if specific source is passed in
    #    if args["source"] is not None:
    #        cam = cv2.VideoCapture(args["source"])
    #        pass
    #
    #    else:
    #        # Reads in source for cam feed
    #        feed = 0
    #        for src in range(-1, 4):
    #            temp = cv2.VideoCapture(src)
    #            ret_val, testImg = temp.read()
    #            if testImg is None:
    #                pass
    #            else:
    #                feed = src
    #                temp.release()
    #                break
    #
    #        # if VideoCapture(feed) doesn't work, manually try -1, 0, 1, 2, 3 (if none of those work,
    #        # the webcam's not supported!)
    #        cam = cv2.VideoCapture(feed)
    
    # Reads in the image
    img1 = cv2.imread('./images/oneway.jpg', 0)
    
    orb_box = None
    
    #    # Labels the image as the name passed in
    #    if args["label"] is not None:
    #        label = args["label"]
    #    else:
    #        # Takes the name of the image as the name
    #        if image[:2] == "./":
    #            label = label = (image.split("/"))[2]
    #        else:
    #            label = image[2:-4]
    label = "Oneway"
    
    try:
        #TODO Initiate each detector
        orb = cv2.ORB_create()
    except AttributeError:
        print("Install 'opencv-contrib-python' for access to the xfeatures2d module")

    #TODO Compute keypoints
    kp, des = orb.detectAndCompute(img1, None)

    #TODO Brute Force matcher creates BFMatcher object for use below
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = False)
    
    frame = source
    
    if frame is None:             # Did we get an image at all?
        pass


    ################################################### Feature Detection

    kp_s, des_s = orb.detectAndCompute(frame, None)
    max_val = 0
    min_val = 0
    max_loc = 0
    min_loc = 0
    r = 0
    
    if(len(kp) >= 2 and len(kp_s) >= 2):
        # Uses the FLANN algorithm to search for nearest neighbors between elements of two images
        # faster than the BFMatcher for larger datasets
        #TODO
        matches = bf.knnMatch(des, des_s, k = 2)
    
    if des_s is None and len(matches) == 0:
        pass
    
    # Store all the good matches (based off Lowe's ratio test)
    good = []
    for k, pair in enumerate(matches):
        try:
            (m, n) = pair
            if m.distance < 0.75 * n.distance:
                good.append(m)
        except ValueError:
            pass

    # When there are enough matches, we convert the keypoints to floats in order to draw them later
    if len(good) >= MIN_MATCH_COUNT:
        try:
            dst_pts = np.float32([ kp_s[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            src_pts = np.float32([ kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        except IndexError:
            pass
    
        # Homography adds a degree of rotation/translation invariance by mapping the transformation
        # of points between two images
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()
        
        h, w = img1.shape
        # pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        pts = np.float32([ [0,0],[0,h-2],[w-2,h-2],[w-2,0] ]).reshape(-1,1,2)
        
        if M is not None:
            dst = cv2.perspectiveTransform(pts, M)
            intDst = np.int32(dst)
            
            # Polylines is an alternative bounding shape around the largest area of recognition
            #cv2.polylines(frame,[np.int32(dst)],True,(255, 0, 0), 2, cv2.LINE_AA)
            
            # Draws a bounding box around the area of most matched points
            cv2.rectangle(frame, (intDst[0][0][0], intDst[0][0][1]), (intDst[2][0][0], intDst[2][0][1]), (0, 0, 255), 4, cv2.LINE_AA, 0)
            orb_box = frame[intDst[0][0][1]:intDst[2][0][1], intDst[0][0][0]:intDst[2][0][0]]
            #            print(orb_box)
            cv2.putText(frame, label, (dst[0][0][0], dst[0][0][1]) , cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 0, 255), lineType = cv2.LINE_AA )
        
        else:
            matchesMask = None

    else:
        #print ("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
        matchesMask = None
    
    # # Stops crashing but slows processing
    # if (len(kp) <= 0 or len(kp_s) <= 0 or len(matches) == 0 or len(good) == 0):
    #     continue
    
    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                       singlePointColor = None,
                       matchesMask = matchesMask, # draw only inliers
                       flags = 2)
    
    try:
       # Option of slicing the 'good' list to display a certain number of matches (ex. good[:6])
       # Take out draw_params if we do not want to draw matches
        src = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.drawMatches(img1, kp, frame, kp_s, good, None, **draw_params)
    except cv2.error:
        pass
#    return orb_box,frame

########################################## Template Matching ############

#########################################################################
#    if orb_box is not None:
#        h,w,d = orb_box.shape
#        sign = orb_box
#        sign_gray = cv2.cvtColor(sign, cv2.COLOR_BGR2GRAY)
##        print(sign_gray.shape)
##        sign_bw = cv2.threshold(sign_gray, 127, 255, cv2.THRESH_BINARY)[1]
#        ret,sign_bw = cv2.threshold(sign,127,255,cv2.THRESH_BINARY)
#        print(sign_bw.shape)
#        sign_left = sign[:, 0:w/2]
#
#        sign_right = sign[:, w/2:w]
#        rospy.loginfo("left: {} right: {}".format(np.count_nonzero(sign_left), np.count_nonzero(sign_right)))
#        if np.count_nonzero(sign_left) < np.count_nonzero(sign_right):
#            return 'left', frame
#        elif np.count_nonzero(sign_left) > np.count_nonzero(sign_right):
#            return 'right', frame
#        else:
#            return 'none', frame
#    else:
#        return 'None', frame


    if orb_box is not None:
        threshold = 0.6
        img_bgr = orb_box
        #        print(img_bgr.shape)
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        #        bw = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY)[1]
#        print(img_bgr.shape)
        template = cv2.imread("./images/LEFT.png",0)
        lress = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)
        template = cv2.imread("./images/RIGHT.png",0)
        rress = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)
        output=0
#        if np.max(lress)>= threshold:
#            output-=1
#        if np.max(rress)>= threshold:
#            output+=1
        if np.max(lress)>= np.max(rress):
            output = -1
        else:
            output = 1
        print(np.max(lress),np.max(rress))
        rospy.loginfo("output: {}".format(output))
        return output, frame
    else:
        return 'None', frame


