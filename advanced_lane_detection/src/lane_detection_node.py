#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('advanced_lane_detectison')
import sys
import rospy
import cv2
import numpy as np
import glob
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt





class image_converter:
    def __init__(self):
        self.image_pub=rospy.Publisher("/image_result",Image,queue_size=1)
        self.bridge=CvBridge()
        self.image_sub=rospy.Subscriber("/top_right_camera/image_rect_color",Image,self.callback,queue_size=1)
    
    def callback(self,data):
        try:
            cv_image=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.processFrames(cv_image)
        except CvBridgeError as e:
            print(e)

       

    def processFrames(self,image):
        point_src=np.float32([[[370, 412]], [[535,412]], [[88,700]], [[858,700]]])
        point_dst=np.float32([[[226, 0]], [[737,0]], [[226,724]], [[737,724]]])
        perspective_matrix=cv2.getPerspectiveTransform(point_src,point_dst)
        image_processed=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        image_processed=cv2.Canny(image_processed,100,150,3)
        image_processed=cv2.warpPerspective(image_processed,perspective_matrix,(image.shape[1],image.shape[0]))
        image_processed=cv2.inRange(image_processed,1,255)

        # Split the initial image into different channels
        image_bin=cv2.warpPerspective(image,perspective_matrix,(image.shape[1],image.shape[0]))
        blue,green,red=cv2.split(image_bin)

        # Red channel binarization
        image_bin=cv2.inRange(red,200,255)

        # Merge the binarized red channel with the edge detected image
        image_final = cv2.add(image_processed,image_bin)
        image_finalRGB=cv2.cvtColor(image_final,cv2.COLOR_GRAY2RGB)

        # Calculate the histogram
        self.calc_line_fits(image_final,image)
        (rows,cols,channels)=image.shape
        if cols>60 and rows>60:
            cv2.circle(image,(50,50),10,255)

    def calc_line_fits(self,img,image_inicial):
        nwindows=9
        margin=100
        minpix=50
        ym_per_pix = 3.0*8.0/924 # meters per pixel in y dimension, 8 lines (5 spaces, 3 lines) at 10 ft each = 3m
        xm_per_pix = 3.7/724 # meters per pixel in x dimension, lane width is 12 ft = 3.7 meters
        
        # Histogram of the bottom half of the image
        histogram=np.sum(img[img.shape[0]//2:,:],axis=0) #img.shape[0]//2:=height/2 
        # Find the peak of the left and the right halfves of the histogram
        # These will be the starting point for the left and right lines
        midpoint=np.int(histogram.shape[0]/2)
        # print('Half point: ',midpoint)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        
        # Set height of the windows
        window_height=np.int(img.shape[0]/nwindows)
        nonzero=img.nonzero()
        nonzeroy=np.array(nonzero[0])
        nonzerox=np.array(nonzero[1])
        

        # Current positions to be updatd for each window
        leftx_current=leftx_base
        rightx_current=rightx_base

        # Create empty lists to receive lefta nd right lane pixel indices
        left_lane_inds=[]
        right_lane_inds=[]

        for window in range (nwindows):

            # Identify window boundaries in x and y (and right and left)
            win_y_low=img.shape[0]-(window+1)*window_height
            win_y_high=img.shape[0]-window*window_height
            win_xleft_low=leftx_current-margin
            win_xleft_high=leftx_current+margin
            win_xright_low=rightx_current-margin
            win_xright_high=rightx_current+margin

            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        # Fit a second order polynomial to each
        left_fit_m = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
        right_fit_m = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
               
        self.create_final_image(image_inicial,img,left_fit,right_fit)
        return left_fit, right_fit, left_fit_m, right_fit_m

    def create_final_image(self,img, binary_warped, leftLine, rightLine):
        left_fit = leftLine
        right_fit = rightLine
        # Create an image to draw the lines on
        warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
        cv2.polylines(color_warp, np.int32([pts_left]), isClosed=False, color=(255,0,255), thickness=20)
        cv2.polylines(color_warp, np.int32([pts_right]), isClosed=False, color=(0,255,255), thickness=20)

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        point_src=np.float32([[[370, 412]], [[535,412]], [[88,700]], [[858,700]]])
        point_dst=np.float32([[[226, 0]], [[737,0]], [[226,724]], [[737,724]]])
        perspective_matrix=cv2.getPerspectiveTransform(point_dst,point_src)
        newwarp = cv2.warpPerspective(color_warp,perspective_matrix,(color_warp.shape[1],color_warp.shape[0]))

        # Combine the result with the original image
        result = cv2.addWeighted(img, 1, newwarp, 0.5, 0)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(result,"bgr8"))
        except CvBridgeError as e:
            print(e)
        

        return result


    





         

def main(args):
    ic=image_converter()
    rospy.init_node('lane_detection_node',anonymous=False)
    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)