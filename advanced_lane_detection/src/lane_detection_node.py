#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('advanced_lane_detectison')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class image_converter:
    def __init__(self):
        self.image_pub=rospy.Publisher("image_published",Image,queue_size=1)
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
        self.calHist(image_final)


        # Detect the lane boundary



        # Lane curve fitting

        





        (rows,cols,channels)=image.shape
        if cols>60 and rows>60:
            cv2.circle(image,(50,50),10,255)
        cv2.imshow("Initial Image",image)
        cv2.imshow("Edge Image Warpped",image_processed)
        cv2.imshow("Initial Warpped Image Binarized",image_bin)
        cv2.imshow("Final Image",image_finalRGB)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_processed,"mono8"))
        except CvBridgeError as e:
            print(e)
        
    def calHist(self,im):
        histogram=[]
        mid_height=im.shape[0]*0.55
        mid_height=int(mid_height)
        for i in range(im.shape[1]): #Width
            ROI = im[i:(im.shape[0]-mid_height - 1),1:mid_height]
            dst=cv2.divide(255,ROI)
            # print(np.sum(dst[0]))

            # histogram.append(np.sum(dst[0]))





         

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