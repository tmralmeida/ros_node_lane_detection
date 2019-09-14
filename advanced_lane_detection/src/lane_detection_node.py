#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('advanced_lane_detectison')
import sys
import rospy
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class image_converter:
    def __init__(self):
        self.image_pub=rospy.Publisher("image_published",Image,queue_size=1)
        self.CvBridge=CvBridge()
        self.image_sub=rospy.Subscriber("/top_right_camera/image_rect_color",Image,self.callback,queue_size=1)
    
    def callback(self,data):
        try:
            cv_image=self.CvBridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels)=cv_image.shape
        if cols>60 and rows>60:
            cv2.circle(cv_image,(50,50),10,255)
        cv2.imshow("Image window",cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.CvBridge.cv2_to_img_msg(cv_image,"bgr8"))
        except CvBridgeError as e:
            print(e)

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