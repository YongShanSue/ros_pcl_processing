#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for detecting objects
# Peter Yu Oct 2016

import rospy
import numpy as np
import cv2  # OpenCV module

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

rospy.init_node('object_detection', anonymous=True)



# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()  
cv_image = 1



def main():

    rospy.Subscriber('/camera/rgb/image_rect_color', Image, rosImageVizCallback)
    rospy.spin()

# Task 1 callback for ROS image
def rosImageVizCallback(msg):
    # 1. convert ROS image to opencv format
    global  cv_image
    cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
 

    # 2. visualize it in a cv window
    cv2.imshow("OpenCV_View", cv_image)

    # set callback func for mouse hover event
    cv2.setMouseCallback("OpenCV_View", cvWindowMouseCallBackFunc)
    cv2.waitKey(3)  # milliseconds

# Task 1 callback for mouse event
def cvWindowMouseCallBackFunc(event, xp, yp, flags, param):
    #print 'In cvWindowMouseCallBackFunc: (xp, yp)=', xp, yp  # xp, yp is the mouse location in the window
    # 1. Set the object to 2 meters away from camera
    global  cv_image
    if(event == cv2.EVENT_LBUTTONDOWN):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        print "h: %d s: %d h: %d" %(hsv_image[int(yp),int(xp),0],hsv_image[int(yp),int(xp),1],hsv_image[int(yp),int(xp),2])#hsv_image[1,1,1] )#+'s: '+hsv_image[int(yp)][int(xp)][2]+'v:'+hsv_image[int(yp)][int(xp)][3])
    # 2. Visualize the pyramid



if __name__=='__main__':
    main()
    
