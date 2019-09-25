#!/usr/bin/env python
import roslib
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def chatter_callback(frame):
    cv_frame = bridge.imgmsg_to_cv2(frame)
    cv2.imshow("Image window", cv_frame)
    cv2.waitKey(3)

    
def show_frame():

    
    rospy.init_node('show_frame', anonymous=True)

    rospy.Subscriber("frame_chatter", Image, chatter_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    show_frame()
