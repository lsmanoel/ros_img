#!/usr/bin/env python
# license removed for brevity
import roslib
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# ======================================================================================================================
def pub_frame():
    
    #video_capture = cv2.VideoCapture('/home/lucas/Videos/driver_1.mp4')
    video_capture = cv2.VideoCapture(0)
    pub = rospy.Publisher('/pub_frame_output_frame', Image, queue_size=10)
    bridge = CvBridge()

    rospy.init_node('pub_frame', anonymous=True)

    rate = rospy.Rate(25)
    i = 0
    while not rospy.is_shutdown():
        ret, frame = video_capture.read()
        frame_msg = "send FRAME %s" % i 
        i=i+1       
        #rospy.loginfo(frame_msg)

        pub.publish(bridge.cv2_to_imgmsg(frame))

        rate.sleep()

if __name__ == '__main__':
    try:
        pub_frame()
    except rospy.ROSInterruptException:
        pass
