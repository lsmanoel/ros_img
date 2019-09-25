#!/usr/bin/env python
# license removed for brevity
import roslib
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def talker_frame():
    
    video_capture = cv2.VideoCapture(0)

    pub = rospy.Publisher('frame_chatter', Image, queue_size=10)
    bridge = CvBridge()

    rospy.init_node('talker_img', anonymous=True)

    rate = rospy.Rate(30)
    i = 0
    while not rospy.is_shutdown():
        ret, frame = video_capture.read()
        frame_msg = "send FRAME %s" % i
        rospy.loginfo(frame_msg)
        pub.publish(bridge.cv2_to_imgmsg(frame))
        rate.sleep()
        i=i+1

if __name__ == '__main__':
    try:
        talker_frame()
    except rospy.ROSInterruptException:
        pass
