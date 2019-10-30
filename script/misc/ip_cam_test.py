#!/usr/bin/env python
# license removed for brevity
import roslib
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def ip_cam_test():   
    #video_capture = cv2.VideoCapture('/home/lucas/Videos/driver_1.mp4')
    video_capture = cv2.VideoCapture('rtsp://admin:admin123@10.66.0.11:554/cam/realmonitor?channel=1&subtype=0')

    i = 0
    while 1:
        ret, frame = video_capture.read()
        cv2.imshow('mono_display', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break    

if __name__ == '__main__':
    try:
        ip_cam_test()
    except rospy.ROSInterruptException:
        pass