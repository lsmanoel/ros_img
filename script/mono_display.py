#!/usr/bin/env python
import sys
import roslib
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ros_img.srv import return_data, return_dataResponse
from cv_bridge import CvBridge, CvBridgeError

# ap = argparse.ArgumentParser()
# ap.add_argument('-i', '--input_topic', required=True,
# 	help='name of the user')
# args = vars(ap.parse_args())

name='mono_display'

bridge = CvBridge()

FULL_FRAME_WIDTH = 640;
FULL_FRAME_HEIGHT = 480;
FULL_FRAME_SIZE = (FULL_FRAME_WIDTH, FULL_FRAME_HEIGHT)
FULL_FRAME_CENTER = (FULL_FRAME_WIDTH//2, FULL_FRAME_HEIGHT//2)

VIEW_FRAME_WIDTH = 320
VIEW_FRAME_HEIGHT = 240
VIEW_FRAME_SIZE = (VIEW_FRAME_WIDTH, VIEW_FRAME_HEIGHT)
VIEW_FRAME_CENTER = (VIEW_FRAME_WIDTH//2, VIEW_FRAME_HEIGHT//2)

M_rot_L = cv2.getRotationMatrix2D(FULL_FRAME_CENTER, 270, 1.0)
M_rot_R = cv2.getRotationMatrix2D(FULL_FRAME_CENTER, 90, 1.0)

_delta_t = []
delta_t_buffer_size=1000

# ----------------------------------------------------------------------------------------
# rostopics
def chatter_callback(frame):
    t0 = rospy.get_rostime().nsecs
    cv_frame = bridge.imgmsg_to_cv2(frame)
    cv2.imshow('mono_display', cv_frame)
    cv2.waitKey(5)

    t = rospy.get_rostime().nsecs
    delta_t = t - t0
    if delta_t > 0:
        pass
    else:
        delta_t = t - t0 + 2**32

# ----------------------------------------------------------------------------------------
# property
@property
def delta_t():
    return _delta_t[-1]

@delta_t.setter
def delta_t(value):
    if len(_delta_t) > delta_t_buffer_size:
        _delta_t.pop(0)
    _delta_t.append(value)

# ----------------------------------------------------------------------------------------
# rosservices
def delta_t_service(msg):
    return return_dataResponse(_delta_t)

def delta_t_service_init():
    return rospy.Service(name + '_delta_t_service', return_data, delta_t_service)

# ====================================================================================================================== 
def mono_display():
    rospy.init_node('mono_display', anonymous=True)
    rospy.Subscriber(sys.argv[1], Image, chatter_callback)
    _delta_t_service = delta_t_service_init()
    rospy.spin()

if __name__ == '__main__':
    mono_display()