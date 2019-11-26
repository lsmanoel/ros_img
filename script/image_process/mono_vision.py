#!/usr/bin/env python2
import os
import sys
import time
import random
import numpy as np
import cv2
import roslib
import rospy
from ros_img.srv import return_data, return_dataResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_process import ImageProcess

class MonoVision(ImageProcess):
    def __init__(self,
                 name=None,
                 rate=30,
                 delta_t_buffer_size=1000):

        if name is None:
            name='mono_vision'

        super(MonoVision, self).__init__(name=name)

        self.delta_t_buffer_size = delta_t_buffer_size
        self.rate = rate
        self.video_source = 0

    def init_init_video_capture(self):
        self.video_capture = cv2.VideoCapture(self.video_source)
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.FULL_FRAME_WIDTH);
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FULL_FRAME_HEIGHT);
        self.bridge = CvBridge()

    # ----------------------------------------------------------------------------------------
    # Main Loop 
    def main_loop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
            self.process_bulk()
            # rospy.spinOnce()

    def main_process(self, frame):
        print("MonoVision.main_process()")
        # **************************
        # PROCESS
        _, frame = self.video_capture.read()
        # **************************
        return frame


# ======================================================================================================================
def mono_vision():
    mono_vision = MonoVision()

    if len(sys.argv)==3:
        if sys.argv[1] == '-input':
            mono_vision.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            mono_vision.output_frame_publisher_init()
        elif sys.argv[1] == '-output':
            mono_vision.output_frame_publisher_init(rostopic_name=sys.argv[2])
            mono_vision.input_frame_subscriber_init()
        else:
            mono_vision.input_frame_subscriber_init()
            mono_vision.output_frame_publisher_init()
    elif len(sys.argv)==5:      
        if sys.argv[1] == '-input':
            mono_vision.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            mono_vision.output_frame_publisher_init(rostopic_name=sys.argv[4])
        elif sys.argv[1] == '-output':
            mono_vision.output_frame_publisher_init(rostopic_name=sys.argv[2])
            mono_vision.input_frame_subscriber_init(rostopic_name=sys.argv[4])
        else:
            mono_vision.input_frame_subscriber_init()
            mono_vision.output_frame_publisher_init()

    else:
        mono_vision.input_frame_subscriber_init()
        mono_vision.output_frame_publisher_init()
    
    mono_vision.init_init_video_capture()
    mono_vision.delta_t_service_init()
    mono_vision.main_loop()

if __name__ == '__main__':
    try:
        mono_vision()
    except rospy.ROSInterruptException:
        pass
