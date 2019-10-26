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

class StereoVision(ImageProcess):
    def __init__(self,
             name=None,
             rate=30,
             delta_t_buffer_size=1000):

        if name is None:
            name='stereo_vision'

        super(StereoVision, self).__init__(name=name,
                                           rate=rate,
                                           delta_t_buffer_size=delta_t_buffer_size)

        self.stereo = cv2.StereoBM_create(numDisparities=32, blockSize=25)

    # ===========================================================
    #  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Puts a centered crosshairs on the frame 
    @staticmethod
    def crosshairs(frame_input):
        frame_output = frame_input
        cv2.line(frame_output,
                 (0, frame_output.shape[0]//2),
                 (frame_output.shape[1], frame_output.shape[0]//2),
                 (255,0,0),
                 1)

        cv2.line(frame_output,
                 (frame_output.shape[1]//2, 0),
                 (frame_output.shape[1]//2, frame_output.shape[0]),
                 (255,0,0),
                 1)
        return frame_output

    # ----------------------------------------------------------------------------------------
    # Main Loop        
    def main_process(self):
        frame = self.input_frame.copy()
        # **************************
        frame_L = frame[:, :frame.shape[1]//2]
        frame_R = frame[:, frame.shape[1]//2:]

        frame_L = cv2.cvtColor(frame_L, cv2.COLOR_BGR2GRAY)
        frame_R = cv2.cvtColor(frame_R, cv2.COLOR_BGR2GRAY)

        frame_L = cv2.GaussianBlur(frame_L, 
        						   (3, 3), 
        						   0)

        frame_R = cv2.GaussianBlur(frame_R, 
        						   (3, 3), 
        						   0)

        frame_L = self.crosshairs(frame_L)
        frame_R = self.crosshairs(frame_R)

        print(frame.shape, frame_L.shape, frame_R.shape)
        # **************************
        self.output_frame = frame_R.copy() 

# ======================================================================================================================
def stereo_vision():
    stereo_vision = StereoVision()

    if len(sys.argv)==3:
        if sys.argv[1] == '-input':
            stereo_vision.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            stereo_vision.output_frame_publisher_init()
        elif sys.argv[1] == '-output':
            stereo_vision.output_frame_publisher_init(rostopic_name=sys.argv[2])
            stereo_vision.input_frame_subscriber_init()
        else:
            stereo_vision.input_frame_subscriber_init()
            stereo_vision.output_frame_publisher_init()
    elif len(sys.argv)==5:      
        if sys.argv[1] == '-input':
            stereo_vision.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            stereo_vision.output_frame_publisher_init(rostopic_name=sys.argv[4])
        elif sys.argv[1] == '-output':
            stereo_vision.output_frame_publisher_init(rostopic_name=sys.argv[2])
            stereo_vision.input_frame_subscriber_init(rostopic_name=sys.argv[4])
        else:
            stereo_vision.input_frame_subscriber_init()
            stereo_vision.output_frame_publisher_init()

    else:
        stereo_vision.input_frame_subscriber_init()
        stereo_vision.output_frame_publisher_init()
    
    stereo_vision.delta_t_service_init()
    stereo_vision.main_loop()

if __name__ == '__main__':
    try:
        stereo_vision()
    except rospy.ROSInterruptException:
        pass