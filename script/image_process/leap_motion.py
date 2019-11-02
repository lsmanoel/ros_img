#!/usr/bin/env python
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
sys.path.insert(0, "/home/lucas/LeapDeveloperKit_2.3.1+31549_linux/LeapSDK/lib")
sys.path.insert(1, "/home/lucas/LeapDeveloperKit_2.3.1+31549_linux/LeapSDK/lib/x64")
import Leap

class LeapMotion(ImageProcess):
    def __init__(self,
             	 name=None,
             	 rate=30):

        if name is None:
            name='leap_motion'

        super(LeapMotion, self).__init__(name=name)

        self.rate = rate
        self.stereo = cv2.StereoBM_create(numDisparities=32, blockSize=25)

    # ===========================================================
    #  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Moving Average Filter apply along the time axis
    # Order = buffer_size
    @staticmethod
    def init_moving_average(frame_shape, buffer_size = 3):
        buffer_pt = 0
        frame_buffer = []
        for i in range(buffer_size):
            frame_buffer.append(np.zeros(frame_shape))

        return frame_buffer, buffer_pt

    @staticmethod
    def moving_average(frame_input, frame_buffer, frame_pt):
        frame_output = frame_input
        buffer_size = len(frame_buffer)
        if frame_pt == buffer_size:
            frame_pt = 0

        frame_buffer[frame_pt] = frame_input
        frame_pt = frame_pt + 1

        for frame in frame_buffer:
            frame_output = frame_output + frame/buffer_size

        # frame_output = frame_output/len(frame_buffer)
        # print(len(frame_buffer))
        # 

        return frame_output, frame_buffer, frame_pt

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
    def main_loop(self):
        print("MonoVision.main_loop()")
        mm_1_buffer_global, mm_1_pt_global = LeapMotion.init_moving_average((480, 640), buffer_size = 3)
        stereo = cv2.StereoBM_create(numDisparities=32, blockSize=25)
        FRAME_SIZE = (
            self.VIEW_FRAME_WIDTH,
            self.VIEW_FRAME_HEIGHT
            )
        FRAME_CENTER = (
            self.VIEW_FRAME_WIDTH//2,
            self.VIEW_FRAME_HEIGHT//2
            )

        M_rot_L = cv2.getRotationMatrix2D(FRAME_CENTER, 270, 1.0)
        M_rot_R = cv2.getRotationMatrix2D(FRAME_CENTER, 90, 1.0)

        controller = Leap.Controller()

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
            #----------------------------------------------------------------------
            print("MonoVision.process_bulk()")
            t0 = rospy.get_rostime().nsecs
            # **************************
            # PROCESS
            controller.frame()
            # **************************  
            t = rospy.get_rostime().nsecs

            delta_t = t - t0
            if delta_t > 0:
                self.delta_t = delta_t


# ======================================================================================================================
def leap_motion():
    leap_motion = LeapMotion()

    if len(sys.argv)==4:
        if sys.argv[1] == '-input':
            leap_motion.video_source_L = sys.argv[2]
            leap_motion.video_source_R = sys.argv[3]                                                    
            leap_motion.output_frame_publisher_init()
            print(1)

        elif sys.argv[1] == '-output':
            leap_motion.output_frame_publisher_init(rostopic_name=sys.argv[2])
            print(2)

        else:
            leap_motion.output_frame_publisher_init()
            print(3)

    elif len(sys.argv)==6:      
        if sys.argv[1] == '-input':
            leap_motion.video_source_L = sys.argv[2]
            leap_motion.video_source_R = sys.argv[3]  
            leap_motion.output_frame_publisher_init(rostopic_name=sys.argv[5])
            print(4)

        elif sys.argv[1] == '-output':
            leap_motion.output_frame_publisher_init(rostopic_name=sys.argv[2])
            leap_motion.video_source_L = sys.argv[4]
            leap_motion.video_source_R = sys.argv[5]
            print(5)  

        else:
            leap_motion.output_frame_publisher_init()
            print(6)

    else:
        leap_motion.output_frame_publisher_init()
        print(7)
    


    leap_motion.delta_t_service_init()
    leap_motion.main_loop()

if __name__ == '__main__':
    try:
        leap_motion()
    except rospy.ROSInterruptException:
        pass