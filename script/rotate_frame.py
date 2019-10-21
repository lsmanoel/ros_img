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

class RotateFrame(ImageProcess):
    def __init__(self,
                 name=None,
                 rate=30,
                 delta_t_buffer_size=1000,
                 frame_angle=180,
                 angle_table=None):

        if name is None:
            name='rotate_frame'

        super(RotateFrame, self).__init__(name=name,
                                          rate=rate,
                                          delta_t_buffer_size=delta_t_buffer_size)

        self.frame_angle = frame_angle
        self.angle_resolution = 2**8

        if angle_table is None:
            self.angle_table = []
            for i in range(-self.angle_resolution//2, self.angle_resolution//2):
                angle = (360/self.angle_resolution)*(i)
                self.angle_table.append(angle)
            print(len(self.angle_table))
        else:
            self.angle_table = angle_table

        self.M_rot = None

    # ----------------------------------------------------------------------------------------
    def rotate_frame_process(self, frame, frame_angle, frame_size):
        if frame_angle == 0:
            return frame
        else:
            return cv2.warpAffine(frame, self.M_rot[int(frame_angle)], frame_size)

    # ----------------------------------------------------------------------------------------
    # Main Loop        
    def main_process(self):
        frame = self.input_frame.copy()
        # **************************
        if self.M_rot is None:
        	self.M_rot = {}
        	for angle in self.angle_table:
        	    self.M_rot[angle] = cv2.getRotationMatrix2D((frame.shape[1]//2, frame.shape[0]//2), angle, 1.0)
                   
        frame = cv2.warpAffine(frame, self.M_rot[int(self.frame_angle)], (frame.shape[1], frame.shape[0]))
        # --------------------------
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # **************************
        self.output_frame = frame.copy() 


# ======================================================================================================================
def rotate_frame():
    rotate_frame = RotateFrame()

    if len(sys.argv)==3:
        if sys.argv[1] == '-input':
            rotate_frame.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            rotate_frame.output_frame_publisher_init()
        elif sys.argv[1] == '-output':
            rotate_frame.output_frame_publisher_init(rostopic_name=sys.argv[2])
            rotate_frame.input_frame_subscriber_init()
        else:
            rotate_frame.input_frame_subscriber_init()
            rotate_frame.output_frame_publisher_init()
    elif len(sys.argv)==5:      
        if sys.argv[1] == '-input':
            rotate_frame.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            rotate_frame.output_frame_publisher_init(rostopic_name=sys.argv[4])
        elif sys.argv[1] == '-output':
            rotate_frame.output_frame_publisher_init(rostopic_name=sys.argv[2])
            rotate_frame.input_frame_subscriber_init(rostopic_name=sys.argv[4])
        else:
            rotate_frame.input_frame_subscriber_init()
            rotate_frame.output_frame_publisher_init()

    else:
        rotate_frame.input_frame_subscriber_init()
        rotate_frame.output_frame_publisher_init()
    
    rotate_frame.delta_t_service_init()
    rotate_frame.main_loop()

if __name__ == '__main__':
    try:
        rotate_frame()
    except rospy.ROSInterruptException:
        pass
