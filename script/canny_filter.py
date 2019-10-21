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

class CannyFilter(ImageProcess):
    def __init__(self,
                 name=None,
                 rate=30,
                 delta_t_buffer_size=1000):

        if name is None:
            name='canny_filter'

        super(CannyFilter, self).__init__(name=name,
                                          rate=rate,
                                          delta_t_buffer_size=delta_t_buffer_size)

    # ----------------------------------------------------------------------------------------
    @staticmethod
    def signature_histogram_generation(frame):
        # frame_output = np.zeros((frame_input.shape))
        histogram = np.zeros(frame.shape[0])

        for i, line in enumerate(frame[:,]):
            line_energy = np.sum(line/255)
            # print(line)
            histogram[i] = int(line_energy)

        return histogram

    @staticmethod
    def show_array(array, output_frame_shape):
        output_frame = np.zeros(output_frame_shape)
        #print(frame_output_shape)
        for i, value in enumerate(array):
            value = int(value)
            if value >= output_frame_shape[1]:
                value = output_frame_shape[1]
            #print(int(value))
            output_frame[i,:int(value)] = np.ones(int(value))

        return output_frame

    # ----------------------------------------------------------------------------------------
    # Main Loop        
    def main_process(self):
        frame = self.input_frame.copy()
        # **************************
        frame = cv2.Canny(frame, 100, 200)
        frame = self.show_array(self.signature_histogram_generation(frame), frame.shape)

        # -----------------------------------
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # **************************
        self.output_frame = frame.copy() 


# ======================================================================================================================
def canny_filter():
    canny_filter = CannyFilter()

    if len(sys.argv)==3:
        if sys.argv[1] == '-input':
            canny_filter.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            canny_filter.output_frame_publisher_init()
        elif sys.argv[1] == '-output':
            canny_filter.output_frame_publisher_init(rostopic_name=sys.argv[2])
            canny_filter.input_frame_subscriber_init()
        else:
            canny_filter.input_frame_subscriber_init()
            canny_filter.output_frame_publisher_init()
    elif len(sys.argv)==5:      
        if sys.argv[1] == '-input':
            canny_filter.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            canny_filter.output_frame_publisher_init(rostopic_name=sys.argv[4])
        elif sys.argv[1] == '-output':
            canny_filter.output_frame_publisher_init(rostopic_name=sys.argv[2])
            canny_filter.input_frame_subscriber_init(rostopic_name=sys.argv[4])
        else:
            canny_filter.input_frame_subscriber_init()
            canny_filter.output_frame_publisher_init()

    else:
        canny_filter.input_frame_subscriber_init()
        canny_filter.output_frame_publisher_init()
    
    canny_filter.delta_t_service_init()
    canny_filter.main_loop()

if __name__ == '__main__':
    try:
        canny_filter()
    except rospy.ROSInterruptException:
        pass