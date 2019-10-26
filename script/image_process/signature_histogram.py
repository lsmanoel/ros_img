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

class SignatureHistogram(ImageProcess):
    def __init__(self,
                 name=None,
                 rate=30,
                 delta_t_buffer_size=1000):

        if name is None:
            name='signature_histogram'

        super(SignatureHistogram, self).__init__(name=name,
                                          rate=rate,
                                          delta_t_buffer_size=delta_t_buffer_size)

        self.h_histogram = np.zeros(self.VIEW_FRAME_HEIGHT)
        self.v_histogram = np.zeros(self.VIEW_FRAME_WIDTH)
    
    # ----------------------------------------------------------------------------------------
    def horizontal_edge_signature(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.Sobel(frame, cv2.CV_32F, 0, 1, -1)
        frame = cv2.convertScaleAbs(frame)
        frame = cv2.GaussianBlur(frame, (3, 3), 0)
        # ret, frame = cv2.threshold(frame, 120, 255, cv2.THRESH_BINARY)
        return frame

    def vertical_edge_signature(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.Sobel(frame, cv2.CV_32F, 1, 0, -1)
        frame = cv2.convertScaleAbs(frame)
        frame = cv2.GaussianBlur(frame, (3, 3), 0)
        # ret, frame = cv2.threshold(frame, 120, 255, cv2.THRESH_BINARY)
        return frame

    def horizontal_histogram(self, frame):
        for i, line in enumerate(frame[:,]):
            self.h_histogram[i] = int((np.sum(line)/(2*self.VIEW_FRAME_WIDTH))**2)

        frame = np.zeros(frame.shape)
        for i, value in enumerate(self.h_histogram):
            value = int(value)
            if value >= frame.shape[1]:
                value = frame.shape[1]
            frame[i, :int(value)] = np.ones(int(value))

        return frame

    def vertical_histogram(self, frame):
        for i, line in enumerate(frame.T[:,]):
            self.v_histogram[i] = int((np.sum(line)/(2*self.VIEW_FRAME_HEIGHT))**2)

        frame = np.zeros(frame.shape)
        for i, value in enumerate(self.v_histogram):
            value = int(value)
            if value >= frame.shape[0]:
                value = frame.shape[0]
            frame[:int(value), i] = np.ones(int(value))

        return frame

    # ----------------------------------------------------------------------------------------
    # Main Loop        
    def main_process(self):
        frame = self.input_frame.copy()
        # **************************
        frame = self.horizontal_edge_signature(frame)
        # frame = self.horizontal_histogram(frame)
        # **************************
        self.output_frame = frame.copy() 


# ======================================================================================================================
def signature_histogram():
    signature_histogram = SignatureHistogram()

    if len(sys.argv)==3:
        if sys.argv[1] == '-input':
            signature_histogram.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            signature_histogram.output_frame_publisher_init()
        elif sys.argv[1] == '-output':
            signature_histogram.output_frame_publisher_init(rostopic_name=sys.argv[2])
            signature_histogram.input_frame_subscriber_init()
        else:
            signature_histogram.input_frame_subscriber_init()
            signature_histogram.output_frame_publisher_init()
    elif len(sys.argv)==5:      
        if sys.argv[1] == '-input':
            signature_histogram.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            signature_histogram.output_frame_publisher_init(rostopic_name=sys.argv[4])
        elif sys.argv[1] == '-output':
            signature_histogram.output_frame_publisher_init(rostopic_name=sys.argv[2])
            signature_histogram.input_frame_subscriber_init(rostopic_name=sys.argv[4])
        else:
            signature_histogram.input_frame_subscriber_init()
            signature_histogram.output_frame_publisher_init()

    else:
        signature_histogram.input_frame_subscriber_init()
        signature_histogram.output_frame_publisher_init()
    
    signature_histogram.delta_t_service_init()
    signature_histogram.main_loop()

if __name__ == '__main__':
    try:
        signature_histogram()
    except rospy.ROSInterruptException:
        pass
