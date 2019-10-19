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

class CannyFilter(object):
    def __init__(self,
    			 name=None,
                 input_frame=None):

        if name is None:
        	self.name='canny_filter'
        else:
        	self.name=name

        rospy.init_node(self.name, anonymous=True)

        self.rate = rospy.Rate(30)#Hz

        self.bridge = CvBridge()
        self.input_frame = input_frame
        self.output_frame = None

        self.delta_t_buffer_size = 1000
        self._delta_t = []
        self.delta_t = 0

    # ----------------------------------------------------------------------------------------
    # rostopics
    def signals_publisher_init(self, rostopic_name=None):
        if rostopic_name is None:
            self.pub_output = rospy.Publisher(self.name+'_output', Image, queue_size=10)
        else:
            self.pub_output = rospy.Publisher(rostopic_name, Image, queue_size=10)

    def signals_subscriber_init(self, rostopic_name=None):
        if rostopic_name is None:
            rospy.Subscriber(self.name+'_input', Image, self.input_frame_callback)
        else:
            rospy.Subscriber(rostopic_name, Image, self.input_frame_callback)

    def input_frame_callback(self, frame):
        self.input_frame = self.bridge.imgmsg_to_cv2(frame)

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
    # property
    @property
    def delta_t(self):
        return self._delta_t[-1]

    @delta_t.setter
    def delta_t(self, value):
        if len(self._delta_t) > self.delta_t_buffer_size:
            self._delta_t.pop(0)
        self._delta_t.append(value)

    # ----------------------------------------------------------------------------------------
    # rosservices
    def delta_t_service_init(self):
        self._delta_t_service = rospy.Service(self.name + '_delta_t_service', return_data, self.delta_t_service)

    def delta_t_service(self, msg):
        return return_dataResponse(self._delta_t)

    # ----------------------------------------------------------------------------------------
    # Main Loop        
    def main_loop(self):
        # ------------------------------------------
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.input_frame is not None:
                t0 = rospy.get_rostime().nsecs
                # -----------------------------------
                frame = self.input_frame.copy()
                frame = cv2.Canny(frame, 100, 200)
                frame = self.show_array(self.signature_histogram_generation(frame), frame.shape)

                # -----------------------------------
                # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.output_frame = frame.copy() 
                self.pub_output.publish(self.bridge.cv2_to_imgmsg(self.output_frame))

                t = rospy.get_rostime().nsecs
                delta_t = t - t0
                if delta_t > 0:
                    self.delta_t = delta_t
                       
        print("break")

# ======================================================================================================================
def canny_filter():
    canny_filter = CannyFilter()
    if len(sys.argv)>=3:      
        if sys.argv[1] == '-input_frame':
            canny_filter.signals_subscriber_init(rostopic_name=sys.argv[2])
        elif sys.argv[1] == '-output_frame':
            canny_filter.signals_publisher_init(rostopic_name=sys.argv[2])
    else:
        canny_filter.signals_subscriber_init()

    if len(sys.argv)>=5:      
        if sys.argv[3] == '-input_frame':
            canny_filter.signals_subscriber_init(rostopic_name=sys.argv[4])
        elif sys.argv[3] == '-output_frame':
            canny_filter.signals_publisher_init(rostopic_name=sys.argv[4])
    else:
        canny_filter.signals_publisher_init()
    
    canny_filter.delta_t_service_init()
    canny_filter.main_loop()

if __name__ == '__main__':
    try:
        canny_filter()
    except rospy.ROSInterruptException:
        pass
