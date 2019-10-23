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
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError

class ImageProcess(object):
    def __init__(self,
                 name=None,
                 rate=30,
                 delta_t_buffer_size=1000):

        if name is None:
            self.name='image_process'
        else:
            self.name=name

        rospy.init_node(self.name, anonymous=True)

        self.rate = rate
        self.rate = rospy.Rate(self.rate)#Hz

        self.delta_t_buffer_size = delta_t_buffer_size
        self._delta_t = []
        self.delta_t = 0

        self.bridge = CvBridge()
        self.input_frame = None
        self.input_frame_flag = None
        self.output_frame = None
        self.pub_output = None

    # ----------------------------------------------------------------------------------------
    # rostopics
    def output_frame_publisher_init(self, rostopic_name=None):
        if rostopic_name is None:
            self.pub_output = rospy.Publisher(self.name + '_output_frame', Image, queue_size=10)
        else:
            self.pub_output = rospy.Publisher(rostopic_name, Image, queue_size=10)

    # ----
    def input_frame_callback(self, frame):
        self.input_frame = self.bridge.imgmsg_to_cv2(frame)
        self.input_frame_flag = True

    def input_frame_subscriber_init(self, rostopic_name=None):
        if rostopic_name is None:
            rospy.Subscriber(self.name+'_input_frame', Image, self.input_frame_callback)
        else:
            rospy.Subscriber(rostopic_name, Image, self.input_frame_callback)

    # ----------------------------------------------------------------------------------------
    # rosservices
    def delta_t_service_init(self):
        self._delta_t_service = rospy.Service(self.name + '_delta_t_service', return_data, self.delta_t_service)

    def delta_t_service(self, msg):
        return return_dataResponse(self._delta_t)

    # ----
    @property
    def delta_t(self):
        return self._delta_t[-1]

    @delta_t.setter
    def delta_t(self, value):
        if len(self._delta_t) > self.delta_t_buffer_size:
            self._delta_t.pop(0)
        self._delta_t.append(value)

    # ----------------------------------------------------------------------------------------
    # Main Loop        
    def main_loop(self):
        # ------------------------------------------
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.input_frame is not None and self.input_frame_flag is not None:
                self.input_frame_flag = None
                # --------------------------
                t0 = rospy.get_rostime().nsecs
                # **************************
                self.main_process()
                # **************************         
                if self.pub_output is not None: 
                    self.pub_output.publish(self.bridge.cv2_to_imgmsg(self.output_frame))
                # --------------------------
                t = rospy.get_rostime().nsecs
                delta_t = t - t0
                if delta_t > 0:
                    self.delta_t = delta_t

        # ------------------------------------------               
        print("break")

    def main_process(self):
        frame = self.input_frame.copy()
        # **************************
        # PROCESS
        # **************************
        self.output_frame = frame.copy() 
        

# ======================================================================================================================
def image_process():
    image_process = ImageProcess()

    if len(sys.argv)==3:
        if sys.argv[1] == '-input':
            image_process.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            image_process.output_frame_publisher_init()
        elif sys.argv[1] == '-output':
            image_process.output_frame_publisher_init(rostopic_name=sys.argv[2])
            image_process.input_frame_subscriber_init()
        else:
            image_process.input_frame_subscriber_init()
            image_process.output_frame_publisher_init()
    elif len(sys.argv)==5:      
        if sys.argv[1] == '-input':
            image_process.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            image_process.output_frame_publisher_init(rostopic_name=sys.argv[4])
        elif sys.argv[1] == '-output':
            image_process.output_frame_publisher_init(rostopic_name=sys.argv[2])
            image_process.input_frame_subscriber_init(rostopic_name=sys.argv[4])
        else:
            image_process.input_frame_subscriber_init()
            image_process.output_frame_publisher_init()

    else:
        image_process.input_frame_subscriber_init()
        image_process.output_frame_publisher_init()
    
    image_process.delta_t_service_init()
    image_process.main_loop()

if __name__ == '__main__':
    try:
        image_process()
    except rospy.ROSInterruptException:
        pass

