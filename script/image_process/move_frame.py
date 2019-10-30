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

class MoveFrame(ImageProcess):
    def __init__(self,
                 name=None):

        if name is None:
            name='move_frame'

        super(MoveFrame, self).__init__(name=name)

        self.inputQuad = np.zeros((4, 2), dtype = "float32")
        self.outputQuad = np.zeros((4, 2), dtype = "float32")
        self.lambda_matrix = np.zeros((self.FULL_FRAME_HEIGHT, self.FULL_FRAME_WIDTH))

    # ----------------------------------------------------------------------------------------
    def rotate_frame_process(self, frame, frame_angle, frame_size):
        if frame_angle == 0:
            return frame
        else:
            return cv2.warpAffine(frame, self.M_rot[int(frame_angle)], frame_size)

    # ----------------------------------------------------------------------------------------
    # Main Loop        
    def main_process(self, frame):
        frame = self.input_frame.copy()
        # **************************

                   
        frame = cv2.warpAffine(frame, self.M_rot[int(self.frame_angle)], (frame.shape[1], frame.shape[0]))
        # --------------------------
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # **************************
        return frame 

    def main_process(self, frame):
        print("MoveFrame.main_process()")
        # **************************
        # PROCESS
        # The 4 points that select quadilateral on the input , from top-left in clockwise order
        # These four pts are the sides of the rect box used as input 
        self.inputQuad[0,:] = np.asarray([-30, -60 ]);
        self.inputQuad[1,:] = np.asarray([frame.shape[1]+50, -50]);
        self.inputQuad[2,:] = np.asarray([frame.shape[1]+100, frame.shape[0]+50]);
        self.inputQuad[3,:] = np.asarray([-50, frame.shape[0]+50]);
        # The 4 points where the mapping is to be done , from top-left in clockwise order
        self.outputQuad[0,:] = np.asarray([0, 0]);
        self.outputQuad[1,:] = np.asarray([frame.shape[1]-1, 0]);
        self.outputQuad[2,:] = np.asarray([frame.shape[1]-1, frame.shape[0]-1]);
        self.outputQuad[3,:] = np.asarray([0, frame.shape[0]-1]);

        #Get the Perspective Transform Matrix i.e. lambda_matrix 
        lambda_matrix = cv2.getPerspectiveTransform(self.inputQuad, self.outputQuad);
        frame = cv2.warpPerspective(frame, lambda_matrix, (self.FULL_FRAME_WIDTH, self.FULL_FRAME_HEIGHT));
        # **************************
        return frame 

# ======================================================================================================================
def move_frame():
    move_frame = MoveFrame()

    if len(sys.argv)==3:
        if sys.argv[1] == '-input':
            move_frame.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            move_frame.output_frame_publisher_init()
        elif sys.argv[1] == '-output':
            move_frame.output_frame_publisher_init(rostopic_name=sys.argv[2])
            move_frame.input_frame_subscriber_init()
        else:
            move_frame.input_frame_subscriber_init()
            move_frame.output_frame_publisher_init()
    elif len(sys.argv)==5:      
        if sys.argv[1] == '-input':
            move_frame.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            move_frame.output_frame_publisher_init(rostopic_name=sys.argv[4])
        elif sys.argv[1] == '-output':
            move_frame.output_frame_publisher_init(rostopic_name=sys.argv[2])
            move_frame.input_frame_subscriber_init(rostopic_name=sys.argv[4])
        else:
            move_frame.input_frame_subscriber_init()
            move_frame.output_frame_publisher_init()

    else:
        move_frame.input_frame_subscriber_init()
        move_frame.output_frame_publisher_init()
    
    move_frame.delta_t_service_init()
    move_frame.main_loop()

if __name__ == '__main__':
    try:
        move_frame()
    except rospy.ROSInterruptException:
        pass
