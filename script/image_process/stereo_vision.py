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
from std_msgs.msg import UInt8
from cv_bridge import CvBridge, CvBridgeError
from image_process import ImageProcess

class StereoVision(ImageProcess):
    def __init__(self,
             name=None,
             rate=30):

        if name is None:
            name='stereo_vision'

        super(StereoVision, self).__init__(name=name)

        self.rate = rate
        self.stereo = cv2.StereoBM_create(numDisparities=32, blockSize=25)

    # ----------------------------------------------------------------------------------------
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
        buffer_size = len(frame_buffer)
        if frame_pt == buffer_size:
            frame_pt = 0

        frame_buffer[frame_pt] = frame_input
        frame_pt = frame_pt + 1

        frame_output = frame_buffer[0]/buffer_size
        for frame in frame_buffer[1:]:
            frame_output = frame_output + frame/buffer_size

        # frame_output = frame_output/len(frame_buffer)
        # print(len(frame_buffer))

        return frame_output, frame_buffer, frame_pt

    # ----------------------------------------------------------------------------------------
    # Puts a centered crosshairs on the frame 
    @staticmethod
    def crosshairs(frame_input, color=(0,0,255)):
        frame_output = frame_input
        cv2.line(frame_output,
                 (0, frame_output.shape[0]//2),
                 (frame_output.shape[1], frame_output.shape[0]//2),
                 color,
                 1)

        cv2.line(frame_output,
                 (frame_output.shape[1]//2, 0),
                 (frame_output.shape[1]//2, frame_output.shape[0]),
                 color,
                 1)
        return frame_output

    @staticmethod
    def histogram(frame):
        return np.histogram(frame.flatten(), 256, [0, 256])[0]

    # ----------------------------------------------------------------------------------------
    @property
    def video_source_L(self):
        return self._video_source_L

    @video_source_L.setter
    def video_source_L(self, value):
        self._video_source_L = int(value)
        self.video_capture_L = cv2.VideoCapture(int(value))
        self.video_capture_L.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")) 

    @property
    def video_source_R(self):
        return self._video_source_R

    @video_source_R.setter
    def video_source_R(self, value):
        self._video_source_R = int(value)
        self.video_capture_R = cv2.VideoCapture(int(value))
        self.video_capture_R.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")) 
 
    # ----------------------------------------------------------------------------------------
    # rostopics
    def output_frame_publisher_init(self, rostopic_name=None):
        if rostopic_name is None:
            self.pub_L_output = rospy.Publisher(self.name + '_output_frame/L', Image, queue_size=10)
            self.pub_R_output = rospy.Publisher(self.name + '_output_frame/R', Image, queue_size=10)
            self.pub_depth_output = rospy.Publisher(self.name + '_output_frame/depth', Image, queue_size=10)
            self.pub_depth_raw_output = rospy.Publisher(self.name + '_output_frame/depth_raw', Image, queue_size=10)
            self.pub_histogram = rospy.Publisher(self.name + '_central_depth_histogram', UInt8, queue_size=10)
        else:
            self.pub_L_output = rospy.Publisher(rostopic_name + '/L', Image, queue_size=10)
            self.pub_R_output = rospy.Publisher(rostopic_name + '/R', Image, queue_size=10)
            self.pub_depth_output = rospy.Publisher(rostopic_name + '/depth', Image, queue_size=10)
            self.pub_depth_raw_output = rospy.Publisher(rostopic_name + '/depth_raw', Image, queue_size=10)
            self.pub_histogram = rospy.Publisher(rostopic_name + '_central_depth_histogram', UInt8, queue_size=10)

    # ----------------------------------------------------------------------------------------
    # Main Loop 
    def main_loop(self):
        print("StereoVision.main_loop()")

        frame_L_acc = np.zeros((480, 640))
        frame_R_acc = np.zeros((480, 640))
        frame_stereo_acc = np.zeros((480, 640))

        mm_L_buffer, mm_L_pt = StereoVision.init_moving_average((480, 640), buffer_size = 10)
        mm_R_buffer, mm_R_pt = StereoVision.init_moving_average((480, 640), buffer_size = 10)
        mm_stereo_buffer, mm_stereo_pt = StereoVision.init_moving_average((480, 640), buffer_size = 10)

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

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
            #----------------------------------------------------------------------
            print("MonoVision.process_bulk()")
            t0 = rospy.get_rostime().nsecs

            # **************************
            # PROCESS
            print("MonoVision.main_process()")
            # if not (self.video_capture_L.grab() and self.video_capture_R.grab()):
            #     print("No more frames")
            #     break
            # ret, frame_L = self.video_capture_L.retrieve()
            # ret, frame_R = self.video_capture_R.retrieve()

            ret, frame_L = self.video_capture_L.read()
            ret, frame_R = self.video_capture_R.read()

            frame_L = np.float32(frame_L)
            frame_R = np.float32(frame_R)
            frame_L = cv2.warpAffine(frame_L, M_rot_L, FRAME_SIZE)  
            frame_R = cv2.warpAffine(frame_R, M_rot_R, FRAME_SIZE)

            frame_L = cv2.cvtColor(frame_L, cv2.COLOR_BGR2GRAY)
            frame_R = cv2.cvtColor(frame_R, cv2.COLOR_BGR2GRAY)

            # frame_L, mm_L_buffer, mm_L_pt = self.moving_average(frame_L, mm_L_buffer, mm_L_pt)
            # frame_R, mm_R_buffer, mm_R_pt = self.moving_average(frame_R, mm_R_buffer, mm_R_pt)
            frame_L_acc = cv2.accumulateWeighted(frame_L, frame_L_acc, 0.33)
            frame_R_acc = cv2.accumulateWeighted(frame_R, frame_R_acc, 0.33)
            frame_L = frame_L_acc.copy()
            frame_R = frame_R_acc.copy()    

            
            frame_L = cv2.GaussianBlur(frame_L, 
                                       (3, 3), 
                                       0)
            frame_R = cv2.GaussianBlur(frame_R, 
                                       (3, 3), 
                                       0)

            frame_stereo = stereo.compute(np.uint8(frame_L), np.uint8(frame_R))
            frame_stereo = cv2.convertScaleAbs(frame_stereo)
            frame_stereo = cv2.GaussianBlur(frame_stereo, 
                                            (3, 3), 
                                            0)
            frame_stereo = np.float32(frame_stereo)

            # frame_stereo, mm_stereo_buffer, mm_stereo_pt = self.moving_average(frame_stereo, mm_stereo_buffer, mm_stereo_pt)
            frame_stereo_acc = cv2.accumulateWeighted(frame_stereo, frame_stereo_acc, 0.1)
            frame_stereo = frame_stereo_acc.copy()

            frame_stereo_raw = frame_stereo.copy()
            # -----------------------------------------------------------------------------------------------------------------------
            histogram_size = [10, 220]
            # n_grid = [(FRAME_SIZE[0]//(2*histogram_size[0]))-1, (FRAME_SIZE[1]//(2*histogram_size[1]))-1]
            n_grid = [1, 1]

            for j in range(1-n_grid[0], n_grid[0]):
                histogram = self.histogram(frame_stereo[FRAME_CENTER[1]-histogram_size[1]//2:FRAME_CENTER[1]+histogram_size[1]//2,
                                                        j*histogram_size[0]+FRAME_CENTER[0]-histogram_size[0]//2:j*histogram_size[0]+FRAME_CENTER[0]+histogram_size[0]//2,])

                frame_stereo[FRAME_CENTER[1]-histogram_size[1]//2:FRAME_CENTER[1]+histogram_size[1]//2,
                             j*histogram_size[0]+FRAME_CENTER[0]-histogram_size[0]//2:j*histogram_size[0]+FRAME_CENTER[0]+histogram_size[0]//2] = np.argmax(histogram)

            # -----------------------------------------------------------------------------------------------------------------------
            frame_L = cv2.cvtColor(np.uint8(frame_L), cv2.COLOR_GRAY2BGR);
            frame_R = cv2.cvtColor(np.uint8(frame_R), cv2.COLOR_GRAY2BGR);
            frame_stereo = cv2.cvtColor(np.uint8(frame_stereo), cv2.COLOR_GRAY2BGR);
            frame_stereo_raw = cv2.cvtColor(np.uint8(frame_stereo_raw), cv2.COLOR_GRAY2BGR);

            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame_stereo, 
                        str(frame_stereo[FRAME_CENTER[1], FRAME_CENTER[0]][0]), 
                        (FRAME_CENTER[1]+120, FRAME_CENTER[0]+130), 
                        font, 3, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.rectangle(frame_stereo, 
                          (FRAME_CENTER[0]-histogram_size[0]*n_grid[0]+histogram_size[0]//2, 
                           FRAME_CENTER[1]-histogram_size[1]*n_grid[1]+histogram_size[1]//2), 
                          (FRAME_CENTER[0]+histogram_size[0]*n_grid[0]-histogram_size[0]//2, 
                           FRAME_CENTER[1]+histogram_size[1]*n_grid[1]-histogram_size[1]//2), 
                          (0, 0, 255), 
                          2)

            frame_L = self.crosshairs(frame_L)
            frame_R = self.crosshairs(frame_R)
            frame_stereo = self.crosshairs(frame_stereo)

            # cv2.imshow(self.name + '_L', frame_L)
            # cv2.imshow(self.name + '_R', frame_R)
            # cv2.imshow(self.name + '_stereo', frame_stereo)
            # cv2.imshow(self.name + '_stereo_raw', frame_stereo_raw)
            # cv2.waitKey(5)

            self.pub_L_output.publish(self.bridge.cv2_to_imgmsg(frame_L, self.output_frame_type))
            self.pub_R_output.publish(self.bridge.cv2_to_imgmsg(frame_R, self.output_frame_type))
            self.pub_depth_output.publish(self.bridge.cv2_to_imgmsg(frame_stereo, self.output_frame_type))
            self.pub_depth_raw_output.publish(self.bridge.cv2_to_imgmsg(frame_stereo_raw, self.output_frame_type))
            self.pub_histogram.publish(int(np.argmax(histogram)))
            # **************************  
            t = rospy.get_rostime().nsecs

            delta_t = t - t0
            if delta_t > 0:
                self.delta_t = delta_t


# ======================================================================================================================
def stereo_vision():
    stereo_vision = StereoVision()

    if len(sys.argv)==4:
        if sys.argv[1] == '-input':
            stereo_vision.video_source_L = sys.argv[2]
            stereo_vision.video_source_R = sys.argv[3]                                                    
            stereo_vision.output_frame_publisher_init()
            print(1)

        elif sys.argv[1] == '-output':
            stereo_vision.output_frame_publisher_init(rostopic_name=sys.argv[2])
            print(2)

        else:
            stereo_vision.output_frame_publisher_init()
            print(3)

    elif len(sys.argv)==6:      
        if sys.argv[1] == '-input':
            stereo_vision.video_source_L = sys.argv[2]
            stereo_vision.video_source_R = sys.argv[3]  
            stereo_vision.output_frame_publisher_init(rostopic_name=sys.argv[5])
            print(4)

        elif sys.argv[1] == '-output':
            stereo_vision.output_frame_publisher_init(rostopic_name=sys.argv[2])
            stereo_vision.video_source_L = sys.argv[4]
            stereo_vision.video_source_R = sys.argv[5]
            print(5)  

        else:
            stereo_vision.output_frame_publisher_init()
            print(6)

    else:
        stereo_vision.output_frame_publisher_init()
        print(7)
    


    stereo_vision.delta_t_service_init()
    stereo_vision.main_loop()

if __name__ == '__main__':
    try:
        stereo_vision()
    except rospy.ROSInterruptException:
        pass