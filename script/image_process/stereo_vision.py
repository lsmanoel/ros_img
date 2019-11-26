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
        self.stereo = cv2.StereoBM_create(numDisparities=200, blockSize=100)


    @staticmethod
    def histogram(frame):
        return np.histogram(frame.flatten(), 256, [0, 256])[0]

    @property
    def histogram_t(self):
        return int(np.argmax(self.histogram(np.asarray(self._histogram_t))))

    @histogram_t.setter
    def histogram_t(self, value):
        if len(self._histogram_t) > self.histogram_t_buffer_size:
            self._histogram_t.pop(0)
        self._histogram_t.append(value)

    # ----------------------------------------------------------------------------------------
    @property
    def video_source_L(self):
        return self._video_source_L

    @video_source_L.setter
    def video_source_L(self, value):
        self._video_source_L = int(value)
        self.video_capture_L = cv2.VideoCapture(int(value))
        self.video_capture_L.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")) 
        self.video_capture_L.set(cv2.CAP_PROP_AUTOFOCUS, 0)

    @property
    def video_source_R(self):
        return self._video_source_R

    @video_source_R.setter
    def video_source_R(self, value):
        self._video_source_R = int(value)
        self.video_capture_R = cv2.VideoCapture(int(value))
        self.video_capture_R.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.video_capture_R.set(cv2.CAP_PROP_AUTOFOCUS, 0) 
 
    # ----------------------------------------------------------------------------------------
    # rostopics
    def output_frame_publisher_init(self, rostopic_name=None):
        if rostopic_name is None:
            self.pub_bgr8_output = rospy.Publisher(self.name + '_output_frame/bgr8', Image, queue_size=10)
            self.pub_depth_output = rospy.Publisher(self.name + '_output_frame/depth', Image, queue_size=10)
            self.pub_depth_raw_output = rospy.Publisher(self.name + '_output_frame/depth_raw', Image, queue_size=10)
            self.pub_histogram = rospy.Publisher("/d_C", UInt8, queue_size=10)
            self.pub_histogram_L = rospy.Publisher("/d_L", UInt8, queue_size=10)
            self.pub_histogram_R = rospy.Publisher("/d_R", UInt8, queue_size=10)
        else:
            self.pub_bgr8_output = rospy.Publisher(rostopic_name + '/bgr8', Image, queue_size=10)
            self.pub_depth_output = rospy.Publisher(rostopic_name + '/depth', Image, queue_size=10)
            self.pub_depth_raw_output = rospy.Publisher(rostopic_name + '/depth_raw', Image, queue_size=10)
            self.pub_histogram = rospy.Publisher('/d_C', UInt8, queue_size=10)
            self.pub_histogram_L = rospy.Publisher('/d_L', UInt8, queue_size=10)
            self.pub_histogram_R = rospy.Publisher('/d_R', UInt8, queue_size=10)

    # ----------------------------------------------------------------------------------------
    # Main Loop 
    def main_loop(self):
        print("StereoVision.main_loop()")

        frame_L_acc = np.zeros((480, 640))
        frame_R_acc = np.zeros((480, 640))
        frame_stereo_acc = np.zeros((480, 640))

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
            print("StereoVision.process_bulk()")
            t0 = rospy.get_rostime().nsecs

            # **************************
            # PROCESS
            print("StereoVision.main_process()")
            # if not (self.video_capture_L.grab() and self.video_capture_R.grab()):
            #     print("No more frames")
            #     break
            # ret, frame_L = self.video_capture_L.retrieve()
            # ret, frame_R = self.video_capture_R.retrieve()

            ret, frame_L = self.video_capture_L.read()
            ret, frame_R = self.video_capture_R.read()

            frame_L = np.float32(frame_L)
            frame_R = np.float32(frame_R)

            frame_L = cv2.cvtColor(frame_L, cv2.COLOR_BGR2GRAY)
            frame_R = cv2.cvtColor(frame_R, cv2.COLOR_BGR2GRAY)

            # frame_L = cv2.GaussianBlur(frame_L, (3, 3), 0)
            # frame_R = cv2.GaussianBlur(frame_R, (3, 3), 0)
            frame_L = cv2.warpAffine(frame_L, M_rot_L, FRAME_SIZE)  
            frame_R = cv2.warpAffine(frame_R, M_rot_R, FRAME_SIZE)

            # frame_L_acc = cv2.accumulateWeighted(frame_L, frame_L_acc, 0.33)
            # frame_R_acc = cv2.accumulateWeighted(frame_R, frame_R_acc, 0.33)
            # frame_L = frame_L_acc.copy()
            # frame_R = frame_R_acc.copy()           

            frame_stereo = stereo.compute(np.uint8(frame_L), np.uint8(frame_R))
            frame_stereo = cv2.convertScaleAbs(frame_stereo)
            # frame_stereo = cv2.GaussianBlur(frame_stereo, 
            #                                 (3, 3), 
            #                                 0)
            frame_stereo = np.float32(frame_stereo)

            # frame_stereo_acc = cv2.accumulateWeighted(frame_stereo, frame_stereo_acc, 0.2)
            # frame_stereo = frame_stereo_acc.copy()

            frame_stereo_raw = frame_stereo.copy()
            # -----------------------------------------------------------------------------------------------------------------------
            # Center
            histogram_size = [100, 10]
            self.histogram_t = int(np.argmax(self.histogram_xy(frame_stereo, histogram_size)))     
            histogram_max_value = self.histogram_t

            # -----------------------------------------------------------------------------------------------------------------------
            # Left
            histogram_size_L = [100, 10]
            self.histogram_t_L = int(np.argmax(self.histogram_xy(frame_stereo, histogram_size_L, histogram_pos=(0, -100))))    
            histogram_max_value_L = self.histogram_t_L

            # -----------------------------------------------------------------------------------------------------------------------
            # Right
            histogram_size_R = [100, 10]
            self.histogram_t_R = int(np.argmax(self.histogram_xy(frame_stereo, histogram_size_R, histogram_pos=(0, 100))))
            histogram_max_value_R = self.histogram_t_R

            # -----------------------------------------------------------------------------------------------------------------------
            frame_stereo = cv2.cvtColor(np.uint8(frame_stereo), cv2.COLOR_GRAY2BGR);
            frame_stereo_raw = cv2.cvtColor(np.uint8(frame_stereo_raw), cv2.COLOR_GRAY2BGR);
           
            frame_stereo = self.crosshairs(frame_stereo)

            frame_stereo = self.draw_roi(frame_stereo, 
                                         histogram_size, 
                                         roi_color=(histogram_max_value, histogram_max_value, histogram_max_value))

            frame_stereo = self.draw_roi(frame_stereo, 
                                         histogram_size_L,
                                         roi_pos =(0, -100), 
                                         roi_color=(histogram_max_value_L, histogram_max_value_L, histogram_max_value_L))

            frame_stereo = self.draw_roi(frame_stereo, 
                                         histogram_size_R,
                                         roi_pos =(0, 100), 
                                         roi_color=(histogram_max_value_R, histogram_max_value_R, histogram_max_value_R))


            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame_stereo, 
                        str(histogram_max_value_R), 
                        (frame_stereo.shape[0]//2+200, frame_stereo.shape[1]//2+130), 
                        font, 3, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.putText(frame_stereo, 
                        str(histogram_max_value), 
                        (frame_stereo.shape[0]//2, frame_stereo.shape[1]//2+130), 
                        font, 3, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.putText(frame_stereo, 
                        str(histogram_max_value_L), 
                        (frame_stereo.shape[0]//2-200, frame_stereo.shape[1]//2+130), 
                        font, 3, (0, 0, 255), 2, cv2.LINE_AA)


            self.pub_depth_output.publish(self.bridge.cv2_to_imgmsg(frame_stereo, self.output_frame_type))
            self.pub_depth_raw_output.publish(self.bridge.cv2_to_imgmsg(frame_stereo_raw, self.output_frame_type))
            self.pub_histogram.publish(histogram_max_value)
            self.pub_histogram_L.publish(histogram_max_value_L)
            self.pub_histogram_R.publish(histogram_max_value_R)
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