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

sys.path.insert(0, '/home/lucas/libfreenect/build/wrappers/python/python2')
import freenect

class KinectVision(ImageProcess):
    def __init__(self,
             name=None,
             rate=30):

        if name is None:
            name='kinect_vision'

        super(KinectVision, self).__init__(name=name)

        self.rate = rate
        self.stereo = cv2.StereoBM_create(numDisparities=32, blockSize=25)
        self.frame_stereo_acc = np.zeros((480, 640))

        self.histogram_t_buffer_size = 10
        self._histogram_t = []
        self.histogram_t = 0

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

    @property
    def histogram_t(self):
        return int(np.argmax(self.histogram(np.asarray(self._histogram_t))))

    @histogram_t.setter
    def histogram_t(self, value):
        if len(self._histogram_t) > self.histogram_t_buffer_size:
            self._histogram_t.pop(0)
        self._histogram_t.append(value)

    # ----------------------------------------------------------------------------------------
    # Kinect
    def display_depth(self, dev, data, timestamp):
        t0 = rospy.get_rostime().nsecs
        # **************************
        # frame_stereo = np.float32(data)
        # self.frame_stereo_acc = cv2.accumulateWeighted(frame_stereo, self.frame_stereo_acc, 0.5)
        # frame_stereo = self.frame_stereo_acc.copy()
        frame_stereo_raw = np.uint8(data)

        # frame_stereo_raw = cv2.cvtColor(frame_stereo_raw, cv2.COLOR_RGB2BGR)

        frame_stereo = frame_stereo_raw.copy()

        # -----------------------------------------------------------------------------------------------------------------------
        histogram_size = [10, 100]
        # n_grid = [(FRAME_SIZE[0]//(2*histogram_size[0]))-1, (FRAME_SIZE[1]//(2*histogram_size[1]))-1]
        n_grid = [1, 1]

        for j in range(1-n_grid[0], n_grid[0]):
            histogram = self.histogram(frame_stereo[240-histogram_size[1]//2:240+histogram_size[1]//2,
                                                    j*histogram_size[0]+320-histogram_size[0]//2:j*histogram_size[0]+320+histogram_size[0]//2])

        histogram_max_value = int(np.argmax(histogram))  

        print('histogram_max_value:', histogram_max_value)    
        self.histogram_t = histogram_max_value
        histogram_max_value = self.histogram_t
        print('histogram_t:', histogram_max_value)
        # -----------------------------------------------------------------------------------------------------------------------
        frame_stereo = cv2.cvtColor(np.uint8(frame_stereo), cv2.COLOR_GRAY2BGR);
        frame_stereo_raw = cv2.cvtColor(np.uint8(frame_stereo_raw), cv2.COLOR_GRAY2BGR);
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame_stereo, 
                    str(histogram_max_value), 
                    (240+120, 320+130), 
                    font, 3, (0, 0, 255), 2, cv2.LINE_AA)

        
        frame_stereo = self.crosshairs(frame_stereo)

        cv2.rectangle(frame_stereo, 
                  (320-histogram_size[0]*n_grid[0]+histogram_size[0]//2, 
                   240-histogram_size[1]*n_grid[1]+histogram_size[1]//2), 
                  (320+histogram_size[0]*n_grid[0]-histogram_size[0]//2, 
                   240+histogram_size[1]*n_grid[1]-histogram_size[1]//2), 
                  (histogram_max_value, histogram_max_value, histogram_max_value),
                  cv2.FILLED, 
                  1)

        cv2.rectangle(frame_stereo, 
                      (320-histogram_size[0]*n_grid[0]+histogram_size[0]//2, 
                       240-histogram_size[1]*n_grid[1]+histogram_size[1]//2), 
                      (320+histogram_size[0]*n_grid[0]-histogram_size[0]//2, 
                       240+histogram_size[1]*n_grid[1]-histogram_size[1]//2), 
                      (0, 0, 255), 
                      1)

        self.pub_depth_output.publish(self.bridge.cv2_to_imgmsg(frame_stereo, self.output_frame_type))
        self.pub_depth_raw_output.publish(self.bridge.cv2_to_imgmsg(frame_stereo_raw, self.output_frame_type))
        self.pub_histogram.publish(histogram_max_value)
        # **************************  
        t = rospy.get_rostime().nsecs

        delta_t = t - t0
        if delta_t > 0:
            self.delta_t = delta_t

    def display_rgb(self, dev, data, timestamp):
        frame_L = np.uint8(data)
        frame_L = cv2.cvtColor(np.uint8(frame_L), cv2.COLOR_RGB2BGR)
        frame_L = self.crosshairs(frame_L)
        self.pub_L_output.publish(self.bridge.cv2_to_imgmsg(data, self.output_frame_type))

    def body(self, *args):
        if rospy.is_shutdown():
            raise freenect.Kill
 
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
        print("KinectVision.main_loop()")
        print('Press ESC in window to stop')

        freenect.runloop(depth=self.display_depth,
                         video=self.display_rgb,
                         body=self.body)


# ======================================================================================================================
def kinect_vision():
    kinect_vision = KinectVision()

    if len(sys.argv)==4:
        if sys.argv[1] == '-output':
            kinect_vision.output_frame_publisher_init(rostopic_name=sys.argv[2])
            print(2)

        else:
            kinect_vision.output_frame_publisher_init()
            print(3)
    else:
        kinect_vision.output_frame_publisher_init()
        print(7)
  
    kinect_vision.delta_t_service_init()
    kinect_vision.main_loop()

if __name__ == '__main__':
    try:
        kinect_vision()
    except rospy.ROSInterruptException:
        pass