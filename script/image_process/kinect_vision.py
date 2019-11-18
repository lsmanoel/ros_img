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

        self.histogram_t_buffer_size_L = 10
        self._histogram_t_L = []
        self.histogram_t_L = 0
        
        self.histogram_t_buffer_size_R = 10
        self._histogram_t_R = []
        self.histogram_t_R = 0

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

    def draw_roi(self, frame, roi_size, roi_pos=(0, 0), filled_roi=True, roi_color=None):
        roi = ((roi_pos[1]+frame.shape[1]//2-roi_size[1]//2, roi_pos[0]+frame.shape[0]//2-roi_size[0]//2),
               (roi_pos[1]+frame.shape[1]//2+roi_size[1]//2, roi_pos[0]+frame.shape[0]//2+roi_size[0]//2))

        if roi_color is None:
            data_roi = frame[roi_pos[0]+frame.shape[0]//2-roi_size[0]//2:roi_pos[0]+frame.shape[0]//2+roi_size[0]//2,
                             roi_pos[1]+frame.shape[1]//2-roi_size[1]//2:roi_pos[1]+frame.shape[1]//2+roi_size[1]//2]
            data_roi_max = int(np.argmax(self.histogram(data_roi)))
            roi_color = (data_roi_max, data_roi_max, data_roi_max)

        if filled_roi is True:
            cv2.rectangle(frame, 
                          roi[0],
                          roi[1], 
                          roi_color,
                          cv2.FILLED, 
                          1)

        cv2.rectangle(frame, 
                      roi[0],
                      roi[1], 
                      (0, 0, 255), 
                      1)

        return frame

    def histogram_xy(self, frame, histogram_size, histogram_pos=(0, 0)):
        histogram = self.histogram(frame[histogram_pos[0]+frame.shape[0]//2-histogram_size[0]//2:
                                         histogram_pos[0]+frame.shape[0]//2+histogram_size[0]//2,
                                         histogram_pos[1]+frame.shape[1]//2-histogram_size[1]//2:
                                         histogram_pos[1]+frame.shape[1]//2+histogram_size[1]//2])
        return histogram

    @property
    def histogram_t(self):
        return int(np.argmax(self.histogram(np.asarray(self._histogram_t))))

    @histogram_t.setter
    def histogram_t(self, value):
        if len(self._histogram_t) > self.histogram_t_buffer_size:
            self._histogram_t.pop(0)
        self._histogram_t.append(value)

    @property
    def histogram_t_L(self):
        return int(np.argmax(self.histogram(np.asarray(self._histogram_t_L))))

    @histogram_t_L.setter
    def histogram_t_L(self, value):
        if len(self._histogram_t_L) > self.histogram_t_buffer_size_L:
            self._histogram_t_L.pop(0)
        self._histogram_t_L.append(value)

    @property
    def histogram_t_R(self):
        return int(np.argmax(self.histogram(np.asarray(self._histogram_t_R))))

    @histogram_t_R.setter
    def histogram_t_R(self, value):
        if len(self._histogram_t_R) > self.histogram_t_buffer_size_R:
            self._histogram_t_R.pop(0)
        self._histogram_t_R.append(value)

    # ----------------------------------------------------------------------------------------
    # Kinect
    def display_depth(self, dev, data, timestamp):
        t0 = rospy.get_rostime().nsecs

        # **************************
        frame_stereo = np.float32(data)
        frame_stereo_raw = np.uint8(frame_stereo)
        frame_stereo = frame_stereo_raw.copy()
        # print(frame_stereo.shape)

        # # -----------------------------------------------------------------------------------------------------------------------
        # # Center
        # histogram_size = [100, 10]
        # self.histogram_t = int(np.argmax(self.histogram_xy(frame_stereo, histogram_size)))     
        # histogram_max_value = self.histogram_t

        # # -----------------------------------------------------------------------------------------------------------------------
        # # Left
        # histogram_size_L = [100, 10]
        # self.histogram_t_L = int(np.argmax(self.histogram_xy(frame_stereo, histogram_size_L, histogram_pos=(0, -100))))    
        # histogram_max_value_L = self.histogram_t_L

        # # -----------------------------------------------------------------------------------------------------------------------
        # # Right
        # histogram_size_R = [100, 10]
        # self.histogram_t_R = int(np.argmax(self.histogram_xy(frame_stereo, histogram_size_R, histogram_pos=(0, 100))))
        # histogram_max_value_R = self.histogram_t_R

        # # -----------------------------------------------------------------------------------------------------------------------
        frame_stereo = cv2.cvtColor(np.uint8(frame_stereo), cv2.COLOR_GRAY2BGR);
        frame_stereo_raw = cv2.cvtColor(np.uint8(frame_stereo_raw), cv2.COLOR_GRAY2BGR);
       
        # frame_stereo = self.crosshairs(frame_stereo)

        # frame_stereo = self.draw_roi(frame_stereo, 
        #                              histogram_size, 
        #                              roi_color=(histogram_max_value, histogram_max_value, histogram_max_value))

        # frame_stereo = self.draw_roi(frame_stereo, 
        #                              histogram_size_L,
        #                              roi_pos =(0, -100), 
        #                              roi_color=(histogram_max_value_L, histogram_max_value_L, histogram_max_value_L))

        # frame_stereo = self.draw_roi(frame_stereo, 
        #                              histogram_size_R,
        #                              roi_pos =(0, 100), 
        #                              roi_color=(histogram_max_value_R, histogram_max_value_R, histogram_max_value_R))


        # font = cv2.FONT_HERSHEY_SIMPLEX
        # cv2.putText(frame_stereo, 
        #             str(histogram_max_value), 
        #             (frame_stereo.shape[0]//2+200, frame_stereo.shape[1]//2+130), 
        #             font, 3, (0, 0, 255), 2, cv2.LINE_AA)

        # cv2.putText(frame_stereo, 
        #             str(histogram_max_value_L), 
        #             (frame_stereo.shape[0]//2, frame_stereo.shape[1]//2+130), 
        #             font, 3, (0, 0, 255), 2, cv2.LINE_AA)

        # cv2.putText(frame_stereo, 
        #             str(histogram_max_value_R), 
        #             (frame_stereo.shape[0]//2-200, frame_stereo.shape[1]//2+130), 
        #             font, 3, (0, 0, 255), 2, cv2.LINE_AA)


        self.pub_depth_output.publish(self.bridge.cv2_to_imgmsg(frame_stereo, self.output_frame_type))
        self.pub_depth_raw_output.publish(self.bridge.cv2_to_imgmsg(frame_stereo_raw, self.output_frame_type))
        # self.pub_histogram.publish(histogram_max_value)
        # self.pub_histogram_L.publish(histogram_max_value_L)
        # self.pub_histogram_R.publish(histogram_max_value_R)

        # **************************  
        t = rospy.get_rostime().nsecs
        delta_t = t - t0
        if delta_t > 0:
            self.delta_t = delta_t

    def display_rgb(self, dev, data, timestamp):
        frame = np.uint8(data)
        frame = cv2.cvtColor(np.uint8(frame), cv2.COLOR_RGB2BGR)
        # frame = self.crosshairs(frame)
        self.pub_bgr8_output.publish(self.bridge.cv2_to_imgmsg(frame, self.output_frame_type))

    def body(self, dev, ctx):
        freenect.set_tilt_degs(dev, 0)
        if rospy.is_shutdown():
            raise freenect.Kill
 
    # ----------------------------------------------------------------------------------------
    # rostopics
    def output_frame_publisher_init(self, rostopic_name=None):
        if rostopic_name is None:
            self.pub_bgr8_output = rospy.Publisher(self.name + '_output_frame/bgr8', Image, queue_size=10)
            self.pub_depth_output = rospy.Publisher(self.name + '_output_frame/depth', Image, queue_size=10)
            self.pub_depth_raw_output = rospy.Publisher(self.name + '_output_frame/depth_raw', Image, queue_size=10)
            self.pub_histogram = rospy.Publisher(self.name + '_central_depth_histogram', UInt8, queue_size=10)
            self.pub_histogram_L = rospy.Publisher(self.name + '_central_depth_histogram_L', UInt8, queue_size=10)
            self.pub_histogram_R = rospy.Publisher(self.name + '_central_depth_histogram_R', UInt8, queue_size=10)
        else:
            self.pub_bgr8_output = rospy.Publisher(rostopic_name + '/bgr8', Image, queue_size=10)
            self.pub_depth_output = rospy.Publisher(rostopic_name + '/depth', Image, queue_size=10)
            self.pub_depth_raw_output = rospy.Publisher(rostopic_name + '/depth_raw', Image, queue_size=10)
            self.pub_histogram = rospy.Publisher(rostopic_name + '_central_depth_histogram', UInt8, queue_size=10)
            self.pub_histogram_L = rospy.Publisher(rostopic_name + '_central_depth_histogram_L', UInt8, queue_size=10)
            self.pub_histogram_R = rospy.Publisher(rostopic_name + '_central_depth_histogram_R', UInt8, queue_size=10)

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