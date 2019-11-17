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


class CameraCalibration(ImageProcess):
    def __init__(self, 
                 name=None):

        if name is None:
            name='camera_calibration'

        super(CameraCalibration, self).__init__(name=name)

        self.input_frame_type = 'bgr8' 
        self.output_frame_type = 'bgr8'

        self.main_state = 'start_state'

    # ----------------------------------------------------------------------------------------
    def main_machine_state(self, frame):
        print(self.main_state)

        if self.main_state == 'start_state':
            self.main_state = 'find_chess_state'
            self.start_state()

        elif self.main_state == 'find_chess_state':
            ret, frame = self.find_chess_state(frame)
            if ret is True:
                self.main_state = 'process_coefficients_state' 

        elif self.main_state == 'process_coefficients_state':
            self.main_state = 'undistort_state'
            self.process_coefficients_state(frame)

        elif self.main_state == 'undistort_state':
            frame = self.undistort_state(frame)

        return frame

    def start_state(self, chess_size=(7, 6)):
        self.chess_size = chess_size
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((chess_size[0]*chess_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:chess_size[0], 0:chess_size[1]].T.reshape(-1, 2)
        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.

    def find_chess_state(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(frame, (7, 6), None)
        # If found, add object points, image points (after refining them)
        if ret is True:
            self.ret = ret
            self.objpoints.append(self.objp)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            self.imgpoints.append(corners)
            # Draw and display the corners
            cv2.drawChessboardCorners(frame, (7, 6), corners, ret)

        return ret, frame

    def process_coefficients_state(self, frame):
        self.ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(self.objpoints, 
                                                                                    self.imgpoints, 
                                                                                    frame.shape[:-1], 
                                                                                    None, 
                                                                                    None)
        h,  w = frame.shape[:2]
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))

        tot_error = 0
        mean_error = 0
        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv2.projectPoints(self.objpoints[i], self.rvecs[i], self.tvecs[i], self.mtx, self.dist)
            error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            tot_error += error

        print("total error: ", mean_error/len(self.objpoints))

    def undistort_state(self, frame):
        # undistort
        frame = cv2.undistort(frame, self.mtx, self.dist, None, self.newcameramtx)
        # crop the image
        x, y, w, h = self.roi
        frame = frame[y:y+h, x:x+w]

        print(self.dist)
        return frame   	

    # ----------------------------------------------------------------------------------------
    # Main Loop        
    def main_process(self, frame):
        # **************************
        frame = self.main_machine_state(frame)
        # **************************
        return frame        


# ======================================================================================================================
def camera_calibration():
    camera_calibration = CameraCalibration()

    if len(sys.argv)==3:
        if sys.argv[1] == '-input':
            camera_calibration.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            camera_calibration.output_frame_publisher_init()
        elif sys.argv[1] == '-output':
            camera_calibration.output_frame_publisher_init(rostopic_name=sys.argv[2])
            camera_calibration.input_frame_subscriber_init()
        else:
            camera_calibration.input_frame_subscriber_init()
            camera_calibration.output_frame_publisher_init()
    elif len(sys.argv)==5:      
        if sys.argv[1] == '-input':
            camera_calibration.input_frame_subscriber_init(rostopic_name=sys.argv[2])
            camera_calibration.output_frame_publisher_init(rostopic_name=sys.argv[4])
        elif sys.argv[1] == '-output':
            camera_calibration.output_frame_publisher_init(rostopic_name=sys.argv[2])
            camera_calibration.input_frame_subscriber_init(rostopic_name=sys.argv[4])
        else:
            camera_calibration.input_frame_subscriber_init()
            camera_calibration.output_frame_publisher_init()

    else:
        camera_calibration.input_frame_subscriber_init()
        camera_calibration.output_frame_publisher_init()
    
    camera_calibration.delta_t_service_init()
    camera_calibration.main_loop()

if __name__ == '__main__':
    try:
        camera_calibration()
    except rospy.ROSInterruptException:
        pass