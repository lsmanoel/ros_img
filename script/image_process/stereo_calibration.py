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
from stereo_vision import StereoVision


class StereoCalibration(StereoVision):
    def __init__(self, 
                 name=None):

        if name is None:
            name='stereo_calibration'

        super(StereoCalibration, self).__init__(name=name)

    # ----------------------------------------------------------------------------------------
    def main_machine_state(self):
        print(self.main_state)

        if self.main_state == 'start_state':
            self.main_state = 'find_chess_state'
            self.start_L_state()
            self.start_R_state()

        elif self.main_state == 'find_chess_L_state':
            ret, frame = self.find_chess_state()
            if ret is True:
                self.main_state = 'process_coefficients_L_state' 

        elif self.main_state == 'process_coefficients_L_state':
            self.main_state = 'find_chess_R_state'
            self.process_coefficients_state()

        elif self.main_state == 'find_chess_R_state':
            ret, frame = self.find_chess_state()
            if ret is True:
                self.main_state = 'process_coefficients_R_state' 

        elif self.main_state == 'process_coefficients_R_state':
            self.main_state = 'main_loop'
            self.process_coefficients_state()

        elif self.main_state == 'main_loop':
            self.get_depth()

        return frame

    def start_L_state(self, chess_size=(7, 6)):
        # termination criteria
        self.criteria_L = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp_L = np.zeros((chess_size[0]*chess_size[1], 3), np.float32)
        self.objp_L[:, :2] = np.mgrid[0:chess_size[0], 0:chess_size[1]].T.reshape(-1, 2)
        # Arrays to store object points and image points from all the images.
        self.objpoints_L = [] # 3d point in real world space
        self.imgpoints_L = [] # 2d points in image plane.

    def start_R_state(self, chess_size=(7, 6)):
        # termination criteria
        self.criteria_R = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp_R = np.zeros((chess_size[0]*chess_size[1], 3), np.float32)
        self.objp_R[:, :2] = np.mgrid[0:chess_size[0], 0:chess_size[1]].T.reshape(-1, 2)
        # Arrays to store object points and image points from all the images.
        self.objpoints_R = [] # 3d point in real world space
        self.imgpoints_R = [] # 2d points in image plane.

    def find_chess_L_state(self):
        ret, frame_L = self.video_capture_L.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(frame, (7, 6), None)
        # If found, add object points, image points (after refining them)
        if ret is True:
            self.ret_L = ret
            self.objpoints_L.append(self.objp_L)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria_L)
            self.imgpoints_L.append(corners)
            # Draw and display the corners
            cv2.drawChessboardCorners(frame, (7, 6), corners, ret)

        return ret, frame

    def process_coefficients_L_state(self):
        self.ret_L, self.mtx_L, self.dist_L, self.rvecs_L, self.tvecs_L = cv2.calibrateCamera(self.objpoints_L, 
                                                                                              self.imgpoints_L, 
                                                                                              frame.shape[:-1], 
                                                                                              None, 
                                                                                              None)
        h,  w = frame.shape[:2]
        self.newcameramtx_L, self.roi_L = cv2.getOptimalNewCameraMatrix(self.mtx_L, self.dist_L, (w, h), 1, (w, h))

        tot_error = 0
        mean_error = 0
        for i in range(len(self.objpoints_L)):
            imgpoints2, _ = cv2.projectPoints(self.objpoints_L[i], self.rvecs_L[i], self.tvecs_L[i], self.mtx_L, self.dist_L)
            error = cv2.norm(self.imgpoints_L[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            tot_error += error

        print("total error: ", mean_error/len(self.objpoints_L))

    def undistort_L_state(self, frame):
        # undistort
        frame = cv2.undistort(frame, self.mtx_L, self.dist_L, None, self.newcameramtx_L)
        # crop the image
        x, y, w, h = self.roi_L
        frame = frame[y:y+h, x:x+w]

        print(self.dist_L)
        return frame  

    def find_chess_R_state(self):
        ret, frame_R = self.video_capture_R.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(frame, (7, 6), None)
        # If found, add object points, image points (after refining them)
        if ret is True:
            self.ret_R = ret
            self.objpoints_R.append(self.objp_R)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria_R)
            self.imgpoints_R.append(corners)
            # Draw and display the corners
            cv2.drawChessboardCorners(frame, (7, 6), corners, ret)

        return ret, frame

    def process_coefficients_R_state(self):
        self.ret_R, self.mtx_R, self.dist_R, self.rvecs_R, self.tvecs_R = cv2.calibrateCamera(self.objpoints_R, 
                                                                                              self.imgpoints_R, 
                                                                                              frame.shape[:-1], 
                                                                                              None, 
                                                                                              None)
        h,  w = frame.shape[:2]
        self.newcameramtx_R, self.roi_R = cv2.getOptimalNewCameraMatrix(self.mtx_R, self.dist_R, (w, h), 1, (w, h))

        tot_error = 0
        mean_error = 0
        for i in range(len(self.objpoints_R)):
            imgpoints2, _ = cv2.projectPoints(self.objpoints_R[i], self.rvecs_R[i], self.tvecs_R[i], self.mtx_R, self.dist_R)
            error = cv2.norm(self.imgpoints_R[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            tot_error += error

        print("total error: ", mean_error/len(self.objpoints_R))

    def undistort_R_state(self, frame):
        # undistort
        frame = cv2.undistort(frame, self.mtx_R, self.dist_R, None, self.newcameramtx_R)
        # crop the image
        x, y, w, h = self.roi_R
        frame = frame[y:y+h, x:x+w]

        print(self.dist_R)
        return frame  

    # ----------------------------------------------------------------------------------------
    # Main Loop
    def main_loop(self):
        self.main_process()

    def main_process(self):
        # **************************
        self.main_machine_state()
        # **************************
        return frame    

    # ----------------------------------------------------------------------------------------
    # Main Loop 
    def get_depth(self):
        print("StereoCalibration.main_loop()")

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
            print("StereoCalibration.process_bulk()")
            t0 = rospy.get_rostime().nsecs

            # **************************
            # PROCESS
            print("StereoCalibration.main_process()")
            # if not (self.video_capture_L.grab() and self.video_capture_R.grab()):
            #     print("No more frames")
            #     break
            # ret, frame_L = self.video_capture_L.retrieve()
            # ret, frame_R = self.video_capture_R.retrieve()

            ret, frame_L = self.video_capture_L.read()
            ret, frame_R = self.video_capture_R.read()

            frame_L = undistort_L_state(frame_L)
            frame_R = undistort_R_state(frame_R)

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
def stereo_calibration():
    stereo_calibration = StereoCalibration()

    if len(sys.argv)==4:
        if sys.argv[1] == '-input':
            stereo_calibration.video_source_L = sys.argv[2]
            stereo_calibration.video_source_R = sys.argv[3]                                                    
            stereo_calibration.output_frame_publisher_init()
            print(1)

        elif sys.argv[1] == '-output':
            stereo_calibration.output_frame_publisher_init(rostopic_name=sys.argv[2])
            print(2)

        else:
            stereo_calibration.output_frame_publisher_init()
            print(3)

    elif len(sys.argv)==6:      
        if sys.argv[1] == '-input':
            stereo_calibration.video_source_L = sys.argv[2]
            stereo_calibration.video_source_R = sys.argv[3]  
            stereo_calibration.output_frame_publisher_init(rostopic_name=sys.argv[5])
            print(4)

        elif sys.argv[1] == '-output':
            stereo_calibration.output_frame_publisher_init(rostopic_name=sys.argv[2])
            stereo_calibration.video_source_L = sys.argv[4]
            stereo_calibration.video_source_R = sys.argv[5]
            print(5)  

        else:
            stereo_calibration.output_frame_publisher_init()
            print(6)

    else:
        stereo_calibration.output_frame_publisher_init()
        print(7)
    


    stereo_calibration.delta_t_service_init()
    stereo_calibration.main_loop()

if __name__ == '__main__':
    try:
        stereo_calibration()
    except rospy.ROSInterruptException:
        pass