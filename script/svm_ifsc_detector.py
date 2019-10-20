#!/usr/bin/env python
import os
import sys
import roslib
import rospy
import dlib
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_process import ImageProcess

class SvmIfscDetector(ImageProcess):
    def __init__(self,
                 name=None,
                 input_frame=None,
                 fps=30,
                 delta_t_buffer_size=1000,
                 dataset_folder_path ='/home/lucas/catkin_ws/src/ros_img/script/ifsc_logo_dataset',
                 training_xml='ifsc_logo.xml',
                 detector_svm='detector.svm',
                 landmarks_dat='landmarks.dat'):

        if name is None:
            name='svm_ifsc_detector'

        super(SvmIfscDetector, self).__init__(name=name,
                                          input_frame=input_frame,
                                          fps=fps,
                                          delta_t_buffer_size=delta_t_buffer_size)

        self.dataset_folder_path = dataset_folder_path
        self.training_xml = os.path.join(dataset_folder_path, training_xml)
        self.detector_svm = os.path.join(dataset_folder_path, detector_svm)
        self.landmarks_dat = os.path.join(dataset_folder_path, landmarks_dat)

        self.detector = dlib.fhog_object_detector(self.detector_svm)
        self.landmarks_detector = dlib.shape_predictor(self.landmarks_dat)
        self.loc = Point32()

        print(cv2.__version__)

    # ----------------------------------------------------------------------------------------
    def trainer(self,
                add_left_right_image_flips=False,
                C=5,
                num_threads=2,
                be_verbose=True):

        #===============================================================================
        print("svn_options ...")
        svm_training_options = dlib.simple_object_detector_training_options()
        #-------------------------------------------------------------------------------
        # During training stage: This option flip the input image. 
        # This helps it get the most value out of the training data.
        svm_training_options.add_left_right_image_flips = add_left_right_image_flips
        #-------------------------------------------------------------------------------
        # The trainer is a kind of support vector machine and therefore has the usual
        # SVM C parameter.  In general, a bigger C encourages it to fit the training
        # data better but might lead to overfitting.  You must find the best C value
        # empirically by checking how well the trained detector works on a test set of
        # images you haven't trained on.  Don't just leave the value set at 5.  Try a
        # few different C values and see what works best for your data.
        svm_training_options.C = C
        #-------------------------------------------------------------------------------
        # Set how many CPU cores your computer has for the fastest training.
        svm_training_options.num_threads = num_threads
        #-------------------------------------------------------------------------------
        # Verbose Mode
        svm_training_options.be_verbose = be_verbose

        #===============================================================================
        print("training svm ...")
        dlib.train_simple_object_detector(self.training_xml, 
                                          self.detector_svm, 
                                          svm_training_options)
        print("ok!")

        #===============================================================================
        print("training landmarks ...")
        dlib.train_shape_predictor(self.training_xml, 
                                   self.landmarks_dat, 
                                   dlib.shape_predictor_training_options())
        print("ok!")

    # ----------------------------------------------------------------------------------------
    @staticmethod
    def printLandmark(image, landmarks, color):    
        for p in landmarks.parts():
            cv2.circle(image, (p.x, p.y), 20, color, 2)
        return image

    # ----------------------------------------------------------------------------------------
    # Main Loop   
    def main_process(self):
        frame = self.input_frame.copy()
        # -----------------------------------
        [boxes, confidences, detector_idxs]  = dlib.fhog_object_detector.run(self.detector, 
                                                                             frame, 
                                                                             upsample_num_times=0, 
                                                                             adjust_threshold=0.1) 
        for i, box in enumerate(boxes):
            e, t, d, b = (int(box.left()), 
                          int(box.top()), 
                          int(box.right()), 
                          int(box.bottom()))
            self.loc.x = (e+d)/2
            self.loc.y = (t+b)/2
            self.loc.z = i
            self.pub_output_loc.publish(self.loc)

            cv2.rectangle(frame, (e, t), (d, b), (0, 0, 255), 2)

            landmark = self.landmarks_detector(frame, box)
            self.printLandmark(frame, landmark, (255, 0, 0))

        # -----------------------------------
        self.pub_output.publish(self.bridge.cv2_to_imgmsg(frame))

# ======================================================================================================================
def svm_ifsc_detector():
    svm_ifsc_detector = SvmIfscDetector()

    if len(sys.argv)==2:
        if sys.argv[1] == 'trainer_mode':
            svm_ifsc_detector.trainer()
    else:      
        if len(sys.argv)>=3:      
            if sys.argv[1] == '-input_frame':
                svm_ifsc_detector.signals_subscriber_init(rostopic_name=sys.argv[2])
            elif sys.argv[1] == '-output_frame':
                svm_ifsc_detector.signals_publisher_init(rostopic_name=sys.argv[2])
        else:
            svm_ifsc_detector.signals_subscriber_init()

        if len(sys.argv)>=5:      
            if sys.argv[3] == '-input_frame':
                svm_ifsc_detector.signals_subscriber_init(rostopic_name=sys.argv[4])
            elif sys.argv[3] == '-output_frame':
                svm_ifsc_detector.signals_publisher_init(rostopic_name=sys.argv[4])
        else:
            svm_ifsc_detector.signals_publisher_init()

        svm_ifsc_detector.delta_t_service_init()
        svm_ifsc_detector.main_loop()

if __name__ == '__main__':
    try:
        svm_ifsc_detector()
    except rospy.ROSInterruptException:
        pass	