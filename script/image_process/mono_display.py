#!/usr/bin/env python
import sys
import roslib
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ros_img.srv import return_data, return_dataResponse
from cv_bridge import CvBridge, CvBridgeError
from image_process import ImageProcess

class MonoDisplay(ImageProcess):
    def __init__(self,
                 name=None):

        if name is None:
            name='mono_display'

        super(MonoDisplay, self).__init__(name=name)

    # ----------------------------------------------------------------------------------------
    # Main Loop        
    def main_loop(self):
        rospy.spin()
                
    # ----------------------------------------------------------------------------------------
    # Main Loop        
    def main_process(self, frame):
        # **************************
        cv2.imshow(self.name, frame)
        cv2.waitKey(5)

# ======================================================================================================================
def mono_display():
    mono_display = MonoDisplay()

    if len(sys.argv)==3:      
        if sys.argv[1] == '-input':
            mono_display.input_frame_subscriber_init(rostopic_name=sys.argv[2])
        else:
            mono_display.input_frame_subscriber_init()
    else:
        mono_display.input_frame_subscriber_init()
   
    mono_display.delta_t_service_init()
    mono_display.main_loop()

if __name__ == '__main__':
    try:
        mono_display()
    except rospy.ROSInterruptException:
        pass