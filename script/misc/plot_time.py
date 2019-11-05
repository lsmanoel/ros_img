#!/usr/bin/env python
from __future__ import print_function
import sys
import cv2
import rospy
from ros_img.srv import return_data, return_dataResponse
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt

# ======================================================================================================================
def plot_time():   
    mode = None
    imshow_on = []
    if len(sys.argv)>=2:
        service_name = sys.argv[1]

    rospy.wait_for_service(service_name)
    service_proxy = rospy.ServiceProxy(service_name, return_data)
    _delta_t = service_proxy()

    plt.xlabel('quadro')
    plt.ylabel('tempo de processamento (ns)')
    # plt.ylim((0, 33e6))
    plt.plot(_delta_t.data)
    plt.show()

if __name__ == '__main__':
    try:
        plot_time()
    except rospy.ROSInterruptException:
        pass