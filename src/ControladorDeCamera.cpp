#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/String.h"
#include <sstream>

using namespace cv;

int main(int, char**)
{
    ROS_INFO("ControladordeCamera start... ");
    
    VideoCapture video_capture(0); // open the default camera
    if(!video_capture.isOpened()){  // check if we succeeded
        ROS_INFO("erro: VideoCapture()");
        return -1;
    }

    Mat input_frame, output_frame, ping_frame, pong_frame;
    namedWindow("edges",1);
    ROS_INFO("start loop..."); 
    while(true)
    {
        video_capture >> input_frame; // get a new frame from camera
        cvtColor(input_frame, ping_frame, COLOR_BGR2GRAY);
        GaussianBlur(ping_frame, pong_frame, Size(7,7), 1.5, 1.5);
        Canny(pong_frame, output_frame, 0, 30, 3);

        imshow("gray image", output_frame);
        if(waitKey(30) == 27){
            ROS_INFO("break loop..."); 
            break;
        }
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    ROS_INFO("END PROGRAM!!!"); 
    return 0;
}