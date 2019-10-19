#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{
    
    ROS_INFO("stereo_vision_mono8_cpp start...");

    const int FULL_FRAME_WIDTH = 640;
    const int FULL_FRAME_HEIGHT = 480;
    const int VIEW_FRAME_WIDTH = 320;
    const int VIEW_FRAME_HEIGHT = 240;

    //=========================================================
    // Check if video source has been passed as a parameter
    if(argv[1] == NULL || argv[2] == NULL){
        ROS_INFO("erro: no video device in argv"); 
        return 1;
    }

    ros::init(argc, argv, "stereo_vision_mono8_cpp");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("stereo_vision_mono8", 1);

    // Convert the passed as command line parameter index for the video device to an integer
    std::istringstream video_L_sourceCmd(argv[1]);
    std::istringstream video_R_sourceCmd(argv[2]);
    int video_L_source, video_R_source;

    cv::Point2f src_center(FULL_FRAME_WIDTH/2.0F, FULL_FRAME_HEIGHT/2.0F);
    cv::Mat rot_mat_180 = cv::getRotationMatrix2D(src_center, 180, 1.0);

    // Check if it is indeed a number
    if(!(video_L_sourceCmd >> video_L_source)) 
        return 1;
    if(!(video_R_sourceCmd >> video_R_source)) 
        return 1;

    cv::VideoCapture video_L_capture(video_L_source);
    // Check if video device can be opened with the given index
    if(!video_L_capture.isOpened()) 
        return 1;

    cv::VideoCapture video_R_capture(video_R_source);
    if(!video_R_capture.isOpened()) 
        return 1;

    video_L_capture.set(0, FULL_FRAME_WIDTH);
    video_L_capture.set(1, FULL_FRAME_HEIGHT);
    video_R_capture.set(0, FULL_FRAME_WIDTH);
    video_R_capture.set(1, FULL_FRAME_HEIGHT);

    cv::Mat input_L_frame, input_R_frame;
    cv::Mat ping_frame, pong_frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(5);

    while (nh.ok()) {
        video_L_capture >> input_L_frame;
        video_R_capture >> input_R_frame;

        // Check if grabbed frame is actually full with some content
        if(!input_L_frame.empty() && !input_R_frame.empty()) {
            cv::warpAffine(input_R_frame, ping_frame, rot_mat_180, input_R_frame.size()); 
            cv::vconcat(ping_frame, input_L_frame, pong_frame);
            cv::cvtColor(pong_frame, ping_frame, CV_BGR2GRAY);
            msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", ping_frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }
        else{
            ROS_INFO("empty input frame");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}