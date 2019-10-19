#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{
  ROS_INFO("ip_cam_vision_bgr8 cpp start...");

  const int FULL_FRAME_WIDTH = 640;
  const int FULL_FRAME_HEIGHT = 480;
  const int VIEW_FRAME_WIDTH = 640;
  const int VIEW_FRAME_HEIGHT = 480;

  //=========================================================
  // Check if video source has been passed as a parameter
  if(argv[1] == NULL) 
    return 1;

  cv::VideoCapture cap(argv[1]);

  // Check if video device can be opened with the given index
  if(!cap.isOpened()) 
    return 1;

  // Configure the camera resolution
  cap.set(0, FULL_FRAME_WIDTH);
  cap.set(1, FULL_FRAME_HEIGHT);

  int rot_angle;
  if(argv[2] == NULL) 
    rot_angle = 0;
  else{
    std::istringstream rot_angleCmd(argv[2]);
    if(!(rot_angleCmd >> rot_angle)) 
      return 1;   
  }

  ROS_INFO("Rotate Angle: '%d'", rot_angle);

  //=========================================================
  //Buffer Ping Pong
  cv::Mat input_frame, ping_frame, pong_frame;
  // Rotation Matrix
  cv::Point2f src_center(FULL_FRAME_WIDTH/2.0F, FULL_FRAME_HEIGHT/2.0F);
  cv::Mat rot_mat = cv::getRotationMatrix2D(src_center, rot_angle, 1.0);
  //ROI
  cv::Rect roi_size(FULL_FRAME_WIDTH/2 - VIEW_FRAME_WIDTH/2, 
                    FULL_FRAME_HEIGHT/2 - VIEW_FRAME_HEIGHT/2, 
                    VIEW_FRAME_WIDTH,
                    VIEW_FRAME_HEIGHT);

  //=========================================================
  ros::init(argc, argv, "ip_cam_vision_bgr8_cpp");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("ip_cam_vision_bgr8", 1);

  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(30);
  while (nh.ok()) {
    cap >> input_frame;  
    // Check if grabbed frame is actually full with some content
    if(!input_frame.empty()) {
      //===============================================================================
      cv::warpAffine(input_frame, ping_frame, rot_mat, pong_frame.size());
      pong_frame = ping_frame(roi_size);
      //===============================================================================
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pong_frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}