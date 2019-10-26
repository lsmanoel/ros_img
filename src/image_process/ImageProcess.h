#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <list>
#include <vector> 
#include <algorithm>
#include "ros_img/return_data.h" 
#include <iostream>
#include <std_msgs/Int64.h>
#include "std_msgs/Int64MultiArray.h"
// using namespace std;

class ImageProcess
{
protected:
	std::string name;

	int rate;
	const int delta_t_buffer_size = 1000;

	std_msgs::Int64MultiArray _delta_t;
	// _delta_t.data.clear();

	const int FULL_FRAME_WIDTH = 640;
 	const int FULL_FRAME_HEIGHT = 480;
  	const int VIEW_FRAME_WIDTH = 640;
  	const int VIEW_FRAME_HEIGHT = 480;

	cv::Mat input_frame;
	int frame_flag = 0;
	cv::Mat output_frame;
	cv::Mat frame;
	std::string input_frame_type, output_frame_type;

  	ros::NodeHandle nh;
 	image_transport::ImageTransport it;
	image_transport::Publisher pub_output;
	image_transport::Subscriber sub_input;
	sensor_msgs::ImagePtr msg;
	ros::ServiceServer _delta_t_service;



public:
	ImageProcess(int argc, char** argv, std::string name_in, int rate_in);

	// ----------------------------------------------------------------------------------------
	// rostopics
	void output_frame_publisher_init(std::string rostopic_name/*="None"*/);

	// ----
	void input_frame_callback(const sensor_msgs::ImageConstPtr& msg);
	void input_frame_subscriber_init(std::string rostopic_name/*="None"*/);

	// ----------------------------------------------------------------------------------------
	// rosservices
	bool delta_t_service(ros_img::return_data::Request  &req, ros_img::return_data::Response &res);
	void delta_t_service_init();

	// ----
	int delta_t_getter();
	void delta_t_setter(std_msgs::Int64 d_t);

	// ----------------------------------------------------------------------------------------
	// Main Loop
	void main_loop();
	virtual void main_process();

};



