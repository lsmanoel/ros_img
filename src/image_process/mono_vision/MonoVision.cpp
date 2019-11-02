#include "MonoVision.h"

MonoVision::MonoVision(int argc, char** argv, std::string name_in, int rate_in)
	:ImageProcess(argc, argv, name_in)
{
	rate = rate_in;

	output_frame_type = "bgr8";
	pub_output_flag == true;
}

void MonoVision::set_video_source(int setter_video_source)
{
	video_source = setter_video_source;
}

int MonoVision::get_video_source()
{
	return video_source;
}

void MonoVision::init_video_capture()
{
	cap = cv::VideoCapture(video_source);

	// Check if video device can be opened with the given index
	if(!cap.isOpened()) 
	  return;

	// Configure the camera resolution
  	cap.set(cv::CAP_PROP_FRAME_WIDTH, FULL_FRAME_WIDTH);
  	cap.set(cv::CAP_PROP_FRAME_HEIGHT, FULL_FRAME_HEIGHT);
}

// ----------------------------------------------------------------------------------------
// Main Loop
void MonoVision::main_loop()
{
	ros::Rate loop_rate(rate);
	// --------------------------
	while (nh.ok())
	{
		loop_rate.sleep();
		process_bulk();
		ros::spinOnce();
	}
	// -------------------------- 		
}

cv::Mat MonoVision::main_process(cv::Mat frame)
{
	// ROS_INFO("MonoVision::main_process()");
	// **************************
	// PROCESS
	cap >> frame;
	// **************************
	return frame;
}
