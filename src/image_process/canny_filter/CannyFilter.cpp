#include "CannyFilter.h"

CannyFilter::CannyFilter(int argc, char** argv, std::string name_in, int rate_in)
	:ImageProcess(argc, argv, name_in, rate_in)
{
	input_frame_type = "bgr8";
	output_frame_type = "mono8";
}

// ----------------------------------------------------------------------------------------
cv::Mat  CannyFilter::main_process(cv::Mat frame)
{
	// ROS_INFO("CannyFilter::main_process()");
	// // **************************
	// // PROCESS
	cv::cvtColor(frame, frame, CV_BGR2GRAY);
	cv::Canny(frame, frame, 50, 150, 3);
	// // **************************
	return frame;
}