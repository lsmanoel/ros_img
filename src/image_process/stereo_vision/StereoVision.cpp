#include "StereoVision.h"

StereoVision::StereoVision(int argc, char** argv, std::string name_in, int rate_in)
	:ImageProcess(argc, argv, name_in, rate_in)
{
	input_frame_type = "bgr8";
	output_frame_type = "mono8";
}

// ----------------------------------------------------------------------------------------
cv::Mat  StereoVision::main_process(cv::Mat frame)
{
	// ROS_INFO("SignatureHistogram::main_process()");
	// **************************
	// PROCESS
	// **************************
	return frame;
}
