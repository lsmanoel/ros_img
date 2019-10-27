#include "MonoVision.h"

MonoVision::MonoVision(int argc, char** argv, std::string name_in, int rate_in)
	:ImageProcess(argc, argv, name_in, rate_in)
{

}

// ----------------------------------------------------------------------------------------
cv::Mat MonoVision::main_process(cv::Mat frame)
{
	// ROS_INFO("MonoDisplay::main_process()");
	// // **************************
	// // PROCESS
	// // **************************
	return frame;
}
