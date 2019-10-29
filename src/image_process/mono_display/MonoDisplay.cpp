#include "MonoDisplay.h"

MonoDisplay::MonoDisplay(int argc, char** argv, std::string name_in)
	:ImageProcess(argc, argv, name_in)
{
	name = name_in;
}

// ----------------------------------------------------------------------------------------
cv::Mat MonoDisplay::main_process(cv::Mat frame)
{
	// ROS_INFO("MonoDisplay::main_process()");
	// // **************************
	// // PROCESS
	cv::imshow(name, frame);
    cv::waitKey(30);
	// // **************************
	return frame;
}