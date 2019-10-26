#include "CannyFilter.h"

CannyFilter::CannyFilter(int argc, char** argv, std::string name_in, int rate_in)
	:ImageProcess(argc, argv, name_in, rate_in)
{
	input_frame_type = "bgr8";
	output_frame_type = "mono8";
}

// ----------------------------------------------------------------------------------------

void CannyFilter::main_process()
{
	// ROS_INFO("CannyFilter::main_process()");
	input_frame.copyTo(frame);
	// // **************************
	// // PROCESS
	cv::cvtColor(frame, frame, CV_BGR2GRAY);
	cv::Canny(frame, frame, 50, 150, 3);
	// // **************************
	frame.copyTo(output_frame);
}

// # frame_output = np.zeros((frame_input.shape))
// histogram = np.zeros(frame.shape[0])

// for i, line in enumerate(frame[:,]):
//     line_energy = np.sum(line/255)
//     # print(line)
//     histogram[i] = int(line_energy)

// return histogram