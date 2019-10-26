#include "ImageProcess.h"

class CannyFilter: public ImageProcess
{
public:
	CannyFilter(int argc, char** argv, std::string name_in, int rate_in);
	cv::Mat signature_histogram_generation(cv::Mat frame);

	// ----------------------------------------------------------------------------------------
	// Main Loop
	void main_process();
};