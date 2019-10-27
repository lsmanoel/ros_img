#include "../image_process/ImageProcess.h"

class CannyFilter: public ImageProcess
{
public:
	CannyFilter(int argc, char** argv, std::string name_in, int rate_in);
	cv::Mat signature_histogram_generation(cv::Mat frame);

	// ----------------------------------------------------------------------------------------
	// Main Loop
	virtual cv::Mat main_process(cv::Mat frame);
};