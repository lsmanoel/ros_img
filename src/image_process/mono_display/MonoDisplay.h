#include "../image_process/ImageProcess.h"

class MonoDisplay: public ImageProcess
{
public:
	MonoDisplay(int argc, char** argv, std::string name_in);

	// ----------------------------------------------------------------------------------------
	// Main Loop
	cv::Mat main_process(cv::Mat frame);
};