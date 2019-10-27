#include "../image_process/ImageProcess.h"

class MonoVision: public ImageProcess
{
public:
	MonoVision(int argc, char** argv, std::string name_in, int rate_in);

	// ----------------------------------------------------------------------------------------
	// Main Loop
	virtual cv::Mat main_process(cv::Mat frame);
};