#include "../image_process/ImageProcess.h"

class MonoVision: public ImageProcess
{
	int video_source;
	cv::VideoCapture cap;
	int rate;

public:
	MonoVision(int argc, char** argv, std::string name_in, int rate_in);

	void set_video_source(int setter_video_source);
	int get_video_source();
	void init_video_capture();
	
	// ----------------------------------------------------------------------------------------
	// Main Loop
	void main_loop();
	cv::Mat main_process(cv::Mat frame);
};