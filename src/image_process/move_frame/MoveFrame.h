#include "../image_process/ImageProcess.h"

class MoveFrame: public ImageProcess
{
	int rotation_angle;
	// Rotation Matrix
	cv::Point2f src_center;
	cv::Mat rotation_matrix;
	//ROI
	cv::Rect roi_size;
public:
	MoveFrame(int argc, char** argv, std::string name_in, int rate_in);

	// ----------------------------------------------------------------------------------------
	// Main Loop
	virtual cv::Mat main_process(cv::Mat frame);
};