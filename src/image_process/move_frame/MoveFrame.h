#include "../image_process/ImageProcess.h"

class MoveFrame: public ImageProcess
{
    // Input Quadilateral or Image plane coordinates
    cv::Point2f inputQuad[4]; 
    // Output Quadilateral or World plane coordinates
    cv::Point2f outputQuad[4];
    // Lambda Matrix
    cv::Mat lambda_matrix;


public:
	MoveFrame(int argc, char** argv, std::string name_in);

	// ----------------------------------------------------------------------------------------
	// Main Loop
	cv::Mat main_process(cv::Mat frame);
};