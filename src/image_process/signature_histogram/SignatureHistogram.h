#include "../image_process/ImageProcess.h"

class SignatureHistogram: public ImageProcess
{
	std::string mode;
	std::vector<int> h_histogram, v_histogram;
public:
	SignatureHistogram(int argc, char** argv, std::string name_in);
	cv::Mat horizontal_edge_signature(cv::Mat frame);
	cv::Mat vertical_edge_signature(cv::Mat frame);	
	cv::Mat horizontal_histogram(cv::Mat frame);
	cv::Mat vertical_histogram(cv::Mat frame);

	// ----------------------------------------------------------------------------------------
	// Main Loop
	cv::Mat main_process(cv::Mat frame);
};