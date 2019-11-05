#include "../image_process/ImageProcess.h"

class StereoVision: public ImageProcess
{
	int video_L_source, video_R_source;
	cv::VideoCapture cap_L, cap_R;
	int rate;

	image_transport::Publisher pub_L_output;
	image_transport::Publisher pub_R_output;
	image_transport::Publisher pub_depth_output;
	image_transport::Publisher pub_depth_raw_output;
	ros::Publisher pub_histogram;

public:
	StereoVision(int argc, char** argv, std::string name_in);

	cv::Mat crosshairs(cv::Mat frama, cv::Scalar color);
	int histogram_max_value(cv::Mat frame);
	// ----------------------------------------------------------------------------------------
	// rostopics
	void output_frame_publisher_init(std::string rostopic_name/*="None"*/);

	void set_video_L_source(int setter_video_source);
	void set_video_R_source(int setter_video_source);
	int get_video_L_source();
	int get_video_R_source();

	void init_video_capture();

	// ----------------------------------------------------------------------------------------
	// Main Loop
	void main_loop();
};