#include "StereoVision.h"

StereoVision::StereoVision(int argc, char** argv, std::string name_in)
	:ImageProcess(argc, argv, name_in)
{
	input_frame_type = "bgr8";
	output_frame_type = "bgr8";

	stereo = cv::StereoMatcher::create(32, 25)
}

void StereoVision::set_video_L_source(int setter_video_source)
{
	video_L_source = setter_video_source;
	cap_L = cv::VideoCapture(video_L_source);
	if(!cap_L.isOpened()) 
		return;
	cap_L.set(cv::CAP_PROP_FRAME_WIDTH, FULL_FRAME_WIDTH);
  	cap_L.set(cv::CAP_PROP_FRAME_HEIGHT, FULL_FRAME_HEIGHT);
}

int StereoVision::get_video_L_source()
{
	return video_L_source;
}

void StereoVision::set_video_R_source(int setter_video_source)
{
	video_R_source = setter_video_source;
	cap_R = cv::VideoCapture(video_R_source);
	if(!cap_R.isOpened()) 
		return;
  	cap_R.set(cv::CAP_PROP_FRAME_WIDTH, FULL_FRAME_WIDTH);
  	cap_R.set(cv::CAP_PROP_FRAME_HEIGHT, FULL_FRAME_HEIGHT);
}

int StereoVision::get_video_R_source()
{
	return video_R_source;
}

// ----------------------------------------------------------------------------------------
// rostopics
void StereoVision::output_frame_publisher_init(std::string rostopic_name/*="None"*/)
{
	if(rostopic_name.compare("None")==0)
	{
		pub_L_output = it.advertise(name + "_output_frame_cpp/L", 10) 
		pub_R_output = it.advertise(name + "_output_frame_cpp/R", 10) 
		pub_depth_output = it.advertise(name + "_output_frame_cpp/depth", 10) 
		pub_depth_raw_output = it.advertise(name + "_output_frame_cpp/depth_raw", 10) 
		pub_histogram = nh.advertise<std_msgs::UInt8>(name + "_central_depth_histogram_cpp", 10); 
	}
	else
	{	
		pub_L_output = it.advertise(rostopic_name + "/L", 10) 
		pub_R_output = it.advertise(rostopic_name + "/R", 10) 
		pub_depth_output = it.advertise(rostopic_name + "/depth", 10) 
		pub_depth_raw_output = it.advertise(rostopic_name + "/depth_raw", 10) 
		pub_histogram = nh.advertise<std_msgs::UInt8>(rostopic_name + "_central_depth_histogram_cpp", 10); 
	}	
}

// ----------------------------------------------------------------------------------------
// Main Loop
void StereoVision::main_loop()
{
	cv::Mat frame_L, frame_R, frame_stereo;
	cv::Mat frame_L_acc, frame_R_acc, frame_stereo_acc;
	cv::Mat frame_stereo_raw;

	// Rotation Matrix
	cv::Point2f src_center(FULL_FRAME_WIDTH/2.0F, FULL_FRAME_HEIGHT/2.0F);

	cv::Mat M_rot_L = cv::getRotationMatrix2D(src_center, 270, 1.0);
	cv::Mat M_rot_R = cv::getRotationMatrix2D(src_center, 90, 1.0);
	//ROI
	cv::Rect roi_size(FULL_FRAME_WIDTH/2 - VIEW_FRAME_WIDTH/2, 
		              FULL_FRAME_HEIGHT/2 - VIEW_FRAME_HEIGHT/2, 
		              VIEW_FRAME_WIDTH,
		              VIEW_FRAME_HEIGHT);

	ros::Rate loop_rate(rate);
	// --------------------------
	while (nh.ok())
	{
		loop_rate.sleep();
		t0.data = ros::Time::now().toNSec();
		// ************************** 
		// PROCESS
		cap_L >> frame_L;
		cap_R >> frame_R;

		cv::cvtColor(frame_L, frame_L, CV_BGR2GRAY);
		cv::cvtColor(frame_R, frame_R, CV_BGR2GRAY);

      	cv::warpAffine(frame_L, frame_L, M_rot_L, frame_L.size());
      	cv::warpAffine(frame_R, frame_R, M_rot_R, frame_R.size());

		cv::GaussianBlur(frame_L, frame_L, cv::Size(3, 3), 0);
		cv::GaussianBlur(frame_R, frame_R, cv::Size(3, 3), 0);

		cv::convertScaleAbs(frame_L, frame_L);

		stereo.compute(frame_L, frame_R, frame_stereo)

		cv::convertScaleAbs(frame_stereo, frame_stereo);
		cv::GaussianBlur(frame_stereo, frame_stereo, cv::Size(3, 3), 0);

		frame_stereo_raw = frame_stereo.clone();

		pub_L_output.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_L).toImageMsg());
		pub_R_output.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_L).toImageMsg());
		pub_depth_output.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_stereo).toImageMsg());
		pub_depth_raw_output.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_stereo_raw).toImageMsg());
		// pub_histogram.publish();
		
		// ************************** 
		t.data = ros::Time::now().toNSec();

		d_t.data = t.data - t0.data;
		if (d_t.data > 0)
			delta_t_setter(d_t);

		ros::spinOnce();
	}
	// -------------------------- 		
}
