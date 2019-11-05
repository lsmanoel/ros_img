#include "StereoVision.h"

StereoVision::StereoVision(int argc, char** argv, std::string name_in)
	:ImageProcess(argc, argv, name_in)
{
	input_frame_type = "bgr8";
	output_frame_type = "bgr8";
}

cv::Mat StereoVision::crosshairs(cv::Mat frame, cv::Scalar color)
{
	cv::line(frame, 
		     cv::Point(0, frame.rows/2), 
		     cv::Point(frame.cols, frame.rows/2), 
		     color, 
		     1);
	cv::line(frame, 
		     cv::Point(frame.cols/2, 0), 
		     cv::Point(frame.cols/2, frame.rows), 
		     color, 
		     1);
	return frame;
}

int StereoVision::histogram_max_value(cv::Mat frame)
{
    int maxVal=0;
    int maxIndex=0;
	int hist[256];
	frame.convertTo(frame, CV_8UC1);

	// ROS_INFO("histogram...");
	for(int i=0; i<256; i++){
		hist[i]=0;
	}

	for(int j=0; j<frame.cols; j++){
		for(int i=0; i<frame.rows; i++){
			// ROS_INFO("	:%d", frame.at<uchar>(i, j));
			hist[frame.at<uchar>(i, j)]++;
		}
	}

	for(int i=0; i<256; i++){
		if(hist[i] > maxVal){
			maxIndex = i;
			maxVal = hist[i];
		} 			
	}

	return maxIndex;
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
		pub_L_output = it.advertise(name + "_output_frame_cpp/L", 1);
		pub_R_output = it.advertise(name + "_output_frame_cpp/R", 1);
		pub_depth_output = it.advertise(name + "_output_frame_cpp/depth", 1);
		pub_depth_raw_output = it.advertise(name + "_output_frame_cpp/depth_raw", 1);
		pub_histogram = nh.advertise<std_msgs::UInt8>(name + "_central_depth_histogram_cpp", 1); 
	}
	else
	{	
		pub_L_output = it.advertise(rostopic_name + "/L", 1);
		pub_R_output = it.advertise(rostopic_name + "/R", 1);
		pub_depth_output = it.advertise(rostopic_name + "/depth", 1);
		pub_depth_raw_output = it.advertise(rostopic_name + "/depth_raw", 1);
		pub_histogram = nh.advertise<std_msgs::UInt8>(rostopic_name + "_central_depth_histogram_cpp", 1); 
	}	
}

// ----------------------------------------------------------------------------------------
// Main Loop
void StereoVision::main_loop()
{
	cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(32, 25);

	cv::Mat frame_L, frame_R, frame_stereo;
	cv::Mat frame_L_acc, frame_R_acc, frame_stereo_acc;
	cv::Mat frame_stereo_raw;
	int histogram;

	frame_L_acc = cv::Mat::zeros(480, 640, CV_64FC1);
	frame_R_acc = cv::Mat::zeros(480, 640, CV_64FC1);
	frame_stereo_acc = cv::Mat::zeros(480, 640, CV_64FC1);

	// Rotation Matrix
	cv::Point2f src_center(FULL_FRAME_WIDTH/2.0F, FULL_FRAME_HEIGHT/2.0F);

	cv::Mat M_rot_L = cv::getRotationMatrix2D(src_center, 270, 1.0);
	cv::Mat M_rot_R = cv::getRotationMatrix2D(src_center, 90, 1.0);
	//ROI
	cv::Rect histogram_size(FULL_FRAME_WIDTH/2 - 5, 
			                FULL_FRAME_HEIGHT/2 - 50, 
			                10,
			                100);

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

		cv::GaussianBlur(frame_L, frame_L, cv::Size(3, 3), 0);
		cv::GaussianBlur(frame_R, frame_R, cv::Size(3, 3), 0);
      	cv::warpAffine(frame_L, frame_L, M_rot_L, frame_L.size());
      	cv::warpAffine(frame_R, frame_R, M_rot_R, frame_R.size());

      	cv::accumulateWeighted(frame_L, frame_L_acc, 0.33);
		cv::accumulateWeighted(frame_R, frame_R_acc, 0.33);
		frame_L = frame_L_acc.clone();
		frame_R = frame_R_acc.clone();

		frame_L.convertTo(frame_L, CV_8UC1);
		frame_R.convertTo(frame_R, CV_8UC1);

		stereo->compute(frame_L, frame_R, frame_stereo);
		cv::convertScaleAbs(frame_stereo, frame_stereo);
		
		cv::GaussianBlur(frame_stereo, frame_stereo, cv::Size(3, 3), 0);
		
		frame_stereo.convertTo(frame_stereo, CV_32FC1);
		cv::accumulateWeighted(frame_stereo, frame_stereo_acc, 0.1);
		frame_stereo = frame_stereo_acc.clone();
		frame_stereo.convertTo(frame_stereo, CV_8UC1);
		histogram = StereoVision::histogram_max_value(frame_stereo(histogram_size));
		ROS_INFO("histogram: '%d'", histogram);
		cv::cvtColor(frame_stereo, frame_stereo, CV_GRAY2BGR);
		frame_stereo_raw = frame_stereo.clone();

		cv::cvtColor(frame_L, frame_L, CV_GRAY2BGR);
		cv::cvtColor(frame_R, frame_R, CV_GRAY2BGR);

		// ----------------------------------------------------------------------------------------

		// ----------------------------------------------------------------------------------------
		cv::putText(frame_stereo, //target image
		            std::to_string(histogram), //text
		            cv::Point(FULL_FRAME_HEIGHT/2+120, FULL_FRAME_WIDTH/2+130), //top-left position
		            cv::FONT_HERSHEY_SIMPLEX,
		            3.0,
		            CV_RGB(255, 0, 0), //font color
		            2);		

		frame_R = StereoVision::crosshairs(frame_R, cv::Scalar(0, 0, 255));
		frame_L = StereoVision::crosshairs(frame_L, cv::Scalar(0, 0, 255));
		frame_stereo = StereoVision::crosshairs(frame_stereo, cv::Scalar(0, 0, 255));

        cv::rectangle(frame_stereo, 
			          histogram_size, 
			          cv::Scalar(histogram, histogram, histogram),
			          CV_FILLED, 
			          1);

        cv::rectangle(frame_stereo, 
			          histogram_size, 
			          cv::Scalar(0, 0, 255), 
			          1);

		if(!frame_L.empty())
			pub_L_output.publish(cv_bridge::CvImage(std_msgs::Header(), output_frame_type.c_str(), frame_L).toImageMsg());
		if(!frame_R.empty())
			pub_R_output.publish(cv_bridge::CvImage(std_msgs::Header(), output_frame_type.c_str(), frame_R).toImageMsg());
		pub_depth_output.publish(cv_bridge::CvImage(std_msgs::Header(), output_frame_type.c_str(), frame_stereo).toImageMsg());
		pub_depth_raw_output.publish(cv_bridge::CvImage(std_msgs::Header(), output_frame_type.c_str(), frame_stereo_raw).toImageMsg());
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
