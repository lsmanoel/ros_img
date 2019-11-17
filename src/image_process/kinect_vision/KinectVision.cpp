#include "KinectVision.h"

KinectVision::KinectVision(int argc, char** argv, std::string name_in)
	:ImageProcess(argc, argv, name_in)
{
	input_frame_type = "bgr8";
	output_frame_type = "bgr8";
}

cv::Mat KinectVision::crosshairs(cv::Mat frame, cv::Scalar color)
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

int KinectVision::histogram_max_value(cv::Mat frame)
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

// ----------------------------------------------------------------------------------------
// rostopics
void KinectVision::output_frame_publisher_init(std::string rostopic_name/*="None"*/)
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
// void KinectVision::main_loop()
// {
// 	freenect_sync_get_rgb_cv(0);
// 	freenect_sync_get_depth_cv(0);
// }

void KinectVision::main_loop()
{
	static std::vector<uint8_t> rgb(640*480*3);
    static std::vector<uint16_t> depth(640*480);

	device = &freenect.createDevice<MyFreenectDevice>(0);
    device->startVideo();
    device->startDepth();

    cv::Mat frame_stereo = cv::Mat::zeros(480, 640, CV_16U);

	cv::Mat frame_L;
	cv::Mat frame_stereo_raw;
	int histogram;

	// Rotation Matrix
	cv::Point2f src_center(FULL_FRAME_WIDTH/2.0F, FULL_FRAME_HEIGHT/2.0F);

	cv::Mat M_rot_L = cv::getRotationMatrix2D(src_center, 270, 1.0);
	cv::Mat M_rot_R = cv::getRotationMatrix2D(src_center, 90, 1.0);
	//ROI
	cv::Rect histogram_size(FULL_FRAME_WIDTH/2 - 5, 
			                FULL_FRAME_HEIGHT/2 - 50, 
			                10,
			                100);

	ros::Rate loop_rate(30);
	// --------------------------
	while (nh.ok())
	{
		loop_rate.sleep();
		t0.data = ros::Time::now().toNSec();
		// ************************** 
		// PROCESS
		device->getRGB(rgb);
		device->getDepth(depth);

		ROS_INFO("depth size: %d", depth.size());
		ROS_INFO("frame_stereo: %d", frame_stereo.cols*frame_stereo.rows);

		// for(int i=0; i<frame_stereo.rows; ++i){
		// 	for(int j=0; j<frame_stereo.cols; ++j){
		// 		depth.at(i*j) = 255*255;
		// 	}
		// }

		for(int j=0; j<frame_stereo.cols; ++j){
			for(int i=0; i<frame_stereo.rows; ++i){
				frame_stereo.at<uint16_t>(i, j) = 255*depth.at(i*j);
				// ROS_INFO("depth.at(i*j): %d", depth.at(i*j));
				// ROS_INFO("frame_stereo.at<uint16_t>(i, j): %d", frame_stereo.depth());
			}
		}
		cv::imshow("frame_stereo", frame_stereo);
    	cv::waitKey(30);

		// frame_stereo.convertTo(frame_stereo, CV_8UC1);
		// histogram = KinectVision::histogram_max_value(frame_stereo(histogram_size));
		// // ROS_INFO("histogram: '%d'", histogram);
		// cv::cvtColor(frame_stereo, frame_stereo, CV_GRAY2BGR);
		// frame_stereo_raw = frame_stereo.clone();

		// cv::cvtColor(frame_L, frame_L, CV_GRAY2BGR);

		// // ----------------------------------------------------------------------------------------
		// // ----------------------------------------------------------------------------------------
		// cv::putText(frame_stereo, //target image
		//             std::to_string(histogram), //text
		//             cv::Point(FULL_FRAME_HEIGHT/2+120, FULL_FRAME_WIDTH/2+130), //top-left position
		//             cv::FONT_HERSHEY_SIMPLEX,
		//             3.0,
		//             CV_RGB(255, 0, 0), //font color
		//             2);		

		// frame_L = KinectVision::crosshairs(frame_L, cv::Scalar(0, 0, 255));
		// frame_stereo = KinectVision::crosshairs(frame_stereo, cv::Scalar(0, 0, 255));

  //       cv::rectangle(frame_stereo, 
		// 	          histogram_size, 
		// 	          cv::Scalar(histogram, histogram, histogram),
		// 	          CV_FILLED, 
		// 	          1);

  //       cv::rectangle(frame_stereo, 
		// 	          histogram_size, 
		// 	          cv::Scalar(0, 0, 255), 
		// 	          1);

		// if(!frame_L.empty())
		// 	pub_L_output.publish(cv_bridge::CvImage(std_msgs::Header(), output_frame_type.c_str(), frame_L).toImageMsg());
		// pub_depth_output.publish(cv_bridge::CvImage(std_msgs::Header(), output_frame_type.c_str(), frame_stereo).toImageMsg());
		// pub_depth_raw_output.publish(cv_bridge::CvImage(std_msgs::Header(), output_frame_type.c_str(), frame_stereo_raw).toImageMsg());
		// // pub_histogram.publish();
		
		// ************************** 
		t.data = ros::Time::now().toNSec();

		d_t.data = t.data - t0.data;
		if (d_t.data > 0)
			delta_t_setter(d_t);

		ros::spinOnce();
	}
	// -------------------------- 		
}
