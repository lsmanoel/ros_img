#include "ImageProcess.h"

ImageProcess::ImageProcess(int argc, char** argv, std::string name_in, int rate_in)
  : it(nh)
{
	ROS_INFO("%s init...", name_in.c_str());
	ROS_INFO("opencv version: %s" , CV_VERSION);

	name = name_in;
	rate = rate_in;

	frame_flag = 0;

	input_frame_type = "bgr8";
	output_frame_type = "bgr8";
}

// ----------------------------------------------------------------------------------------
// rostopics
void ImageProcess::output_frame_publisher_init(std::string rostopic_name/*="None"*/)
{
	if(rostopic_name.compare("None")==0)
	{
		pub_output = it.advertise(name + "_output_frame_cpp", 1);
	}
	else
	{	
		pub_output = it.advertise(rostopic_name, 1);
	}	
}

// ----
void ImageProcess::input_frame_callback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		frame_flag = 1;
		input_frame = cv_bridge::toCvShare(msg, input_frame_type.c_str())->image;
		bulk_process();
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to '%s'.", msg->encoding.c_str(), input_frame_type.c_str());
	}
}

void ImageProcess::input_frame_subscriber_init(std::string rostopic_name/*="None"*/)
{
	if(rostopic_name.compare("None")==0)
	{
		sub_input = it.subscribe(name + "_input_frame_cpp", 1, &ImageProcess::input_frame_callback, this);
	}
	else
	{
		sub_input = it.subscribe(rostopic_name, 1, &ImageProcess::input_frame_callback, this);
	}
}

// ----------------------------------------------------------------------------------------
// rosservices
bool ImageProcess::delta_t_service(ros_img::return_data::Request  &req, ros_img::return_data::Response &res)
{
	res.data = _delta_t.data;
}

void ImageProcess::delta_t_service_init()
{
	_delta_t_service = nh.advertiseService(name + "_delta_t_cpp_service", &ImageProcess::delta_t_service, this);
}

// ----
int ImageProcess::delta_t_getter()
{
	return _delta_t.data.back(); 
}

void ImageProcess::delta_t_setter(std_msgs::Int64 d_t)
{
	// ROS_INFO("dt: %d", d_t);
	_delta_t.data.push_back(d_t.data);
	if(_delta_t.data.size() > delta_t_buffer_size)
		_delta_t.data.pop_back();
}

// ----------------------------------------------------------------------------------------
// Main Loop
void ImageProcess::main_loop()
{
	ros::spin();
}

void ImageProcess::bulk_process()
{
	t0.data = ros::Time::now().toNSec();
	// **************************	
	output_frame = main_process(input_frame);
	// **************************
	t.data = ros::Time::now().toNSec();

	// --------------------------
	msg = cv_bridge::CvImage(std_msgs::Header(), output_frame_type, output_frame).toImageMsg();
	pub_output.publish(msg);	
	d_t.data = t.data - t0.data;
	if (d_t.data > 0)
		delta_t_setter(d_t);
}

// void ImageProcess::main_loop()
// {
// 	ros::Rate loop_rate(rate);
// 	// ------------------------------------------
// 	while (nh.ok())
// 	{
// 		loop_rate.sleep();
// 		ros::spinOnce();
// 		if(frame_flag && !input_frame.empty())
// 		{
// 			frame_flag=0;
// 			bulk_process();
// 		} 
// 	}
// 	// ------------------------------------------
// }

cv::Mat ImageProcess::main_process(cv::Mat frame)
{
	// ROS_INFO("ImageProcess::main_process()");
	// **************************
	// PROCESS
	// **************************
	return frame;
}
