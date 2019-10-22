#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <list>
#include <vector> 
#include <algorithm>
#include "ros_img/return_data.h" 
// using namespace std;

std::list<int64_t> _delta_t;

bool delta_t_service(ros_img::return_data::Request  &req, ros_img::return_data::Response &res)
{
	std::list<int64_t>::iterator iter;
	for(iter=_delta_t.begin(); iter!=_delta_t.end();iter++)
		 res.data.push_back(*iter);
}

class ImageProcess
{
	std::string name;

	int rate;

	const int delta_t_buffer_size = 1000;
	cv::Mat input_frame;
	cv::Mat output_frame;
	cv::Mat frame;

  	ros::NodeHandle nh;
 	image_transport::ImageTransport it;
	image_transport::Publisher pub_output;
	image_transport::Subscriber sub_input;
	sensor_msgs::ImagePtr msg;
	ros::ServiceServer _delta_t_service;

public:
	ImageProcess(int argc, char** argv, std::string name_in, int rate_in)
	  : it(nh)
	{
		name = name_in;

		rate = rate_in;

		ros::init(argc, argv, name);
	}

	// ----------------------------------------------------------------------------------------
	// rostopics
	void output_frame_publisher_init(std::string rostopic_name/*="None"*/)
	{
		if(rostopic_name.compare("None"))
			pub_output = it.advertise(name + "_output_frame_cpp", 1);
		else
			pub_output = it.advertise(rostopic_name, 1);
	}

	// ----
	void input_frame_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		try
		{
			input_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		}
	}

	void input_frame_subscriber_init(std::string rostopic_name/*="None"*/)
	{
		if(rostopic_name.compare("None"))
			sub_input = it.subscribe(name + "_input_frame_cpp", 1, &ImageProcess::input_frame_callback, this);
		else
			sub_input = it.subscribe(rostopic_name, 1, &ImageProcess::input_frame_callback, this);
	}

	// ----------------------------------------------------------------------------------------
	// rosservices
	//bool delta_t_service(ros_img::return_data::Request  &req, ros_img::return_data::Response &res)

	void delta_t_service_init()
	{
		_delta_t_service = nh.advertiseService(name + "_delta_t_cpp_service", delta_t_service);
	}

	// ----
	int delta_t_getter()
	{
		return _delta_t.back(); 
	}

	void delta_t_setter(int d_t)
	{
		_delta_t.push_back(d_t);
		if(_delta_t.size() > delta_t_buffer_size)
			_delta_t.pop_front();
	}

	// ----------------------------------------------------------------------------------------
	// Main Loop
	void main_loop()
	{
		int t, t0, d_t;
		ros::Rate loop_rate(rate);
		// ------------------------------------------
		while (nh.ok())
		{
			loop_rate.sleep();
			if(!input_frame.empty())
			{
				// --------------------------
				t = ros::Time::now().toNSec();
				// **************************
				main_process();
				// **************************
				// --------------------------
				t0 = ros::Time::now().toNSec();
				d_t = t - t0;
				if (d_t > 0)
					delta_t_setter(d_t);
			} 
		}
	}

	void main_process()
	{
		frame = input_frame;
		// // **************************
		// // PROCESS
		// // **************************
		output_frame = frame; 
	}

};

int main(int argc, char** argv)
{
	// ros::init(argc, argv, "image_process");
	ImageProcess img_process(argc, argv, "image_process", 30);
	ros::spin();
	return 0;
}

