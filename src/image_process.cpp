#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <list>
#include <vector> 
#include <algorithm>
#include "ros_img/return_data.h" 
#include <iostream>
#include <std_msgs/Int64.h>
#include "std_msgs/Int64MultiArray.h"
// using namespace std;


class ImageProcess
{
	std::string name;

	int rate;
	const int delta_t_buffer_size = 1000;

	std_msgs::Int64MultiArray _delta_t;
	// _delta_t.data.clear();

	const int FULL_FRAME_WIDTH = 640;
 	const int FULL_FRAME_HEIGHT = 480;
  	const int VIEW_FRAME_WIDTH = 640;
  	const int VIEW_FRAME_HEIGHT = 480;

	cv::Mat input_frame;
	int frame_flag = 0;
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

		frame_flag = 0;

		input_frame = cv::Mat(VIEW_FRAME_HEIGHT, VIEW_FRAME_WIDTH, CV_8UC3);
		output_frame = cv::Mat(VIEW_FRAME_HEIGHT, VIEW_FRAME_WIDTH, CV_8UC3);
		frame = cv::Mat(VIEW_FRAME_HEIGHT, VIEW_FRAME_WIDTH, CV_8UC3);

		// input_frame.reserve(2000);
		// output_frame.reserve(2000);
		// frame.reserve(2000);
	}

	// ----------------------------------------------------------------------------------------
	// rostopics
	void output_frame_publisher_init(std::string rostopic_name/*="None"*/)
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
	void input_frame_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		try
		{
			frame_flag = 1;
			input_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		}
	}

	void input_frame_subscriber_init(std::string rostopic_name/*="None"*/)
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
	bool delta_t_service(ros_img::return_data::Request  &req, ros_img::return_data::Response &res)
	{
		// std::list<std_msgs::Int64>::iterator iter;
		// for(iter=_delta_t.data.begin(); iter!=_delta_t.data.end();iter++)
			 // res.data.push_back(*iter);
		res.data = _delta_t.data;
	}

	void delta_t_service_init()
	{
		_delta_t_service = nh.advertiseService(name + "_delta_t_cpp_service", &ImageProcess::delta_t_service, this);
	}

	// ----
	int delta_t_getter()
	{
		return _delta_t.data.back(); 
	}

	void delta_t_setter(std_msgs::Int64 d_t)
	{
		// ROS_INFO("dt: %d", d_t);
		_delta_t.data.push_back(d_t.data);
		if(_delta_t.data.size() > delta_t_buffer_size)
			_delta_t.data.pop_back();
	}

	// ----------------------------------------------------------------------------------------
	// Main Loop
	void main_loop()
	{
		std_msgs::Int64 t, t0, d_t;
		ros::Rate loop_rate(rate);
		// ------------------------------------------
		while (nh.ok())
		{
			loop_rate.sleep();
			ros::spinOnce();
			if(frame_flag && !input_frame.empty())
			{
				frame_flag=0;
				// --------------------------
				t0.data = ros::Time::now().toNSec();
				// **************************	
				main_process();
				// **************************
				msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_frame).toImageMsg();
				pub_output.publish(msg);

				// --------------------------
				t.data = ros::Time::now().toNSec();
				d_t.data = t.data - t0.data;
				if (d_t.data > 0)
					delta_t_setter(d_t);
			} 
		}
		// ------------------------------------------
	}

	void main_process()
	{
		input_frame.copyTo(frame);
		// // **************************
		// // PROCESS
		// cv::cvtColor(input_frame, output_frame, CV_RGB2BGR);
		// cv::cvtColor(output_frame, output_frame, CV_RGB2BGR);
		// // **************************
		frame.copyTo(output_frame); 
	}

};


// ======================================================================================================================
int main(int argc, char** argv)
{
	std::vector<std::string> _argv;

	for(int i=0; i<argc; i++)
	{
		_argv.push_back(argv[i]);
	}

	std::string name = "image_process";

	ros::init(argc, argv, name);
	ImageProcess image_process(argc, argv, name, 30);
	
    if(argc==3)
    {
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("1");
            image_process.input_frame_subscriber_init(argv[2]);
            image_process.output_frame_publisher_init("None");
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("2");
            image_process.output_frame_publisher_init(argv[2]);
            image_process.input_frame_subscriber_init("None");
        }
        else
        {
            ROS_INFO("3");
            image_process.input_frame_subscriber_init("None");
            image_process.output_frame_publisher_init("None");
		}
	}    
    else if(argc==5)
    {      
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("4");
            image_process.input_frame_subscriber_init(argv[2]);
            image_process.output_frame_publisher_init(argv[4]);
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("5");
            image_process.output_frame_publisher_init(argv[2]);
            image_process.input_frame_subscriber_init(argv[4]);
        }
        else
        {
            ROS_INFO("6");
            image_process.input_frame_subscriber_init("None");
            image_process.output_frame_publisher_init("None");
        }
    }
    else
    {
        ROS_INFO("7");
        image_process.input_frame_subscriber_init("None");
        image_process.output_frame_publisher_init("None");
    }

    image_process.delta_t_service_init();
    image_process.main_loop();

	return 0;
}

