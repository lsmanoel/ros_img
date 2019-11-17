#include "kinect_vision/KinectVision.h"

// ======================================================================================================================
int main(int argc, char** argv)
{
	std::vector<std::string> _argv;

	for(int i=0; i<argc; i++)
	{
		_argv.push_back(argv[i]);
	}

	std::string name = "kinect_vision";

	ros::init(argc, argv, name);
	KinectVision kinect_vision(argc, argv, name);
	
    if(argc==3)
    {
        if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("2");
            kinect_vision.output_frame_publisher_init(argv[2]);
        }
        else
        {
            ROS_INFO("3");
            kinect_vision.output_frame_publisher_init("None");
		}
	}    
    else
    {
        ROS_INFO("7");
        kinect_vision.input_frame_subscriber_init("None");
        kinect_vision.output_frame_publisher_init("None");
    }

    kinect_vision.delta_t_service_init();
    kinect_vision.main_loop();

	return 0;
}