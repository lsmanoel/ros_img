#include "stereo_vision/StereoVision.h"

// ======================================================================================================================
int main(int argc, char** argv)
{
	std::vector<std::string> _argv;

	for(int i=0; i<argc; i++)
	{
		_argv.push_back(argv[i]);
	}

	std::string name = "stereo_vision";

	ros::init(argc, argv, name);
	StereoVision stereo_vision(argc, argv, name, 30);
	
    if(argc==3)
    {
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("1");
            stereo_vision.input_frame_subscriber_init(argv[2]);
            stereo_vision.output_frame_publisher_init("None");
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("2");
            stereo_vision.output_frame_publisher_init(argv[2]);
            stereo_vision.input_frame_subscriber_init("None");
        }
        else
        {
            ROS_INFO("3");
            stereo_vision.input_frame_subscriber_init("None");
            stereo_vision.output_frame_publisher_init("None");
		}
	}    
    else if(argc==5)
    {      
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("4");
            stereo_vision.input_frame_subscriber_init(argv[2]);
            stereo_vision.output_frame_publisher_init(argv[4]);
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("5");
            stereo_vision.output_frame_publisher_init(argv[2]);
            stereo_vision.input_frame_subscriber_init(argv[4]);
        }
        else
        {
            ROS_INFO("6");
            stereo_vision.input_frame_subscriber_init("None");
            stereo_vision.output_frame_publisher_init("None");
        }
    }
    else
    {
        ROS_INFO("7");
        stereo_vision.input_frame_subscriber_init("None");
        stereo_vision.output_frame_publisher_init("None");
    }

    stereo_vision.delta_t_service_init();
    stereo_vision.main_loop();

	return 0;
}