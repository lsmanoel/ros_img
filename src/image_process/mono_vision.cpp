#include "mono_vision/MonoVision.h"

// ======================================================================================================================
int main(int argc, char** argv)
{
	std::vector<std::string> _argv;

	for(int i=0; i<argc; i++)
	{
		_argv.push_back(argv[i]);
	}

	std::string name = "mono_vision";

	ros::init(argc, argv, name);
	MonoVision mono_vision(argc, argv, name, 30);
	
    if(argc==3)
    {
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("1");
            mono_vision.input_frame_subscriber_init(argv[2]);
            mono_vision.output_frame_publisher_init("None");
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("2");
            mono_vision.output_frame_publisher_init(argv[2]);
            mono_vision.input_frame_subscriber_init("None");
        }
        else
        {
            ROS_INFO("3");
            mono_vision.input_frame_subscriber_init("None");
            mono_vision.output_frame_publisher_init("None");
		}
	}    
    else if(argc==5)
    {      
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("4");
            mono_vision.input_frame_subscriber_init(argv[2]);
            mono_vision.output_frame_publisher_init(argv[4]);
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("5");
            mono_vision.output_frame_publisher_init(argv[2]);
            mono_vision.input_frame_subscriber_init(argv[4]);
        }
        else
        {
            ROS_INFO("6");
            mono_vision.input_frame_subscriber_init("None");
            mono_vision.output_frame_publisher_init("None");
        }
    }
    else
    {
        ROS_INFO("7");
        mono_vision.input_frame_subscriber_init("None");
        mono_vision.output_frame_publisher_init("None");
    }

    mono_vision.delta_t_service_init();
    mono_vision.main_loop();

	return 0;
}