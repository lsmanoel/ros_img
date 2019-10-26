#include "ImageProcess.h"

// ======================================================================================================================
#ifndef __CANNY_FILTER__

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

#endif
