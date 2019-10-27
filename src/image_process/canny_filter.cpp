#include "canny_filter/CannyFilter.h"

// ======================================================================================================================
int main(int argc, char** argv)
{
	std::vector<std::string> _argv;

	for(int i=0; i<argc; i++)
	{
		_argv.push_back(argv[i]);
	}

	std::string name = "canny_filter";

	ros::init(argc, argv, name);
	CannyFilter canny_filter(argc, argv, name, 30);
	
    if(argc==3)
    {
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("1");
            canny_filter.input_frame_subscriber_init(argv[2]);
            canny_filter.output_frame_publisher_init("None");
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("2");
            canny_filter.output_frame_publisher_init(argv[2]);
            canny_filter.input_frame_subscriber_init("None");
        }
        else
        {
            ROS_INFO("3");
            canny_filter.input_frame_subscriber_init("None");
            canny_filter.output_frame_publisher_init("None");
		}
	}    
    else if(argc==5)
    {      
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("4");
            canny_filter.input_frame_subscriber_init(argv[2]);
            canny_filter.output_frame_publisher_init(argv[4]);
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("5");
            canny_filter.output_frame_publisher_init(argv[2]);
            canny_filter.input_frame_subscriber_init(argv[4]);
        }
        else
        {
            ROS_INFO("6");
            canny_filter.input_frame_subscriber_init("None");
            canny_filter.output_frame_publisher_init("None");
        }
    }
    else
    {
        ROS_INFO("7");
        canny_filter.input_frame_subscriber_init("None");
        canny_filter.output_frame_publisher_init("None");
    }

    canny_filter.delta_t_service_init();
    canny_filter.main_loop();

	return 0;
}