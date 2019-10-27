#include "move_frame/MoveFrame.h"

// ======================================================================================================================
int main(int argc, char** argv)
{
	std::vector<std::string> _argv;

	for(int i=0; i<argc; i++)
	{
		_argv.push_back(argv[i]);
	}

	std::string name = "move_frame";

	ros::init(argc, argv, name);
	MoveFrame move_frame(argc, argv, name, 30);
	
    if(argc==3)
    {
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("1");
            move_frame.input_frame_subscriber_init(argv[2]);
            move_frame.output_frame_publisher_init("None");
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("2");
            move_frame.output_frame_publisher_init(argv[2]);
            move_frame.input_frame_subscriber_init("None");
        }
        else
        {
            ROS_INFO("3");
            move_frame.input_frame_subscriber_init("None");
            move_frame.output_frame_publisher_init("None");
		}
	}    
    else if(argc==5)
    {      
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("4");
            move_frame.input_frame_subscriber_init(argv[2]);
            move_frame.output_frame_publisher_init(argv[4]);
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("5");
            move_frame.output_frame_publisher_init(argv[2]);
            move_frame.input_frame_subscriber_init(argv[4]);
        }
        else
        {
            ROS_INFO("6");
            move_frame.input_frame_subscriber_init("None");
            move_frame.output_frame_publisher_init("None");
        }
    }
    else
    {
        ROS_INFO("7");
        move_frame.input_frame_subscriber_init("None");
        move_frame.output_frame_publisher_init("None");
    }

    move_frame.delta_t_service_init();
    move_frame.main_loop();

	return 0;
}