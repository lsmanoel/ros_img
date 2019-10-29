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
            // Convert the passed as command line parameter index for the video device to an integer
            std::istringstream video_sourceCmd(argv[2]);
            int video_source;
            // Check if it is indeed a number
            if(!(video_sourceCmd >> video_source)) 
                return 0;
            mono_vision.set_video_source(video_source);
            mono_vision.output_frame_publisher_init("None");
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("2");
            mono_vision.set_video_source(0);
            mono_vision.output_frame_publisher_init(argv[2]);
        }
        else
        {
            ROS_INFO("3");
            mono_vision.set_video_source(0);
            mono_vision.output_frame_publisher_init("None");
		}
	}    
    else if(argc==5)
    {      
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("4");
            std::istringstream video_sourceCmd(argv[2]);
            int video_source;
            if(!(video_sourceCmd >> video_source)) 
                return 0;
            mono_vision.set_video_source(video_source);
            mono_vision.output_frame_publisher_init(argv[4]);
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("5");
            mono_vision.output_frame_publisher_init(argv[2]);
            std::istringstream video_sourceCmd(argv[4]);
            int video_source;
            if(!(video_sourceCmd >> video_source)) 
                return 0;
            mono_vision.set_video_source(video_source);
        }
        else
        {
            ROS_INFO("6");
            mono_vision.set_video_source(0);
            mono_vision.output_frame_publisher_init("None");
        }
    }
    else
    {
        ROS_INFO("7");
        mono_vision.set_video_source(0);
        mono_vision.output_frame_publisher_init("None");
    }

    mono_vision.init_video_capture();
    mono_vision.delta_t_service_init();
    mono_vision.main_loop();

	return 0;
}