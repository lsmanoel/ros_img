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
	StereoVision stereo_vision(argc, argv, name);
	
    if(argc==4)
    {
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("1");
            // Convert the passed as command line parameter index for the video device to an integer
            std::istringstream video_L_sourceCmd(argv[2]);
            std::istringstream video_R_sourceCmd(argv[3]);
            int video_L_source, video_R_source;
            // Check if it is indeed a number
            if(!(video_L_sourceCmd >> video_L_source)) 
                return 0;
            if(!(video_R_sourceCmd >> video_R_source)) 
                return 0;

            mono_vision.set_video_L_source(video_L_source);
            mono_vision.set_video_R_source(video_R_source);

            stereo_vision.output_frame_publisher_init("None");
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("2");
            stereo_vision.output_frame_publisher_init(argv[2]);
            stereo_vision.set_video_L_source(2);
            stereo_vision.set_video_R_source(4);
        }
        else
        {
            ROS_INFO("3");
            stereo_vision.set_video_L_source(2);
            stereo_vision.set_video_R_source(4);
            stereo_vision.output_frame_publisher_init("None");
		}
	}    
    else if(argc==6)
    {      
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("4");
            // Convert the passed as command line parameter index for the video device to an integer
            std::istringstream video_L_sourceCmd(argv[2]);
            std::istringstream video_R_sourceCmd(argv[3]);
            int video_L_source, video_R_source;
            // Check if it is indeed a number
            if(!(video_L_sourceCmd >> video_L_source)) 
                return 0;
            if(!(video_R_sourceCmd >> video_R_source)) 
                return 0;

            mono_vision.set_video_L_source(video_L_source);
            mono_vision.set_video_R_source(video_R_source);

            stereo_vision.output_frame_publisher_init(argv[5]);
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("5");
            stereo_vision.output_frame_publisher_init(argv[2]);
            
            // Convert the passed as command line parameter index for the video device to an integer
            std::istringstream video_L_sourceCmd(argv[4]);
            std::istringstream video_R_sourceCmd(argv[5]);
            int video_L_source, video_R_source;
            // Check if it is indeed a number
            if(!(video_L_sourceCmd >> video_L_source)) 
                return 0;
            if(!(video_R_sourceCmd >> video_R_source)) 
                return 0;

            mono_vision.set_video_L_source(video_L_source);
            mono_vision.set_video_R_source(video_R_source);

        }
        else
        {
            ROS_INFO("6");
            stereo_vision.set_video_L_source(2);
            stereo_vision.set_video_R_source(4);
            stereo_vision.output_frame_publisher_init("None");
        }
    }
    else
    {
        ROS_INFO("7");
        stereo_vision.set_video_L_source(2);
        stereo_vision.set_video_R_source(4);
        stereo_vision.output_frame_publisher_init("None");
    }

    stereo_vision.delta_t_service_init();
    stereo_vision.main_loop();

	return 0;
}