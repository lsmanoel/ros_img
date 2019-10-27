#include "signature_histogram/SignatureHistogram.h"

// ======================================================================================================================
int main(int argc, char** argv)
{
	std::vector<std::string> _argv;

	for(int i=0; i<argc; i++)
	{
		_argv.push_back(argv[i]);
	}

	std::string name = "signature_histogram";

	ros::init(argc, argv, name);
	SignatureHistogram signature_histogram(argc, argv, name, 30);
	
    if(argc==3)
    {
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("1");
            signature_histogram.input_frame_subscriber_init(argv[2]);
            signature_histogram.output_frame_publisher_init("None");
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("2");
            signature_histogram.output_frame_publisher_init(argv[2]);
            signature_histogram.input_frame_subscriber_init("None");
        }
        else
        {
            ROS_INFO("3");
            signature_histogram.input_frame_subscriber_init("None");
            signature_histogram.output_frame_publisher_init("None");
		}
	}    
    else if(argc==5)
    {      
        if(_argv[1].compare("-input")==0)
        {
            ROS_INFO("4");
            signature_histogram.input_frame_subscriber_init(argv[2]);
            signature_histogram.output_frame_publisher_init(argv[4]);
        }
        else if(_argv[1].compare("-output")==0)
        {
            ROS_INFO("5");
            signature_histogram.output_frame_publisher_init(argv[2]);
            signature_histogram.input_frame_subscriber_init(argv[4]);
        }
        else
        {
            ROS_INFO("6");
            signature_histogram.input_frame_subscriber_init("None");
            signature_histogram.output_frame_publisher_init("None");
        }
    }
    else
    {
        ROS_INFO("7");
        signature_histogram.input_frame_subscriber_init("None");
        signature_histogram.output_frame_publisher_init("None");
    }

    signature_histogram.delta_t_service_init();
    signature_histogram.main_loop();

	return 0;
}