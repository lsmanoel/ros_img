#include "MoveFrame.h"

MoveFrame::MoveFrame(int argc, char** argv, std::string name_in)
	:ImageProcess(argc, argv, name_in)
{
	input_frame_type = "bgr8";
	output_frame_type = "bgr8";
}

// ----------------------------------------------------------------------------------------
cv::Mat MoveFrame::main_process(cv::Mat frame)
{
	// ROS_INFO("MoveFrame::main_process()");
	// **************************
	// PROCESS
    // Input Quadilateral or Image plane coordinates
    cv::Point2f inputQuad[4]; 
    // Output Quadilateral or World plane coordinates
    cv::Point2f outputQuad[4];
         
    // Lambda Matrix
    cv::Mat lambda( 2, 4, CV_32FC1 );
    // Set the lambda matrix the same type and size as input
    lambda = cv::Mat::zeros(frame.rows, frame.cols, frame.type());
 
    // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input 
    inputQuad[0] = cv::Point2f(-30, -60 );
    inputQuad[1] = cv::Point2f(frame.cols+50, -50);
    inputQuad[2] = cv::Point2f(frame.cols+100, frame.rows+50);
    inputQuad[3] = cv::Point2f(-50, frame.rows+50 );  
    // The 4 points where the mapping is to be done , from top-left in clockwise order
    outputQuad[0] = cv::Point2f(0, 0);
    outputQuad[1] = cv::Point2f(frame.cols-1, 0);
    outputQuad[2] = cv::Point2f(frame.cols-1, frame.rows-1);
    outputQuad[3] = cv::Point2f(0, frame.rows-1 );
 
    // Get the Perspective Transform Matrix i.e. lambda 
    lambda = cv::getPerspectiveTransform(inputQuad, outputQuad);
    // Apply the Perspective Transform just found to the src image
    cv::warpPerspective(frame, frame, lambda, frame.size());
	// **************************
	return frame;
}
