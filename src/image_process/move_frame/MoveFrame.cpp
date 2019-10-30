#include "MoveFrame.h"

MoveFrame::MoveFrame(int argc, char** argv, std::string name_in)
	:ImageProcess(argc, argv, name_in)
{
	input_frame_type = "bgr8";
	output_frame_type = "bgr8";

    lambda_matrix = cv::Mat(2, 4, CV_32FC1);

    lambda_matrix = cv::Mat::zeros(FULL_FRAME_HEIGHT, FULL_FRAME_WIDTH, CV_32FC1);
}


// ----------------------------------------------------------------------------------------
cv::Mat MoveFrame::main_process(cv::Mat frame)
{
    // ROS_INFO("MoveFrame::main_process()");
    // **************************
    // PROCESS
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
 
    // Get the Perspective Transform Matrix i.e. lambda_matrix 
    lambda_matrix = cv::getPerspectiveTransform(inputQuad, outputQuad);
    // Apply the Perspective Transform just found to the src image
    cv::warpPerspective(frame, frame, lambda_matrix, frame.size());
    // **************************
    return frame;
}
