#include "SignatureHistogram.h"

SignatureHistogram::SignatureHistogram(int argc, char** argv, std::string name_in)
	:ImageProcess(argc, argv, name_in)
{
	input_frame_type = "bgr8";
	output_frame_type = "mono8";

	for(int i=0; i<VIEW_FRAME_HEIGHT; i++)
		h_histogram.push_back(0);

	for(int i=0; i<VIEW_FRAME_WIDTH; i++)
		v_histogram.push_back(0);
}

// ----------------------------------------------------------------------------------------
cv::Mat SignatureHistogram::horizontal_edge_signature(cv::Mat frame)
{
	ROS_INFO("%d", frame.rows);

	cv::cvtColor(frame, frame, CV_BGR2GRAY);
	cv::Scharr(frame, frame, CV_32F, 0 ,1 , -1, 1, 0);
	cv::convertScaleAbs(frame, frame);
	cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0);

 	/* 0: Binary
 	 1: Binary Inverted
 	 2: Threshold Truncated
 	 3: Threshold to Zero
 	 4: Threshold to Zero Inverted
 	*/
 	// cv::adaptiveThreshold(frame, frame, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, 105, 1);

 	// cv::threshold(frame, frame, 120, 255, CV_THRESH_BINARY);

	return frame;
}

cv::Mat SignatureHistogram::vertical_edge_signature(cv::Mat frame)
{
	ROS_INFO("%d", frame.cols);

	// for(int i=0; i<VIEW_FRAME_WIDTH; i++)
	// {
	// 	vertical_histogram[i] =
	// }

	// // for(int i; i<frame.rows)
	// // {
	// frame = cv2.Sobel(frame, cv2.CV_32F, 0, 1, -1)
	// }
	return frame;
}

cv::Mat SignatureHistogram::horizontal_histogram(cv::Mat frame)
{
	return frame;
}

cv::Mat SignatureHistogram::vertical_histogram(cv::Mat frame)
{
	return frame;
}

cv::Mat SignatureHistogram::main_process(cv::Mat frame)
{
	// ROS_INFO("SignatureHistogram::main_process()");
	// // **************************
	// // PROCESS
	frame = horizontal_edge_signature(frame);
	// // **************************
	return frame;
}

// # frame_output = np.zeros((frame_input.shape))
// histogram = np.zeros(frame.shape[0])

// for i, line in enumerate(frame[:,]):
//     line_energy = np.sum(line/255)
//     # print(line)
//     histogram[i] = int(line_energy)

// return histogram