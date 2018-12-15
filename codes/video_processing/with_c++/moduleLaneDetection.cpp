#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include <iostream>
//#include <fstream>
#include "LaneDetector.hpp"
#include "LaneDetector.cpp"
#include "opencv2/imgproc/imgproc_c.h"

/** HSV PARAMETER SETTINGS **/
const int max_value_H = 360 / 4;
const int max_value = 255;
const cv::String window_detection_name = "Lane Detection";
int low_H = 40, low_S = 45, low_V = 50;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
int maxGap = 50;
int minLength = 50;
/** HSV PARAMETER SETTINGS **/

static void on_low_H_thresh_trackbar(int, void *);
static void on_high_H_thresh_trackbar(int, void *);
static void on_low_S_thresh_trackbar(int, void *);
static void on_high_S_thresh_trackbar(int, void *);
static void on_low_V_thresh_trackbar(int, void *);
static void on_high_V_thresh_trackbar(int, void *);

/**
 *@brief Function main executes the algorithm of the lane detection.
 *@brief It reads a video of a green lane and it will output the
 *@brief same video scene but with the detected lane.
 *@param argv[] is a string to the full path of the demo video
 *@return flag_plot tells if the demo has sucessfully finished
 */
int main(int argc, char* argv[]) {

	cv::VideoCapture cap("./green-640-15.mp4");
	LaneDetector lanedetector;  // Create the class object

	// variables for calculating average process time
	double avgRunTime;
	int avgCounter = 0;
	int const fps = 30;
	double timeCapture;

	// image processing variables
	int frameWidth = 0;
	int frameHeight = 0;
	cv::Mat img_denoise;
	cv::Mat img_edges;
	cv::Mat img_mask;
	cv::Mat img_lines;
	std::vector<cv::Vec4i> lines;
	std::vector<std::vector<cv::Vec4i> > left_right_lines;
	std::vector<cv::Point> lane;
	std::string turn;
	int flag_plot = -1;

	//Mat whiteFrame(426, 640, CV_8UC3, Scalar(255, 255, 255));

	// create output window
	cv::namedWindow(window_detection_name);

	// create trackbar for editing the HSV filtering
	cv::createTrackbar("Low H", window_detection_name, &low_H, max_value_H,
			on_low_H_thresh_trackbar);
	cv::createTrackbar("High H", window_detection_name, &high_H, max_value_H,
			on_high_H_thresh_trackbar);
	cv::createTrackbar("Low S", window_detection_name, &low_S, max_value,
			on_low_S_thresh_trackbar);
	cv::createTrackbar("High S", window_detection_name, &high_S, max_value,
			on_high_S_thresh_trackbar);
	cv::createTrackbar("Low V", window_detection_name, &low_V, max_value,
			on_low_V_thresh_trackbar);
	cv::createTrackbar("High V", window_detection_name, &high_V, max_value,
			on_high_V_thresh_trackbar);

	cv::Mat frame_HSV;
	cv::Mat frame_orig, frame_fitered2D, frame_threshed, frame_cannied,
			frame_final;
	cv::Mat sharpenKernel =
			(cv::Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	cv::Mat frame_houghP;
	std::vector<cv::Vec4i> lines_houghP; // will hold the results of the detection

	// loop through live frames
	while (true) {

		timeCapture = (double) cv::getTickCount(); // capture the starting time

		cap >> frame_orig;

		// check if the input video can be opened
		if (frame_orig.empty()) {
			std::cout << "!!! Input video could not be opened" << std::endl;
			return -1;
		}
		avgCounter++; // increment the process counter
		frameHeight = frame_orig.rows;
		frameWidth = frame_orig.cols;

		// denoise the frame using a Gaussian filter
		img_denoise = lanedetector.deNoise(frame_orig);

		// convert from BGR to HSV colorspace
		cv::cvtColor(img_denoise, frame_HSV, cv::COLOR_BGR2HSV);

		// apply color thresholding HSV range for green color
		cv::inRange(frame_HSV, cv::Scalar(low_H, low_S, low_V),
				cv::Scalar(high_H, high_S, high_V), frame_threshed);

		// canny edge detection to the color thresholded image
		Canny(frame_threshed, frame_cannied, 200, 400, 3);

		// copy cannied image
		cv::cvtColor(frame_cannied, frame_houghP, cv::COLOR_GRAY2BGR);

//		std::ofstream myfile;
//		myfile.open("test.txt", std::ios_base::app);

		//img_mask = lanedetector.cropROI(frame_cannied);
		// runs the line detection

		lines_houghP = lanedetector.houghLines(frame_cannied);

		if (!lines_houghP.empty()) {
			// Separate lines into left and right lines
			left_right_lines = lanedetector.lineSeparation(lines_houghP,
					frame_cannied);

			// Apply regression to obtain only one line for each side of the lane
			lane = lanedetector.regression(left_right_lines, frame_threshed);

			// Plot lane detection
			flag_plot = lanedetector.plotLane(frame_orig, lane);
		}
		for (size_t i = 0; i < lines_houghP.size(); i++) {
			cv::Vec4i l = lines_houghP[i];
			cv::line(frame_houghP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
					cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
		}
		//	std::cout << "xTrainData (python)  = " << std::endl << format(frame_houghP, Formatter::FMT_PYTHON) << std::endl << std::endl;

		// calculate the process time
		timeCapture = ((double) cv::getTickCount() - timeCapture)
				/ cv::getTickFrequency() * 1000;
		if (avgCounter == fps) {
			std::cout
					<< "The average process time for each 30 frames in milliseconds:     "
					<< (avgRunTime / fps) << std::endl;
			avgCounter = 0;
			avgRunTime = 0;
		} else
			avgRunTime += timeCapture;

		//imshow(window_capture_name, frame_orig);
		imshow(window_detection_name, frame_houghP);

		char key = (char) cv::waitKey(30);
		if (key == 'q' || key == 27) {
			break;
		}
	}
	return 0;
}

static void on_low_H_thresh_trackbar(int, void *) {
	low_H = cv::min(high_H - 1, low_H);
	cv::setTrackbarPos("Low H", window_detection_name, low_H);
}

static void on_high_H_thresh_trackbar(int, void *) {
	high_H = cv::max(high_H, low_H + 1);
	cv::setTrackbarPos("High H", window_detection_name, high_H);
}

static void on_low_S_thresh_trackbar(int, void *) {
	low_S = cv::min(high_S - 1, low_S);
	cv::setTrackbarPos("Low S", window_detection_name, low_S);
}

static void on_high_S_thresh_trackbar(int, void *) {
	high_S = cv::max(high_S, low_S + 1);
	cv::setTrackbarPos("High S", window_detection_name, high_S);
}

static void on_low_V_thresh_trackbar(int, void *) {
	low_V = cv::min(high_V - 1, low_V);
	cv::setTrackbarPos("Low V", window_detection_name, low_V);
}

static void on_high_V_thresh_trackbar(int, void *) {
	high_V = cv::max(high_V, low_V + 1);
	cv::setTrackbarPos("High V", window_detection_name, high_V);
}
