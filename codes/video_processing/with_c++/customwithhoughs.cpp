#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <fstream>
#include "LaneDetector.hpp"
#include "LaneDetector.cpp"
#include "opencv2/imgproc/imgproc_c.h"

//using namespace cv;

/** Global Variables */
//const int max_value_H = 360 / 2;
//const int max_value = 255;
/** HSV VALUE SETTINGS **/
const int max_value_H = 360 / 4;
const int max_value = 255;
const cv::String window_capture_name = "Video Capture";
const cv::String window_detection_name = "Object Detection";
int low_H = 40, low_S = 45, low_V = 50;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
int maxGap = 50;
int minLength = 50;
/** HSV VALUE SETTINGS **/

//! [low]
static void on_low_H_thresh_trackbar(int, void *) {
	low_H = cv::min(high_H - 1, low_H);
	cv::setTrackbarPos("Low H", window_detection_name, low_H);
}
//! [low]

//! [high]
static void on_high_H_thresh_trackbar(int, void *) {
	high_H = cv::max(high_H, low_H + 1);
	cv::setTrackbarPos("High H", window_detection_name, high_H);
}

//! [high]
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

static void on_maxGap_thresh_trackbar(int, void *) {
	maxGap = cv::min(high_H - 1, maxGap);
	cv::setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_minLength_thresh_trackbar(int, void *) {
	minLength = cv::min(high_H - 1, minLength);
	cv::setTrackbarPos("Low H", window_detection_name, low_H);
}

int main(int argc, char* argv[]) {
	//! [cap]
	cv::VideoCapture cap("./green-640-10.mp4");
	//! [cap]
	LaneDetector lanedetector;  // Create the class object
	double avgRunTime;
	int avgCounter = 0;
	int const fps = 30;

	cv::Mat img_denoise;
	cv::Mat img_edges;
	cv::Mat img_mask;
	cv::Mat img_lines;
	std::vector<cv::Vec4i> lines;
	std::vector<std::vector<cv::Vec4i> > left_right_lines;
	std::vector<cv::Point> lane;
	std::string turn;
	int flag_plot = -1;
	double t;
	//Mat whiteFrame(426, 640, CV_8UC3, Scalar(255, 255, 255));
	//! [window]
	cv::namedWindow(window_capture_name);
	cv::namedWindow(window_detection_name);
	//! [window]

	/** TRACKBAR FOR SIMULTANEOUS HSV THRESHING **/
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
	cv::createTrackbar("maxGap", window_detection_name, &maxGap, max_value_H,
			on_maxGap_thresh_trackbar);
	cv::createTrackbar("minLength", window_detection_name, &minLength, max_value_H,
			on_minLength_thresh_trackbar);
	/** TRACKBAR FOR SIMULTANEOUS HSV THRESHING **/

	/** OUTPUT VIDEO CONFGURATIONS**/
	int codec = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
	int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
	int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
	cv::VideoWriter output_cap("output.avi", codec, 30,
			cv::Size(frame_width, frame_height), false);

	if (!output_cap.isOpened()) {
		std::cout << "!!! Output video could not be opened" << std::endl;
		return -1;
	}
	/** OUTPUT VIDEO CONFGURATIONS**/

	// Loop to read frames from the input capture and write it to the output capture
	cv::Mat frame_HSV;
	cv::Mat frame_orig, frame_fitered2D, frame_threshed, frame_cannied, frame_final;
	cv::Mat sharpenKernel = (cv::Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	cv::Mat frame_houghP;
	std::vector<cv::Vec4i> lines_houghP; // will hold the results of the detection


	while (true) {

		t = (double) cv::getTickCount(); //processing time counter

		cap >> frame_orig;
		if (frame_orig.empty())
			break;

		avgCounter++;
		/** IMAGE SHARPENER **/
		filter2D(frame_orig, frame_fitered2D, frame_orig.depth(),
				sharpenKernel);

		// Convert from BGR to HSV colorspace
		cv::cvtColor(frame_fitered2D, frame_HSV, cv::COLOR_BGR2HSV);

		// Detect the object based on HSV Range Values
		cv::inRange(frame_HSV, cv::Scalar(low_H, low_S, low_V),
				cv::Scalar(high_H, high_S, high_V), frame_threshed);

		// Canny edge detection to the masked image
		Canny(frame_threshed, frame_cannied, 200, 400, 3);

		// Copy edges to the images that will display the results in BGR
		cv::cvtColor(frame_cannied, frame_houghP, cv::COLOR_GRAY2BGR);

//		std::ofstream myfile;
//		myfile.open("test.txt", std::ios_base::app);

		// runs the line detection
		HoughLinesP(frame_cannied, lines_houghP, 1, CV_PI / 180, 50, minLength,
				maxGap);

//////////////////////////////////////////
		if (!lines_houghP.empty()) {
			// Separate lines into left and right lines
			left_right_lines = lanedetector.lineSeparation(lines_houghP,
					frame_cannied);

			// Apply regression to obtain only one line for each side of the lane
			lane = lanedetector.regression(left_right_lines, frame_threshed);

			// Predict the turn by determining the vanishing point of the the lines
			turn = lanedetector.predictTurn();

			// Plot lane detection
			flag_plot = lanedetector.plotLane(frame_orig, lane, turn);
		}
		for (size_t i = 0; i < lines_houghP.size(); i++) {
			cv::Vec4i l = lines_houghP[i];
			cv::line(frame_houghP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
					cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
		}

//////////////////////////////////////////

//	std::cout << "xTrainData (python)  = " << std::endl << format(frame_houghP, Formatter::FMT_PYTHON) << std::endl << std::endl;

		output_cap.write(frame_houghP);

		/** TOTAL PROCESS TIME **/
		t = ((double) cv::getTickCount() - t) / cv::getTickFrequency() * 1000;
		if(avgCounter == fps) {
			std::cout << "The average process time for each 30 frames in milliseconds:     " << (avgRunTime / fps) << std::endl;
			avgCounter = 0;
			avgRunTime = 0;
		}
		else avgRunTime += t;

		/** TOTAL PROCESS TIME **/

		//imshow(window_capture_name, frame_orig);
		imshow(window_detection_name, frame_houghP);

		char key = (char) cv::waitKey(30);
		if (key == 'q' || key == 27) {
			break;
		}
	}
	return 0;
}
