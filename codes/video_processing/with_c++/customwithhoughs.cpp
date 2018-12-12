#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <fstream>
#include "LaneDetector.hpp"
#include "LaneDetector.cpp"
#include "opencv2/imgproc/imgproc_c.h"

using namespace cv;

/** Global Variables */
//const int max_value_H = 360 / 2;
//const int max_value = 255;
/** HSV VALUE SETTINGS **/
const int max_value_H = 360 / 4;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 40, low_S = 45, low_V = 50;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
int maxGap = 50;
int minLength = 50;
/** HSV VALUE SETTINGS **/

//! [low]
static void on_low_H_thresh_trackbar(int, void *) {
	low_H = min(high_H - 1, low_H);
	setTrackbarPos("Low H", window_detection_name, low_H);
}
//! [low]

//! [high]
static void on_high_H_thresh_trackbar(int, void *) {
	high_H = max(high_H, low_H + 1);
	setTrackbarPos("High H", window_detection_name, high_H);
}

//! [high]
static void on_low_S_thresh_trackbar(int, void *) {
	low_S = min(high_S - 1, low_S);
	setTrackbarPos("Low S", window_detection_name, low_S);
}

static void on_high_S_thresh_trackbar(int, void *) {
	high_S = max(high_S, low_S + 1);
	setTrackbarPos("High S", window_detection_name, high_S);
}

static void on_low_V_thresh_trackbar(int, void *) {
	low_V = min(high_V - 1, low_V);
	setTrackbarPos("Low V", window_detection_name, low_V);
}

static void on_high_V_thresh_trackbar(int, void *) {
	high_V = max(high_V, low_V + 1);
	setTrackbarPos("High V", window_detection_name, high_V);
}

static void on_maxGap_thresh_trackbar(int, void *) {
	maxGap = min(high_H - 1, maxGap);
	setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_minLength_thresh_trackbar(int, void *) {
	minLength = min(high_H - 1, minLength);
	setTrackbarPos("Low H", window_detection_name, low_H);
}


int main(int argc, char* argv[]) {
	//! [cap]
	VideoCapture cap("./green-640-10.mp4");
	//! [cap]
	LaneDetector lanedetector;  // Create the class object

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
	Mat whiteFrame(426, 640, CV_8UC3, Scalar(255, 255, 255));
	//! [window]
	namedWindow(window_capture_name);
	namedWindow(window_detection_name);
	//! [window]

	/** TRACKBAR FOR SIMULTANEOUS HSV THRESHING **/
	createTrackbar("Low H", window_detection_name, &low_H, max_value_H,
			on_low_H_thresh_trackbar);
	createTrackbar("High H", window_detection_name, &high_H, max_value_H,
			on_high_H_thresh_trackbar);
	createTrackbar("Low S", window_detection_name, &low_S, max_value,
			on_low_S_thresh_trackbar);
	createTrackbar("High S", window_detection_name, &high_S, max_value,
			on_high_S_thresh_trackbar);
	createTrackbar("Low V", window_detection_name, &low_V, max_value,
			on_low_V_thresh_trackbar);
	createTrackbar("High V", window_detection_name, &high_V, max_value,
			on_high_V_thresh_trackbar);
	createTrackbar("maxGap", window_detection_name, &maxGap, max_value_H,
			on_maxGap_thresh_trackbar);
	createTrackbar("minLength", window_detection_name, &minLength, max_value_H,
			on_minLength_thresh_trackbar);
	/** TRACKBAR FOR SIMULTANEOUS HSV THRESHING **/

	/** OUTPUT VIDEO CONFGURATIONS**/
	int codec = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
	int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
	int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
	cv::VideoWriter output_cap("output.avi", codec, 30,
			Size(frame_width, frame_height), false);

	if (!output_cap.isOpened()) {
		std::cout << "!!! Output video could not be opened" << std::endl;
		return -1;
	}
	/** OUTPUT VIDEO CONFGURATIONS**/

	// Loop to read frames from the input capture and write it to the output capture
	Mat frame, frame_p, frame_HSV, frame_threshold;
	int a = 0;
	while (true) {
		//a++;
		t = (double) getTickCount(); //processing time counter
		//! [while]
		cap >> frame;
		if (frame.empty()) {
			break;
		}

		/** IMAGE SHARPENER **/
		Mat kernel = (Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
		//![kern]

		filter2D(frame, frame_p, frame.depth(), kernel);
		/** IMAGE SHARPENER **/

		// Convert from BGR to HSV colorspace
		cvtColor(frame_p, frame_HSV, COLOR_BGR2HSV);

		// Detect the object based on HSV Range Values
		inRange(frame_HSV, Scalar(low_H, low_S, low_V),
				Scalar(high_H, high_S, high_V), frame_threshold);

		/* CHECK MASKING AND CANNY */

		// Mask the filtered image with the original image
		bitwise_and(frame_threshold, frame_threshold, frame_p);

		// Canny edge detection to the masked image
		Canny(frame_threshold, frame_threshold, 200, 400, 3);
		/**/
		Mat cdst, cdstP;

		// Copy edges to the images that will display the results in BGR
		cvtColor(frame_threshold, cdst, COLOR_GRAY2BGR);
		cdstP = cdst.clone();

		//![hough_lines]
		// Standard Hough Line Transform
		std::vector<Vec2f> lines; // will hold the results of the detection

		std::ofstream myfile;
		myfile.open("test.txt", std::ios_base::app);
		//![hough_lines_p]
		// Probabilistic Line Transform
		std::vector<Vec4i> linesP; // will hold the results of the detection

		HoughLinesP(frame_threshold, linesP, 1, CV_PI / 180, 50, minLength, maxGap); // runs the actual detection

//////////////////////////////////////////
		if (!linesP.empty()) {
			// Separate lines into left and right lines
			left_right_lines = lanedetector.lineSeparation(linesP,
					frame_threshold);

			// Apply regression to obtain only one line for each side of the lane
			lane = lanedetector.regression(left_right_lines, frame);

			// Predict the turn by determining the vanishing point of the the lines
			turn = lanedetector.predictTurn();

			// Plot lane detection
			flag_plot = lanedetector.plotLane(frame, lane, turn);
		}
		for (size_t i = 0; i < linesP.size(); i++) {
			Vec4i l = linesP[i];
			line(cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255),
					3, LINE_AA);
		}

//////////////////////////////////////////

//	std::cout << "xTrainData (python)  = " << std::endl << format(cdstP, Formatter::FMT_PYTHON) << std::endl << std::endl;
		bitwise_and(whiteFrame, cdstP, frame_threshold);
		bitwise_and(frame_threshold, cdstP, frame_threshold);

		output_cap.write(frame_threshold);

		/** TOTAL PROCESS TIME **/
		t = ((double) getTickCount() - t) / getTickFrequency();
		std::cout << "The process time in seconds:     " << t << std::endl;
		//std::cout << frame_threshold << std::endl;
		/** TOTAL PROCESS TIME **/

		imshow(window_capture_name, frame);
		imshow(window_detection_name, frame_threshold);
		//! [show]

		char key = (char) waitKey(30);
		if (key == 'q' || key == 27) {
			break;
		}
	}
	return 0;
}
