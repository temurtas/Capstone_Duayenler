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
const cv::String window_lane_detected = "Lane Detection";
const cv::String winodw_hsv_filtered = "HSV Filtered";
const cv::String window_canny_applied = "Canny Applied";
const cv::String window_masked = "Masked";

int low_H = 60, low_S = 120, low_V = 106; // low_S = 144,introduces bug
int high_H = max_value_H, high_S = max_value, high_V = max_value;
int maxGap = 50;
int minLength = 50;
/** HSV PARAMETER SETTINGS **/

/** **/
int 	threshold = 20;
int 	maxLineGap = 20;
int 	minLineLength = 30;
/** **/

static void on_low_H_thresh_trackbar(int, void *);
static void on_high_H_thresh_trackbar(int, void *);
static void on_low_S_thresh_trackbar(int, void *);
static void on_high_S_thresh_trackbar(int, void *);
static void on_low_V_thresh_trackbar(int, void *);
static void on_high_V_thresh_trackbar(int, void *);

static void on_threshold_thresh_trackbar(int, void *);
static void on_maxLineGap_trackbar(int, void *);
static void on_minLineLength_thresh_trackbar(int, void *);

void quickSort(std::vector<cv::Vec4i> & v, unsigned int low, unsigned int high);
unsigned int pivot (std::vector<cv::Vec4i> & v, unsigned int start,
	unsigned int stop, unsigned int position);

/**
 *@brief Function main executes the algorithm of the lane detection.
 *@brief It reads a video of a green lane and it will output the
 *@brief same video scene but with the detected lane.
 *@param argv[] is a string to the full path of the demo video
 *@return flag_plot tells if the demo has sucessfully finished
 */
int main(int argc, char* argv[]) {

	cv::VideoCapture cap("./videos/green-640-15.mp4");
	//cv::VideoCapture cap(0, cv::CAP_V4L2);
	LaneDetector lanedetector;  // Create the class object
	//LaneDetector lanedetector;  // Create the class object

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
	cv::Mat frame_masked;
	cv::Mat img_lines;
	std::vector<cv::Vec4i> lines;
	std::vector<std::vector<cv::Vec4i> > left_right_lines;
	std::vector<cv::Point> lane;
	std::string turn;
	int flag_plot = -1;

	// colors for hough lines
	int red = 250;
	int green = 250;

	//Mat whiteFrame(426, 640, CV_8UC3, Scalar(255, 255, 255));

	// create output window
	cv::namedWindow(window_lane_detected);

	// create trackbar for editing the HSV filtering
	cv::createTrackbar("Low H", window_lane_detected, &low_H, max_value_H,
			on_low_H_thresh_trackbar);
	cv::createTrackbar("High H", window_lane_detected, &high_H, max_value_H,
			on_high_H_thresh_trackbar);
	cv::createTrackbar("Low S", window_lane_detected, &low_S, max_value,
			on_low_S_thresh_trackbar);
	cv::createTrackbar("High S", window_lane_detected, &high_S, max_value,
			on_high_S_thresh_trackbar);
	cv::createTrackbar("Low V", window_lane_detected, &low_V, max_value,
			on_low_V_thresh_trackbar);
	cv::createTrackbar("High V", window_lane_detected, &high_V, max_value,
			on_high_V_thresh_trackbar);

	cv::createTrackbar("threshold", window_lane_detected, &threshold, max_value,
			on_threshold_thresh_trackbar);
	cv::createTrackbar("maxLineGap", window_lane_detected, &maxLineGap, max_value,
			on_maxLineGap_trackbar);
	cv::createTrackbar("minLineLength", window_lane_detected, &minLineLength, max_value,
			on_minLineLength_thresh_trackbar);

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
		// (50,200,3)
		Canny(frame_threshed, frame_cannied, 133, 400, 5, true);

		// copy cannied image
		cv::cvtColor(frame_cannied, frame_houghP, cv::COLOR_GRAY2BGR);

//		std::ofstream myfile;
//		myfile.open("test.txt", std::ios_base::app);

		frame_masked = lanedetector.cropROI(frame_cannied);
		// runs the line detection
		std::vector<cv::Vec4i> line;
		HoughLinesP(frame_masked, lines_houghP, 1, CV_PI / 180, threshold,(double) maxLineGap, (double)minLineLength);
//		for (auto i : lines_houghP) {
//			std::cout << i << std::endl;
//		}
		// sort the found lines from smallest x to largest x coordinate
		quickSort(lines_houghP, 0, lines_houghP.size());
//		std::cout << lines_houghP[0][1] << std::endl;
/*		for (auto i : lines_houghP) {
			std::cout << i << std::endl;
		}
*/
		if (!lines_houghP.empty()) {
			// Separate lines into left and right lines
			left_right_lines = lanedetector.lineSeparation(lines_houghP,
					frame_masked);

			// Apply regression to obtain only one line for each side of the lane
			lane = lanedetector.regression(left_right_lines, frame_threshed);

			// Plot lane detection
			flag_plot = lanedetector.plotLane(frame_orig, lane);
		}
		for (size_t i = 0; i < lines_houghP.size(); i++) {
			cv::Vec4i l = lines_houghP[i];
			if (red<0) red = 155;
			if (green<0) green = 55;
			cv::line(frame_houghP, cv::Point(l[0], l[1]),
					cv::Point(l[2], l[3]),
					cv::Scalar(255, green, red), 3, cv::LINE_AA);
			red = red - 20;
			green = green - 20;
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
		imshow(window_lane_detected, frame_houghP);
		imshow(winodw_hsv_filtered, frame_threshed);
		imshow(window_canny_applied, frame_cannied);
		imshow(window_masked, frame_masked);

		red =250;
		green = 250;

		char key = (char) cv::waitKey(30);
		if (key == 'q' || key == 27) {
			break;
		}

		std::cin.get();
	}
	return 0;
}

static void on_low_H_thresh_trackbar(int, void *) {
	low_H = cv::min(high_H - 1, low_H);
	cv::setTrackbarPos("Low H", window_lane_detected, low_H);
}

static void on_high_H_thresh_trackbar(int, void *) {
	high_H = cv::max(high_H, low_H + 1);
	cv::setTrackbarPos("High H", window_lane_detected, high_H);
}

static void on_low_S_thresh_trackbar(int, void *) {
	low_S = cv::min(high_S - 1, low_S);
	cv::setTrackbarPos("Low S", window_lane_detected, low_S);
}

static void on_high_S_thresh_trackbar(int, void *) {
	high_S = cv::max(high_S, low_S + 1);
	cv::setTrackbarPos("High S", window_lane_detected, high_S);
}

static void on_low_V_thresh_trackbar(int, void *) {
	low_V = cv::min(high_V - 1, low_V);
	cv::setTrackbarPos("Low V", window_lane_detected, low_V);
}

static void on_high_V_thresh_trackbar(int, void *) {
	high_V = cv::max(high_V, low_V + 1);
	cv::setTrackbarPos("High V", window_lane_detected, high_V);
}
static void on_threshold_thresh_trackbar(int, void *) {
	threshold = cv::min(high_V - 1, threshold);
	cv::setTrackbarPos("threshold", window_lane_detected, threshold);
}
static void on_maxLineGap_trackbar(int, void *) {
	maxLineGap = cv::min(high_V - 1, maxLineGap);
	cv::setTrackbarPos("maxLineGap", window_lane_detected, maxLineGap);
}
static void on_minLineLength_thresh_trackbar(int, void *) {
	minLineLength = cv::min(high_V - 1, minLineLength);
	cv::setTrackbarPos("minLineLength", window_lane_detected, minLineLength);
}

/*void quickSort(std::vector<cv::Vec4i>& array, int low, int high) {
	int i = low;
	int j = high;
	int pivot = array[(i + j) / 2][0];
	int temp;

	while (i <= j) {
		while (array[i][0] < pivot)
			i++;
		while (array[j][0] > pivot)
			j--;
		if (i <= j) {

			temp = array[i];
			array[i] = array[j];
			array[j] = temp;

			std::swap(array[i],array[j]);
			i++;
			j--;
		}
	}
	if (j > low)
		quickSort(array, low, j);
	if (i < high)
		quickSort(array, i, high);
}*/


unsigned int pivot (std::vector<cv::Vec4i> & v, unsigned int start,
	unsigned int stop, unsigned int position)
	// partition vector into two groups
	// values smaller than or equal to pivot
	// values larger than pivot
	// return location of pivot element
{
		// swap pivot into starting position
	std::swap (v[start], v[position]);

		// partition values
	unsigned int low = start + 1;
	unsigned int high = stop;
	while (low < high)
		if (v[low][0] < v[start][0])
			low++;
		else if (v[--high][0] < v[start][0])
			std::swap (v[low], v[high]);

		// then swap pivot back into place
	std::swap (v[start], v[--low]);
	return low;
}

void quickSort(std::vector<cv::Vec4i> & v, unsigned int low, unsigned int high)
{
	// no need to sort a vector of zero or one elements
	if (low >= high)
		return;

	// select the pivot value
	unsigned int pivotIndex = (low + high) / 2;

	// partition the vector
	pivotIndex = pivot (v, low, high, pivotIndex);

	// sort the two sub vectors
	if (low < pivotIndex)
		quickSort(v, low, pivotIndex);
	if (pivotIndex < high)
		quickSort(v, pivotIndex + 1, high);
}



