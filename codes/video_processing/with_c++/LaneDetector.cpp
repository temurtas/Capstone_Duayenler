/*Copyright (c) 2019 DUAYENLER Ltd. Sti
 */
/*
 *@copyright Copyright 2019 DUAYENLER Ltd. Sti
 *@file LaneDetector.cpp
 *@author Erdem Tuna
 *@brief Definition of all the function that form part of the LaneDetector class.
 *@brief The class will take RGB images as inputs and will output the same RGB image but
 *@brief with the plot of the detected lanes and the turn prediction.
 */
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "LaneDetector.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/videoio/legacy/constants_c.h"
#include <thread>

// IMAGE BLURRING
/**
 *@brief Apply gaussian filter to the input image to denoise it
 *@param inputImage is the frame of a video in which the
 *@param lane is going to be detected
 *@return Blurred and denoised image
 */
cv::Mat LaneDetector::deNoise(cv::Mat inputImage) {
	cv::Mat output;

	cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);

	return output;
}

// EDGE DETECTION
/**
 *@brief Detect all the edges in the blurred frame by filtering the image
 *@param img_noise is the previously blurred frame
 *@return Binary image with only the edges represented in white
 */
cv::Mat LaneDetector::edgeDetector(cv::Mat img_noise) {
	cv::Mat output;
	cv::Mat kernel;
	cv::Point anchor;

	// Convert image from RGB to gray
	cv::cvtColor(img_noise, output, cv::COLOR_RGB2GRAY);
	// Binarize gray image
	cv::threshold(output, output, 140, 255, cv::THRESH_BINARY);

	// Create the kernel [-1 0 1]
	// This kernel is based on the one found in the
	// Lane Departure Warning System by Mathworks
	anchor = cv::Point(-1, -1);
	kernel = cv::Mat(1, 3, CV_32F);
	kernel.at<float>(0, 0) = -1;
	kernel.at<float>(0, 1) = 0;
	kernel.at<float>(0, 2) = 1;

	// Filter the binary image to obtain the edges
	cv::filter2D(output, output, -1, kernel, anchor, 0, cv::BORDER_DEFAULT);

	return output;
}

// MASK THE EDGE IMAGE
/**
 *@brief Mask the image so that only the edges that form part of the lane are detected
 *@param img_edges is the edges image from the previous function
 *@return Binary image with only the desired edges being represented
 */
cv::Mat LaneDetector::cropROI(cv::Mat img_edges) {
	cv::Mat output;
	img_rows = img_edges.rows;
	img_cols = img_edges.cols;

	cv::Mat mask = cv::Mat::zeros(img_edges.size(), img_edges.type());
	// a ROI of (img_edges.cols x 250)
	cv::Point ptsROI[4] = { cv::Point(0, img_edges.rows - 300), cv::Point(
			img_edges.cols, img_edges.rows - 300), cv::Point(img_edges.cols,
			img_edges.rows - 50), cv::Point(0, img_edges.rows - 50) };

	// create a binary polygon mask
	cv::fillConvexPoly(mask, ptsROI, 4, cv::Scalar(255, 255, 255));
	// AND the edges image and the mask to get the output
	cv::bitwise_and(img_edges, mask, output);

	return output;
}

// HOUGH LINES
/**
 *@brief Obtain all the line segments in the masked images which are
 *@brief  going to be part of the lane boundaries
 *@param img_mask is the masked binary image from the previous function
 *@return Vector that contains all the detected lines in the image
 */
std::vector<cv::Vec4i> LaneDetector::houghLines(cv::Mat img_mask) {
	std::vector<cv::Vec4i> line;

	// rho and theta are selected by trial and error
	HoughLinesP(img_mask, line, 1, CV_PI / 180, 15, 30, 40);
	return line;
}

// FIND CENTER of the LINES
/**
 *@brief Finds the center of the right and left lines
 *@param counter_x_left and counter_x_right determines how many x-coordinates are used.
 *@param sum_x_left and sum_x_right determines aggregated coordinate value.
 *@return img_center that is a global private variable inside the class
 */
void LaneDetector::findLineCenter(int& counter_x_left, int& sum_x_left,
		int& counter_x_right, int& sum_x_right) {
	if (counter_x_left == 0 && counter_x_right != 0) {
		// prevent division with 0.
		// assume center of lines is 200 pixel away from
		// center of right lines
		sum_x_right = sum_x_right / counter_x_right;
		img_center = sum_x_right - 200;
		return;
	} else if (counter_x_left != 0 && counter_x_right == 0) {
		// prevent division with 0.
		// assume center of lines is 200 pixel away from
		// center of left lines
		sum_x_left = sum_x_left / counter_x_right;
		img_center = sum_x_left + 200;
		return;
	} else if (counter_x_left == 0 && counter_x_right == 0) {
		// prevent division with 0.
		// assume center of lines is 320th pixel
		img_center = 320;
		return;
	} else {
		// calculate center x coordinate
		sum_x_left = sum_x_left / counter_x_left;
		sum_x_right = sum_x_right / counter_x_right;
		std::cout << "left center: " << sum_x_left << " " << "right center: "
				<< sum_x_right << std::endl;
		std::cout << "img_center: " << (sum_x_left + sum_x_right) / 2
				<< std::endl;
		//img_center = static_cast<double>((img_edges.cols / 2));
		img_center = (sum_x_left + sum_x_right) / 2;
		return;
	}
}

void LaneDetector::removeBadLines(std::vector<cv::Vec4i>& lines, int& i1,
		int& i2) {
	for (auto i : lines)
		std::cout << i << " ";
	std::cout << std::endl;

	for (int i = i2; i >= i1; i--) {
		lines.erase(lines.begin() + i);
	}

	for (auto i : lines)
		std::cout << i << " ";
	std::cout << std::endl;
	return;
}

// FIX PROBLEMATIC LINES
/**
 *@brief Removes the problematic lines. (problematic is defined in reports)
 *@brief Fixing is done only if there are 2 problematic index are found.
 *@brief No fixing is realized if there 1 problematic index
 *@param lines is the vector that contains all the detected lines
 *@param slopes is the slopes corresponding to respective line
 *@param slopes_dx is the slope difference between two consecutive slope
 *@return index1 stores first problematic index
 *@return index2 stores second problematic index
 */
void LaneDetector::fixProblems(std::vector<cv::Vec4i>& lines,
		std::vector<double>& slopes, std::vector<double>& slopes_dx,
		int& index1, int& index2) {
	double slopes_dx_thres = 3; // second derivative threshold
	double slopes_thres = 0.5; // first derivative threshold
	unsigned int current_index = 0;

	for (auto i : slopes)
		std::cout << i << " ";
	std::cout << std::endl;

	// iterate for every slope derivative
	for (auto &i : slopes_dx) {
		// find the current_index
		if (&i == &slopes_dx[0])
			current_index = 0;
		else
			current_index = (&i) - (&slopes_dx[0]);

		// spot if second derivative changed abruptly
		if (std::abs(i) > slopes_dx_thres) {
			// if there is a big slope change,
			// spot the problem
			// problem is at the current_index or at the current_index+1
			if ((std::abs(slopes[current_index + 1])) < slopes_thres) {
				if (index1 == -1 && index2 == -1) {
					index1 = current_index + 1;
				}

				else if (index1 != -1 && index2 == -1) {
					index2 = current_index + 1;
				}

				else if (index2 != -1 && (current_index + 1) > index2) {
					index2 = current_index + 1;
				}

			} else if ((std::abs(slopes[current_index])) < slopes_thres) {
				if (index1 == -1 && index2 == -1) {
					index1 = current_index;
				}

				else if (index1 != -1 && index2 == -1) {
					index2 = current_index;
				}

				else if (index2 != -1 && (current_index + 1) > index2) {
					index2 = current_index;
				}

			}
		}
	}
	//std::cout << "b1: " << index1 << " b2: " << index2 << std::endl;

	// if there are 2 problematic indexes, clean between
	if (index1 != -1 && index2 != -1) {
		// clean out problematic indexes
		removeBadLines(lines, index1, index2);
	}
	return;
}

// SORT RIGHT AND LEFT LINES
/**
 *@brief Sort all the detected Hough lines by slope.
 *@brief The lines are classified into right or left depending
 *@brief on the sign of their slope and their approximate location
 *@param lines is the vector that contains all the detected lines sorted from smallest
 *@param starting coordinate to biggest starting coordinate (with quicksort)
 *@param img_edges is used for determining the image center
 *@return The output is a vector(2) that contains all the classified lines
 */
std::vector<std::vector<cv::Vec4i> > LaneDetector::lineSeparation(
		std::vector<cv::Vec4i> lines, cv::Mat img_edges) {
	std::vector<std::vector<cv::Vec4i> > output(2);

	// variables to find center of the image
	int sum_x_right = 0;
	int sum_x_left = 0;
	int counter_x_right = 0;
	int counter_x_left = 0;

	// variables to save spotted problematic indexes
	int bad_index_left1 = -1;
	int bad_index_left2 = -1;
	int bad_index_right1 = -1;
	int bad_index_right2 = -1;

	size_t j = 0;
	// initial and final points of a line
	cv::Point ini;
	cv::Point fini;
	// slope threshold
	double slope_thresh = 0.005;
	// vectors to hold slopes of right and left lines

	// variables to store slopes and slope derivatives
	std::vector<double> right_slopes, left_slopes;
	std::vector<double> right_slopes_dx, left_slopes_dx;

	// variables to store line coordinates
	std::vector<cv::Vec4i> right_lines, left_lines;

	std::vector<cv::Vec4i> selected_lines;

	double diff = 0; // stores line slope difference

	// Calculate the slope of all the detected lines
	// Forms the slope derivative vector at the same time
	for (auto i : lines) {
		ini = cv::Point(i[0], i[1]);
		fini = cv::Point(i[2], i[3]);

		// Basic algebra: slope = (y1 - y0)/(x1 - x0)
		double slope =
				(static_cast<double>(fini.y) - static_cast<double>(ini.y))
						/ (static_cast<double>(fini.x)
								- static_cast<double>(ini.x) + 0.00001);

		// If the slope is too horizontal, discard the line
		// If not, save them  and their respective slope
		//if (std::abs(slope) > slope_thresh) {

		//slopes.push_back(slope);
		//selected_lines.push_back(i);

		// add to determine mass point in x axis
		if (ini.x + fini.x > img_edges.cols) {
			right_lines.push_back(i); // classify as right line
			sum_x_right += ini.x + fini.x;
			counter_x_right += 2;
			// if a new slope is to be added, find the difference
			// between two slopes and save the difference
			if (!right_slopes.empty()) {
				diff = slope - right_slopes.back();
				right_slopes_dx.push_back(diff);
			}
			right_slopes.push_back(slope);
		} else {
			left_lines.push_back(i); // classify as left line
			sum_x_left += ini.x + fini.x;
			counter_x_left += 2;
			// if a new slope is to be added, find the difference
			// between two slopes and save the difference
			if (!left_slopes.empty()) {
				diff = slope - left_slopes.back();
				left_slopes_dx.push_back(diff);
			}
			left_slopes.push_back(slope);
		}

		//}
	}

	// find the center of the lines
	findLineCenter(counter_x_left, sum_x_left, counter_x_right, sum_x_right);

	// fix the problems if any for both lines
	fixProblems(left_lines, left_slopes, left_slopes_dx, bad_index_left1,
			bad_index_left2);
	fixProblems(right_lines, right_slopes, right_slopes_dx, bad_index_right1,
			bad_index_right2);

	/*	if ((bad_index_left1 != -1 && bad_index_left2 == -1)
	 && (bad_index_right1 == -1 && bad_index_right2 == -1)) {*/

	// if there is one problem in left line and there is NOT ONE problem in right line
	if ((bad_index_left1 != -1 && bad_index_left2 == -1)
			&& !(bad_index_right1 != -1 && bad_index_right2 == -1)) {
		int targetIndex = static_cast<int>(left_lines.size()) - 1;
		double current_index_diff = std::abs(right_slopes.back())
				/ std::abs(left_slopes[bad_index_left1]);
		double previous_index_diff = std::abs(right_slopes.back())
				/ std::abs(left_slopes[bad_index_left1 - 1]);

		// if this is true, than previous indexed lines were more parallel to right line
		// remove the rest
		std::cout << "curr: " << current_index_diff << " prev: "
				<< previous_index_diff << std::endl;
		if (current_index_diff > previous_index_diff) {
			//std::cout << "save previous"<< std::endl;
			removeBadLines(left_lines, bad_index_left1, targetIndex);
		} else {
			targetIndex = 0;
			//std::cout << "save rest"<< std::endl;
			removeBadLines(left_lines, targetIndex, bad_index_left1);
		}
	} else if ((bad_index_right1 != -1 && bad_index_right2 == -1)
			&& !(bad_index_left1 != -1 && bad_index_left2 == -1)) {
		int targetIndex = static_cast<int>(left_lines.size()) - 1;
		double current_index_diff = std::abs(left_slopes.back())
				/ std::abs(right_slopes[bad_index_left1]);
		double previous_index_diff = std::abs(left_slopes.back())
				/ std::abs(right_slopes[bad_index_left1 - 1]);

		// if this is true, than previous indexed lines were more parallel to right line
		// remove the rest
		std::cout << "curr: " << current_index_diff << " prev: "
				<< previous_index_diff << std::endl;
		if (current_index_diff > previous_index_diff) {
			//std::cout << "save previous"<< std::endl;
			removeBadLines(right_lines, bad_index_right1, targetIndex);
		} else {
			targetIndex = 0;
			//std::cout << "save rest"<< std::endl;
			removeBadLines(right_lines, targetIndex, bad_index_right1);
		}
	}

	output[0] = right_lines;
	output[1] = left_lines;
	left_flag = true;
	right_flag = true;
	return output;
}

// REGRESSION FOR LEFT AND RIGHT LINES
/**
 *@brief Regression takes all the classified line segments initial and final points and fits a new lines out of them using the method of least squares.
 *@brief This is done for both sides, left and right.
 *@param left_right_lines is the output of the lineSeparation function
 *@param inputImage is used to select where do the lines will end
 *@return output contains the initial and final points of both lane boundary lines
 */
std::vector<cv::Point> LaneDetector::regression(
		std::vector<std::vector<cv::Vec4i> > left_right_lines,
		cv::Mat inputImage) {
	std::vector<cv::Point> output(4);
	cv::Point ini;
	cv::Point fini;
	cv::Point ini2;
	cv::Point fini2;
	cv::Vec4d right_line;
	cv::Vec4d left_line;
	std::vector<cv::Point> right_pts;
	std::vector<cv::Point> left_pts;

	// If right lines are being detected, fit a line using all the init and final points of the lines
	if (right_flag == true) {
		for (auto i : left_right_lines[0]) {
			ini = cv::Point(i[0], i[1]);
			fini = cv::Point(i[2], i[3]);

			right_pts.push_back(ini);
			right_pts.push_back(fini);
		}

		if (right_pts.size() > 0) {
			// The right line is formed here
			cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
			right_m = right_line[1] / right_line[0];
			right_b = cv::Point(right_line[2], right_line[3]);
		}
	}

	// If left lines are being detected, fit a line using all the init and final points of the lines
	if (left_flag == true) {
		for (auto j : left_right_lines[1]) {
			ini2 = cv::Point(j[0], j[1]);
			fini2 = cv::Point(j[2], j[3]);

			left_pts.push_back(ini2);
			left_pts.push_back(fini2);
		}

		if (left_pts.size() > 0) {
			// The left line is formed here
			cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
			left_m = left_line[1] / left_line[0];
			left_b = cv::Point(left_line[2], left_line[3]);
		}
	}

	// One the slope and offset points have been obtained, apply the line equation to obtain the line points
	int ini_y = img_rows - 50;
	int fin_y = img_rows - 175;

	double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
	double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

	double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
	double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;

	output[0] = cv::Point(right_ini_x, ini_y);
	output[1] = cv::Point(right_fin_x, fin_y);
	output[2] = cv::Point(left_ini_x, ini_y);
	output[3] = cv::Point(left_fin_x, fin_y);

	return output;
}

// TURN PREDICTION
/**
 *@brief Predict if the lane is turning left, right or if it is going straight
 *@brief It is done by seeing where the vanishing point is with respect to
 *@brief the center of the detected lines
 *@return String that says if there is left or right turn or if the road is straight
 */
std::string LaneDetector::predictTurn(double pivot, int angle) {
	std::string output;
	double vanish_x = img_center;
	double thr_vp = 10;

/*	// The vanishing point is the point where both lane boundary lines intersect
//	vanish_x = static_cast<double>(((right_m * right_b.x) - (left_m * left_b.x)
//			- right_b.y + left_b.y) / (right_m - left_m));

	// The vanishing points location determines where is the road turning
	if (vanish_x < (pivot - thr_vp))
		output = "Turn Left";
	else if (vanish_x > (pivot + thr_vp))
		output = "Turn Right";
	else if (vanish_x >= (pivot - thr_vp) && vanish_x <= (pivot + thr_vp))
		output = "Go Straight";

	if (angle > 180)
		angle = 360 - angle; // adjust for left/right turn
							 // subtract from 360 for left turns
	output += " " + std::to_string(angle);*/


	// The vanishing point is the point where both lane boundary lines intersect
//	vanish_x = static_cast<double>(((right_m * right_b.x) - (left_m * left_b.x)
//			- right_b.y + left_b.y) / (right_m - left_m));

	// The vanishing points location determines where is the road turning
	std::cout << angle << std::endl;
	if (angle < 360-3 && angle > 100)
		output = "Turn Left";
	else if (angle > 3 && angle < 100)
		output = "Turn Right";
	else
		output = "Go Straight";

	if (angle > 180)
		angle = 360 - angle; // adjust for left/right turn
							 // subtract from 360 for left turns
	output += " " + std::to_string(angle);

	return output;
}

// PLOT RESULTS
/**
 *@brief This function plots both sides of the lane, the turn prediction message and a transparent polygon that covers the area inside the lane boundaries
 *@param inputImage is the original captured frame
 *@param lane is the vector containing the information of both lines
 *@param turn is the output string containing the turn information
 *@return The function returns a 0
 */
int LaneDetector::plotLane(cv::Mat& inputImage, std::vector<cv::Point> lane) {
	std::vector<cv::Point> poly_points;
	cv::Mat output;
	double current_x = img_cols/2;
	double target_x = (lane[1].x + lane[3].x) / 2;
	int turnAngle = cv::fastAtan2(-(current_x - target_x), 125); // gives the angle
	std::string turn = predictTurn(current_x, turnAngle); // prediction text
	// create the transparent polygon for a better visualization of the lane
	inputImage.copyTo(output);
	//std::cout << lane[2] <<" ";
	//std::cout << lane[0] <<" ";
	//std::cout << lane[1] <<" ";
	//std::cout << lane[3] << std::endl;
	poly_points.push_back(lane[2]);
	poly_points.push_back(lane[0]);
	poly_points.push_back(lane[1]);
	poly_points.push_back(lane[3]);

	//std::cout << poly_points <<std::endl;
	cv::fillConvexPoly(output, poly_points, cv::Scalar(255, 0, 0), CV_AA, 0);

	// transparent polygon
	cv::addWeighted(output, 0.3, inputImage, 1.0 - 0.3, 0, inputImage);

	// plot both lines of the lane boundary with DUAYENLER blue
	cv::line(inputImage, cv::Point(target_x, img_rows-175),
			cv::Point(current_x, inputImage.rows), cv::Scalar(0, 255, 255), 5,
			CV_AA);
	cv::line(inputImage, lane[0], lane[1], cv::Scalar(164, 76, 20), 5, CV_AA);
	cv::line(inputImage, lane[2], lane[3], cv::Scalar(164, 76, 20), 5, CV_AA);

	// plot the turn message with DUAYENLER red
	cv::putText(inputImage, turn,
			cv::Point(inputImage.cols / 3, inputImage.rows - 50),
			cv::FONT_HERSHEY_DUPLEX, 1, cvScalar(67, 32, 206), 1, CV_AA);

	// show the final output image
	//cv::namedWindow(window_vision);
	//cv::imshow(window_vision, inputImage);

	return 0;
}

