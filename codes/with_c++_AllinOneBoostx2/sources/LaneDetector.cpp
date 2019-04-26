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
#include "./headers/LaneDetector.hpp"
#include "./headers/ArduinoComm.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/videoio/legacy/constants_c.h"
#include <thread>

/*******/
#include <time.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
//#include <wiringPi.h>
//#include <wiringSerial.h>
/*******/

// CONSTRUCTOR
LaneDetector::LaneDetector() {
    right_m = 1;
    left_m = 1;
    img_rows = 1;
    img_cols = 600;

    img_leftBound = 0;
    img_rightBound = 600;
    img_center = 300;

    ROInterest_high = 225;
    ROInterest_low = 25;
    ROTarget_high = 125;
    ROTarget_low = 25;

    /*processNoise.create(4, 1, CV_32F);
    measurement.create(2,1);
    state.create(4,1);
    initKalman(0, 0);*/

    angle_estimation = 0;
    alfa_estimation = 0.875;
    angle_tolerance = 20;
}

// UPDATE ANGLE BOUND
/**
 *@brief There should be an interval for a turn angle to be valid.
 *@brief This function sets that base angle prediction.
 *@brief The function give weight to current measurement and previous
 *@brief prediction and sets the next prediction.
 *@param beta = measured angle
 */
void LaneDetector::updateAngleBound(float beta) {
    if(beta > 180)
        beta = beta-360;
    /** DEBUG PRINT BELOW **/
    /*
    std::ofstream myFile;
    myFile.open("testFilter.txt", std::ios_base::app);
    myFile << beta << " " << angle_estimation << "\n" ;
    std::cout << "measured beta: " << beta << " ";
    std::cout << "alfa_estimation: " << alfa_estimation << " ";
    std::cout << "angle_estimation: " << angle_estimation << std::endl;
    * */
    angle_estimation = (alfa_estimation * angle_estimation) + ((1 - alfa_estimation) * beta);
    return;
}

void LaneDetector::updateROVariables(int distance) {
    distance = distance / 10; // convert to cm
    if(distance >= 14) {
        ROInterest_high = 225;
        ROInterest_low = 25;
        ROTarget_high = 175;
        ROTarget_low = 50;
        }
    else if ((distance < 14) && (distance > 3)) {
        ROInterest_high = 14 * distance + 43;
        ROTarget_high = ROTarget_low + (ROInterest_high - ROInterest_low) / 2;
        }
    else {
        ROInterest_high = 10 * distance + 55;
        ROTarget_high = ROTarget_low + (ROInterest_high - ROInterest_low) / 2;
        }
    }

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
    cv::Point ptsROI[4] = { cv::Point(0, img_edges.rows - ROInterest_high), cv::Point(
                                img_edges.cols, img_edges.rows - ROInterest_high), cv::Point(img_edges.cols,
                                        img_edges.rows - ROInterest_low), cv::Point(0, img_edges.rows - ROInterest_low)
                          };

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

// FIND LINE BORDERS
/**
 *@brief This function finds the lane border coordinates in x-axis.
 *@param img is the input HSV or LAB filtered image containing only white or black pixels.
 *@return nothing
 */
void LaneDetector::setLineBorders(cv::Mat img) {

    const int width = img.cols; // width of the image
    const int confidenceCount = 190; // minimum white pixel count in a column
    // 250 is the row count of ROI

    int* whitePixels = new int[width](); // create the dynamic array initialized to zero
    /*
    for (int x = 0; x < width; x++) {
        whitePixels[x] = 0;
        for (int y = 0; y < img.rows; y++) {
            //cv::Mat::at(int row, int col)

            if (img.at<uchar>(y, x) == 255) {
                whitePixels[x]++;
            }
        }
    }*/
    uint8_t *myData = img.data;
    int cols = img.cols;
    int rows = img.rows;
    int _stride = img.step;//in case cols != strides
    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; j++) {
            if(myData[ i * _stride + j] == 255) {
                whitePixels[cols]++;
            }
            //do whatever you want with your value
        }
    }

    // find left bound
    for (int i = 0; i < width; i++) {
        //std::cout << "confidenceCount: " << confidenceCount << " " << whitePixels[i]  << std::endl;
        if (whitePixels[i] > confidenceCount) {
            img_leftBound = i;
            break;
        }
    }
    // find right bound
    for (int i = width - 1; i >= 0; i--) {
        //std::cout << "confidenceCount: " << confidenceCount << " " << whitePixels[i]  << std::endl;
        if (whitePixels[i] > confidenceCount) {
            img_rightBound = i;
            break;
        }
    }

    img_center = (img_leftBound + img_rightBound) / 2;
    /** DEBUG PRINT BELOW **/
    /*
    std::cout << "left bound: " << img_leftBound << " right bound: "
    		<< img_rightBound << std::endl;
    std::cout << "img center: " << img_center << std::endl;
    */
    delete [] whitePixels;
    return;
}

void LaneDetector::saveBadIndex(unsigned int index, int& base_i1,
                                int& base_i2) {
    if (base_i1 == -1 && base_i2 == -1) {
        base_i1 = index;
    }

    else if (base_i1 != -1 && base_i2 == -1) {
        base_i2 = index;
    }

    else if (base_i2 != -1 && (index + 1) > base_i2) {
        base_i2 = index;
    }

    return;
}

// REMOVE THE BAD LINES
/**
 *@brief Lines with bad indices are determined.
 *@brief This function deletes the lines that are between bad indices
 *@param lines = line vector containing all lines together bads and goods
 *@param angles = the angles of the respective lines
 *@param i1 = bad index 1
 *@param i2 = bad index 2
 */
void LaneDetector::removeBadLines(std::vector<cv::Vec4i>& lines,
                                  std::vector<double>& angles, int& i1, int& i2) {
    /** DEBUG PRINT BELOW **/
    /*
    for (auto i : lines)
    	std::cout << i << " ";
    std::cout << std::endl;*/

    for (int i = i2; i >= i1; i--) {
        lines.erase(lines.begin() + i);
        angles.erase(angles.begin() + i);
    }

    /** DEBUG PRINT BELOW **/
    /*
    for (auto i : lines)
    	std::cout << i << " ";
    std::cout << std::endl;*/
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
                               std::vector<double>& angles, std::vector<double>& angles_diff,
                               int& index1, int& index2) {

    double ang_thres_low = 10;
    double ang_thres_high = 88; //was 84
    double ang_diff_thres_low = 16; // 10
    double ang_diff_thres_high = 100; // 50 --> -50
    double ang_diff_thres_avg = 15;
    double sign_change_thres = -12;

    unsigned int current_index = 0;

    /** DEBUG PRINT BELOW **/
    /*
    for (auto i : angles)
    	std::cout << i << " ";
    std::cout << std::endl;*/

    for (auto &i : angles_diff) {
        // find the current_index
        if (&i == &angles_diff[0])
            current_index = 0;
        else
            current_index++;
        double angle_mul = angles[current_index + 1] * angles[current_index];
        // spot if second derivative changed abruptly
        if ((std::abs(i) > ang_diff_thres_low)) {
            /** DEBUG PRINT BELOW **/
            /*
            std::cout << "currentIndex: " << current_index << " criteria 1"
            		<< std::endl;*/
            // if there is a big slope angle change,
            // spot the problem
            // problem is at the current_index or at the current_index+1
            if (angles[current_index + 1] * angles[current_index]
                    < sign_change_thres) {
                /** DEBUG PRINT BELOW **/
                /*
                std::cout << "criteria 1.1" << std::endl;*/
                saveBadIndex(current_index, index1, index2);

            } else if (((std::abs(angles[current_index + 1])) < ang_thres_low)
                       || ((std::abs(angles[current_index + 1])) > ang_thres_high)) {
                /** DEBUG PRINT BELOW **/
                /*
                std::cout << "criteria 1.2" << std::endl;*/
                saveBadIndex(current_index, index1, index2);
            } else if (((std::abs(angles[current_index])) < ang_thres_low)
                       || ((std::abs(angles[current_index])) > ang_thres_high)) {
                /** DEBUG PRINT BELOW **/
                /*
                std::cout << "criteria 1.3" << std::endl;*/
                saveBadIndex(current_index, index1, index2);

            } else if ((std::abs(i) > ang_diff_thres_avg)) {
                /** DEBUG PRINT BELOW **/
                /*
                std::cout << "criteria 1.4" << std::endl;*/
                saveBadIndex(current_index, index1, index2);
            }
            //std::cin.get();
        } else if ((angle_mul < sign_change_thres)
                   && (std::abs(i) < ang_diff_thres_high)) {
            /** DEBUG PRINT BELOW **/
            /*
            std::cout << "currentIndex: " << current_index << " criteria 2"
            		<< std::endl;*/
            saveBadIndex(current_index, index1, index2);
        }

    }
    if (index1 >= -1 && index2 >= -1)
        /** DEBUG PRINT BELOW **/
        /*
        	std::cout << "b1: " << index1 << " b2: " << index2 << std::endl;*/

        if (index1 != -1 && index2 != -1) {
            // clean out problematic indexes
            removeBadLines(lines, angles, index1, index2);
        }

    // if there are no bad indexes or two bad indexes
    // remove unnecessary lines
    if (!(index1 != -1 && index2 == -1)) {
        int max = lines.size() - 1;
        int min = lines.size() / 2;
        removeBadLines(lines, angles, min, max);
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

    // variables to save spotted problematic indexes
    int l_bad_index_1 = -1;
    int l_bad_index_2 = -1;
    int r_bad_index_1 = -1;
    int r_bad_index_2 = -1;

    // initial and final points of a line
    cv::Point ini;
    cv::Point fini;
    // vectors to hold slopes of right and left lines

    // variables to store slopes and slope derivatives
    std::vector<double> right_slopes, left_slopes;
    std::vector<double> right_slopes_dx, left_slopes_dx;

    // variables to store slopes and slope derivatives
    std::vector<double> r_slope_ang, l_slope_ang;
    std::vector<double> r_slope_ang_diff, l_slope_ang_diff;

    // variables to store line coordinates
    std::vector<cv::Vec4i> right_lines, left_lines;

    std::vector<cv::Vec4i> selected_lines;

    double ang_diff = 0.1; // stores line slope difference
    double l_slope_avg = 0;
    double r_slope_avg = 0;

//	// find the center of the lines
//	for (auto i : lines) {
//		ini = cv::Point(i[0], i[1]);
//		fini = cv::Point(i[2], i[3]);
//
//		if (ini.x + fini.x > img_edges.cols) {
//			sum_x_right += ini.x + fini.x;
//			counter_x_right += 2;
//		} else {
//			sum_x_left += ini.x + fini.x;
//			counter_x_left += 2;
//		}
//
//	}
//	std::cout << sum_x_right << std::endl;
//	findLineCenter(counter_x_left, sum_x_left, counter_x_right, sum_x_right);

    // Calculate the slope of all the detected lines
    // Forms the slope derivative vector at the same time
    for (auto i : lines) {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);
        if (std::abs(ini.y - fini.y) <= 3) {
            continue;
        }
        double delta_y = (static_cast<double>(fini.y)
                          - static_cast<double>(ini.y));
        double delta_x = (static_cast<double>(fini.x)
                          - static_cast<double>(ini.x) + 0.00001);

        // Basic algebra: slope = (y1 - y0)/(x1 - x0)
        double m_angle = (cv::fastAtan2(delta_y, delta_x));
        if (m_angle > 269)
            m_angle = m_angle - 180;

        //std::cout << "delta_y: "<< delta_y << "delta_x: " << delta_x << "m: " << slope << " m_angle: " << m_angle << std::endl;

        // If the slope is too horizontal, discard the line
        // If not, save them  and their respective slope
        //if (std::abs(slope) > slope_thresh) {

        //slopes.push_back(slope);
        //selected_lines.push_back(i);

        //std::cout << i << " ";

        // CHECK HERE BACK!!!!!!! 30.03.2019
        if (((ini.x + fini.x) > 2 * sum_x_left)
                && ((ini.x + fini.x) < 2 * sum_x_right)
                && (std::abs((ini.x + fini.x) - (2 * img_center)) < 200)) {
            // discard the line
            // std::cout << "discarded" << std::endl;
        }

        // add to determine mass point in x axis
        else if (ini.x + fini.x > 2 * img_center) {
            right_lines.push_back(i); // classify as right line

            // SLOPE IMPLEMENTATION
            /*			// if a new slope is to be added, find the difference
             // between two slopes and save the difference
             if (!right_slopes.empty()) {
             diff = slope - right_slopes.back();
             right_slopes_dx.push_back(diff);
             }
             right_slopes.push_back(slope);*/
            if (!r_slope_ang.empty()) {
                ang_diff = m_angle - r_slope_ang.back();
                r_slope_ang_diff.push_back(ang_diff);
            }
            r_slope_ang.push_back(m_angle);
        } else {

            left_lines.push_back(i); // classify as left line

            // SLOPE IMPLEMENTATION
            /*			// if a new slope is to be added, find the difference
             // between two slopes and save the difference
             if (!left_slopes.empty()) {
             diff = slope - left_slopes.back();
             left_slopes_dx.push_back(diff);
             }
             left_slopes.push_back(slope);*/
            if (!l_slope_ang.empty()) {
                ang_diff = m_angle - l_slope_ang.back();
                l_slope_ang_diff.push_back(ang_diff);
            }
            l_slope_ang.push_back(m_angle);
        }

        //}
    }
    /** DEBUG PRINT BELOW **/
    /*
    std::cout << "right size: " << right_lines.size() << std::endl;*/
    if (right_lines.size() == 0) {
        //right_lines.push_back(cv::Vec4i(img_leftBound + 248, img_edges.rows - 300, img_leftBound + 248, img_edges.rows - 50));
        right_lines.emplace_back(img_leftBound + 348, img_edges.rows - 300,
                                 img_leftBound + 348, img_edges.rows - 50);
    }
    if (left_lines.size() == 0) {
        //left_lines.push_back(cv::Vec4i(img_rightBound - 248, img_edges.rows - 300, img_rightBound - 248, img_edges.rows - 50));
        left_lines.emplace_back(img_rightBound - 348, img_edges.rows - 300,
                                img_rightBound - 348, img_edges.rows - 50); //
    }
    /** DEBUG PRINT BELOW **/
    /*
    std::cout << "right size: " << right_lines.size() << std::endl;

    std::cout << "leftLines" << std::endl;
    for (auto i : left_lines)
    	std::cout << i << " ";
    std::cout << std::endl;
    for (auto i : right_lines)
    	std::cout << i << " ";
    std::cout << std::endl;*/

    // fix the problems if any for both lines
    if (left_lines.size() > 1) {
        fixProblems(left_lines, l_slope_ang, l_slope_ang_diff, l_bad_index_1,
                    l_bad_index_2);
    } else {
        l_bad_index_1 = -1;
        l_bad_index_2 = -1;
    }

    if (right_lines.size() > 1) {
        fixProblems(right_lines, r_slope_ang, r_slope_ang_diff, r_bad_index_1,
                    r_bad_index_2);
    } else {
        r_bad_index_1 = -1;
        r_bad_index_2 = -1;
    }

    if ((l_bad_index_1 != -1 && l_bad_index_2 == -1)
            && !(r_bad_index_1 != -1 && r_bad_index_2 == -1)) {
        //std::cout << "------------l has 1 prob" << std::endl;
        int targetIndex = static_cast<int>(left_lines.size()) - 1;
        for (auto i : r_slope_ang)
            r_slope_avg += i;
        r_slope_avg = r_slope_avg / r_slope_ang.size();

        double current_index_diff = std::abs(
                                        std::abs(r_slope_avg) - std::abs(l_slope_ang[l_bad_index_1]));
        double next_index_diff = 0;
        if (l_bad_index_1 != 0)
            next_index_diff = std::abs(
                                  std::abs(r_slope_avg)
                                  - std::abs(l_slope_ang[l_bad_index_1 + 1]));
        /** DEBUG PRINT BELOW **/
        /*
        std::cout << "LnextD: " << next_index_diff << "LcurrD: "
        		<< current_index_diff << std::endl;*/
        if (current_index_diff < next_index_diff) {
            /** DEBUG PRINT BELOW **/
            /*
            std::cout << "save left" << std::endl;*/
            removeBadLines(left_lines, l_slope_ang, l_bad_index_1, targetIndex);
        } else {
            targetIndex = 0;
            /** DEBUG PRINT BELOW **/
            /*
            std::cout << "save right" << std::endl;*/
            removeBadLines(left_lines, l_slope_ang, targetIndex, l_bad_index_1);
        }
    } else if ((r_bad_index_1 != -1 && r_bad_index_2 == -1)
               && !(l_bad_index_1 != -1 && l_bad_index_2 == -1)) {
        //std::cout << "------------r has 1 prob" << std::endl;
        int targetIndex = static_cast<int>(right_lines.size()) - 1;
        for (auto i : l_slope_ang)
            l_slope_avg += i;

        l_slope_avg = l_slope_avg / l_slope_ang.size();

        double current_index_diff = std::abs(
                                        std::abs(l_slope_avg) - std::abs(r_slope_ang[r_bad_index_1]));
        double next_index_diff = 0;
        if (r_bad_index_1 != 0)
            next_index_diff = std::abs(
                                  std::abs(l_slope_avg)
                                  - std::abs(r_slope_ang[r_bad_index_1 + 1]));
        /** DEBUG PRINT BELOW **/
        /*
        std::cout << "RnextD: " << next_index_diff << "RcurrD: "
        		<< current_index_diff << std::endl;*/
        if (current_index_diff < next_index_diff) {
            /** DEBUG PRINT BELOW **/
            /*
            std::cout << "save left" << std::endl;*/
            removeBadLines(right_lines, r_slope_ang, r_bad_index_1,
                           targetIndex);
        } else {
            targetIndex = 0;
            /** DEBUG PRINT BELOW **/
            /*
            std::cout << "save right" << std::endl;*/
            removeBadLines(right_lines, r_slope_ang, targetIndex,
                           r_bad_index_1);
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
    int ini_y = img_rows - ROTarget_low;
    int fini_y = img_rows - ROTarget_high;

    double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
    double right_fin_x = ((fini_y - right_b.y) / right_m) + right_b.x;

    double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
    double left_fin_x = ((fini_y - left_b.y) / left_m) + left_b.x;

    output[0] = cv::Point(right_ini_x, ini_y);
    output[1] = cv::Point(right_fin_x, fini_y);
    output[2] = cv::Point(left_ini_x, ini_y);
    output[3] = cv::Point(left_fin_x, fini_y);

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

    // The vanishing points location determines where is the road turning
    /** DEBUG PRINT BELOW **/
    /*
    std::cout << angle << std::endl;*/
    if (angle < 360 - 3 && angle > 100)
        output = "Turn Left";
    else if (angle > 3 && angle < 100)
        output = "Turn Right";
    else
        output = "Go Straight";

    if (angle > 180) {
        angle = angle - 360; // adjust for left/right turn
        // subtract from 360 for left turns
    } else {

    }
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
    double current_x = img_cols / 2;
    double target_x = (lane[1].x + lane[3].x) / 2;
    int turnAngle = cv::fastAtan2(-(current_x - target_x), (ROTarget_high - ROTarget_low)); // gives the angle
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
    cv::line(inputImage, cv::Point(target_x, img_rows - ROTarget_high),
             cv::Point(current_x, inputImage.rows), cv::Scalar(0, 255, 255), 5,
             CV_AA);
    cv::line(inputImage, lane[0], lane[1], cv::Scalar(164, 76, 20), 5, CV_AA);
    cv::line(inputImage, lane[2], lane[3], cv::Scalar(164, 76, 20), 5, CV_AA);

    // plot the turn message with DUAYENLER red
    cv::putText(inputImage, turn,
                cv::Point(inputImage.cols / 3, inputImage.rows - 50),
                cv::FONT_HERSHEY_DUPLEX, 1, cvScalar(67, 32, 206), 1, CV_AA);


    //cv::Point ini = cv::Point(lane[0].x, lane[1].x);
    //cv::Point fini = cv::Point(lane[2].x, lane[3].x);
    /** DEBUG PRINT BELOW **/
    /*
    std::cout << lane[0] << " " << lane[1] << " " << lane[2] << " " << lane[3] << std::endl;*/
    float downCenter = static_cast<float>(lane[0].x + lane[2].x) /2;
    float upCenter = static_cast<float>(lane[1].x + lane[3].x) /2;

    float y1 = downCenter - 320;
    float y2 = upCenter - 320;
    float beta = cv::fastAtan2(upCenter - downCenter, (ROTarget_high - ROTarget_low));
    updateAngleBound(beta); // update estimation

    /***************KALMAN************/
//	cv::Point sKalman, pKalman;
//	pKalman = kalmanPredict();
    /** DEBUG PRINT BELOW **/
    /*
    std::cout << "kalman prediction: " << pKalman.x << " " << pKalman.y << std::endl;*/

//	if(beta>180)
//		sKalman = kalmanCorrect(y1,beta-360);
//    else
//		sKalman = kalmanCorrect(y1,beta);
    /** DEBUG PRINT BELOW **/
    /*
    std::cout << "kalman corrected state: " << sKalman.x << " " << sKalman.y << std::endl;*/
//    kalmanUpdate();
//   cv::putText(inputImage,  std::to_string(pKalman.x),
//			cv::Point(inputImage.cols / 2, inputImage.rows - 100),
//			cv::FONT_HERSHEY_DUPLEX, 1, cvScalar(67, 32, 206), 1, CV_AA);
    /************KALMAN***************/
    /*
    std::ofstream myFile;
    myFile.open("test.txt", std::ios_base::app);
    myFile << y1 << " " << pKalman.x << " " ;
    if(beta>180)
    	myFile << beta-360 << " " ;
    else
    	myFile << beta << " " ;

    if (pKalman.y > 180)
    myFile << pKalman.y -360 << std::endl;
    else
    	myFile << pKalman.y << std::endl;
    myFile.close();*/

    /***** SENDING BETA ********/
    //beta = sKalman.y;
    arduinoComm.sendToController("B", std::to_string(beta));

    /***** SENDING Y1 *******/
    //y1 = pKalman.x;
    //std::string s3;

    if(y1 > 0) {
        arduinoComm.sendToController("P", std::to_string(y1));
    } else {
        arduinoComm.sendToController("N", std::to_string(std::abs(y1)));
    }
    /**************************/

    /***** SENDING Y2 *******/
    //std::string s2;
    if(y2 > 0) {
        arduinoComm.sendToController("R", std::to_string(y2));
    } else {
        arduinoComm.sendToController("L", std::to_string(std::abs(y2)));
    }


    // show the final output image
    //cv::namedWindow(window_vision);
    //cv::imshow(window_vision, inputImage);
    return 0;
}

/*
cv::Point LaneDetector::kalmanPredict() {
	cv::Mat prediction = KF.predict();
	cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
    return predictPt;
}

cv::Point LaneDetector::kalmanCorrect(float x, float y) {
    measurement(0) = x;
    measurement(1) = y;
    cv::Mat estimated = KF.correct(measurement);
    cv::Point statePt(estimated.at<float>(0),estimated.at<float>(1));
    return statePt;
}

void LaneDetector::kalmanUpdate() {
    cv::randn( processNoise, cv::Scalar(0), cv::Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
    state = KF.transitionMatrix*state + processNoise;
}

void LaneDetector::initKalman(float x, float y) {
    // Instantate Kalman Filter with
    // 4 dynamic parameters and 2 measurement parameters,
    // where my measurement is: 2D location of object,
    // and dynamic is: 2D location and 2D velocity.
    KF.init(4, 2, 0);

    measurement = cv::Mat_<float>::zeros(2,1);
    measurement.at<float>(0, 0) = x;
    measurement.at<float>(0, 0) = y;


    KF.statePre.setTo(0);
    KF.statePre.at<float>(0, 0) = x;
    KF.statePre.at<float>(1, 0) = y;

    KF.statePost.setTo(0);
    KF.statePost.at<float>(0, 0) = x;
    KF.statePost.at<float>(1, 0) = y;

    setIdentity(KF.transitionMatrix);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, cv::Scalar::all(.5)); //adjust this for faster convergence - but higher noise
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(0.1));
    setIdentity(KF.errorCovPost, cv::Scalar::all(1));
}*/
