/*Copyright (c) 2019 DUAYENLER Ltd. Sti
 */
/*
 *@copyright Copyright 2019 DUAYENLER Ltd. Sti
 *@file LaneDetector.cpp
 *@author Erdem Tuna
 *@brief Header file for the LaneDetector class. Functions are developed in LaneDetector.cpp
 */

#pragma once
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include <chrono>
#include "./headers/ArduinoComm.hpp"

/**
 *@brief Definition of the LaneDetector class. It contains all the functions and variables depicted in the
 *@brief Activity diagram and UML Class diagram.
 *@brief It detects the lanes in an image if a highway and outputs the
 *@brief same image with the plotted lane.
 */
class LaneDetector {
  private:
    ArduinoComm arduinoComm;
    int img_rows;
    int img_cols;
    double img_center;
    double img_leftBound;
    double img_rightBound;
    double timeCapture = (double) cv::getTickCount();

    // estimation variable
    float angle_estimation;
    float angle_tolerance;
    float alfa_estimation;

    // ROI, ROT variables
    int ROInterest_high;
    int ROInterest_low;
    int ROTarget_high;
    int ROTarget_low;

    /******KALMAN VARIABLES *////
    cv::Mat processNoise;
    cv::KalmanFilter KF;
    cv::Mat_<float> measurement;
    cv::Mat_<float> state;

    void updateAngleBound(float beta);

    cv::Point ptsROI[4]; // points classifying the ROI
    bool left_flag = false; // Tells us if there's left boundary of lane detected
    bool right_flag = false; // Tells us if there's right boundary of lane detected
    cv::Point right_b; // Members of both line equations of the lane boundaries:
    double right_m;  // y = m*x + b
    cv::Point left_b;  //
    double left_m;  //
    const cv::String window_vision = "Lane and Vehicle Vision";

    void fixProblems(std::vector<cv::Vec4i>& lines, std::vector<double>& angles,
                     std::vector<double>& angles_diff, int& index1, int& index2); // fixes problematic lines
    void removeBadLines(std::vector<cv::Vec4i>& lines, std::vector<double>& angles, int& i1, int& i2);
    void saveBadIndex(unsigned int index, int& base_i1, int& base_i2); // saves bad lines indices

    // Kalman related functions
    cv::Point kalmanPredict();
    cv::Point kalmanCorrect(float x, float y);
    void kalmanUpdate ();
    void initKalman(float x, float y);

  public:
    LaneDetector();
    void updateROVariables(int distance);
    void updateRegionBorders(float distance);
    void setLineBorders(cv::Mat img);  // Sets line border pixels
    cv::Mat deNoise(cv::Mat inputImage); // Apply Gaussian blurring to the input Image
    cv::Mat edgeDetector(cv::Mat img_noise); // Filter the image to obtain only edges
    cv::Mat cropROI(cv::Mat img_edges); // Mask the edges image to only care about ROI
    std::vector<cv::Vec4i> houghLines(cv::Mat img_mask); // Detect Hough lines in masked edges image

    std::vector<std::vector<cv::Vec4i> > lineSeparation(
        std::vector<cv::Vec4i> lines, cv::Mat img_edges); // Separate detected lines by their slope into right and left lines
    std::vector<cv::Point> regression(
        std::vector<std::vector<cv::Vec4i> > left_right_lines,
        cv::Mat inputImage);  // Get only one line for each side of the lane
    std::string predictTurn(double pivot, int angle); // Determine if the lane is turning or not by calculating the position of the vanishing point
    int plotLane(cv::Mat& inputImage, std::vector<cv::Point> lane); // Plot the resultant lane and turn prediction in the frame.
};
