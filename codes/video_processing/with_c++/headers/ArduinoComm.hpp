/*Copyright (c) 2018 DUAYENLER Ltd. Sti
 */
/*
 *@copyright Copyright 2018 DUAYENLER Ltd. Sti
 *@file AarduinoComm.cpp
 *@author Erdem Tuna
 *@brief Header file for the ArduinoComm class. Functions are developed in ArduinoComm.cpp
 */

#pragma once
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

/**
 *@brief Definition of the LaneDetector class. It contains all the functions and variables depicted in the
 *@brief Activity diagram and UML Class diagram.
 *@brief It detects the lanes in an image if a highway and outputs the
 *@brief same image with the plotted lane.
 */
class ArduinoComm {
private:
	int serialDeviceId;
public:
	ArduinoComm();
	void sendToController(std::string payload);
};
