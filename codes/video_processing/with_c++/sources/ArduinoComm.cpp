/*Copyright (c) 2019 DUAYENLER Ltd. Sti
 */
/*
 *@copyright Copyright 2019 DUAYENLER Ltd. Sti
 *@file ArduinoComm.cpp
 *@author Erdem Tuna
 *@brief Definition of all the function that form part of the ArduinoComm class.
 *@brief The class will take a string input and sen it to Arduino via USB serial.
 */

/*******/
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include "./headers/wiringPi.h"
#include "./headers/wiringSerial.h"
#include "./headers/ArduinoComm.hpp"


ArduinoComm::ArduinoComm() {
	serialDeviceId = serialOpen("/dev/ttyUSB0", 9600);
}
// SEND TO CONTROLLER
/**
 *@brief Sends the input payload to Arduino.
 *@param payload
 *@return
 */
void ArduinoComm::sendToController(std::string payload) {
	/*************************/
	//int serialDeviceId = 0;
	//serialDeviceId = serialOpen("/dev/ttyUSB0", 9600);
	std::cout << "sender " << serialDeviceId << std::endl;
	if (serialDeviceId == -1) {
		std::cout << "Unable to open serial device" << std::endl;
		return;
	}
	if (wiringPiSetup() == -1) {
		return;
	}
	serialPuts(serialDeviceId, payload.c_str());
	return;
}
