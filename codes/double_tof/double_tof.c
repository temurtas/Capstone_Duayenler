#include "vl6180_pi/vl6180_pi.h"
#include <stdio.h>
#include <wiringPi.h>
#define sensor1Pin 0
#define sensor2Pin 1

int main(){

	if(wiringPiSetup() == -1) {
		printf("setup wiringPi failed !\n");
		return -1;
	}
	
	pinMode(sensor1Pin, OUTPUT);
	pinMode(sensor2Pin, OUTPUT);
	digitalWrite(sensor1Pin,LOW);
	digitalWrite(sensor2Pin,LOW);
	
	digitalWrite(sensor1Pin,HIGH);
	delay(50);
	vl6180 fhandle = vl6180_initialise(1);
	if(fhandle<=0){
		printf("ERROR FOR FRONT HANDLE !\n");
		return -1;
	}
	vl6180_change_addr(fhandle,0x54);
	
	digitalWrite(sensor2Pin,HIGH);
	delay(50);
	vl6180 bhandle = vl6180_initialise(1);
	if(fhandle<=0){
		printf("ERROR FOR BACK HANDLE !\n");
		return -1;
	}
	vl6180_change_addr(bhandle,0x56);

	int  fdistance, bdistance;
	
	while (1){
		fdistance = get_distance(fhandle);
		bdistance = get_distance(bhandle);

		printf("Front distance at scaling 1 is %d mm\n", fdistance);
		delay(500);
		printf("Back distance at scaling 1 is %d mm\n", bdistance);
		delay(500);
	}
		
	return 0;
}
