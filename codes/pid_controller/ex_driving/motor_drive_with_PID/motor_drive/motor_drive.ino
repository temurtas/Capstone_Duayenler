//EE493 Capstone Project Arduino code
//All rights are reserved
// DUAYENLER Ltd. Şti

#include <SerialCommand.h>
SerialCommand scmd;
#define NOP __asm__ __volatile__ ("nop\n\t")
#define left_motor_back 6
#define left_motor_forward 7
#define right_motor_forward 8
#define right_motor_back 9
#define left_motor_pwm 5
#define right_motor_pwm 10

// Variable Declerations
unsigned long deltaT = 1000; // time between samples (usecs) 1000->50000
const int pastSize = 3; // used for integrating

float duration = 0; // The duration between two loops
double p_time = 0; // A variable to hold past time

float pastError[pastSize]; //Holds the past errors
float sum; //Holds the summation of the errors

int dacMax = 255; // Arduino dac is eight bits.
int adcMax = 1024; //Arduino uses a 10 bit dac, 10th power of 2 = 1024
int adcCenter = 512; // 1024/2

int pwm_offset = 0; // Direct motor command from host.
int pwm_corr = 12;
int desired = 0; // Desired value (speed or angle) from host.

float Kp = 0; // Feedback gains (proportional)
float Kd = 0; // Feedback gains (delta)
float Ki = 0; // Feedback gains (sum)

float maxSum = 1; // Optional integral term
unsigned long delta; //Calculates the derivative of the error
char *arg;

unsigned long t; //time variable
int incoming[12]; // array that holds the variabels coming from GUI
int ang = 0;



void setup() {
  // Pin initialization
  pinMode(left_motor_back, OUTPUT);
  pinMode(left_motor_forward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_back, OUTPUT);
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(right_motor_pwm, OUTPUT);
  Serial.begin(9600);
  digitalWrite(left_motor_back, LOW);
  digitalWrite(left_motor_forward, LOW);
  digitalWrite(right_motor_back, LOW);
  digitalWrite(right_motor_forward, LOW);

  scmd.addCommand("A",getangle);
  scmd.addDefaultHandler(defaultt);
}

void getangle(){
  char *arg;
  arg=scmd.next();
  if(arg !=NULL){
    ang=atoi(arg);
  }
} 

void defaultt(){
   NOP;
}


void loop() {
  t = millis();
  duration = t - p_time;
  p_time = t;
  

  //scang = 75;
  
  //Assignment of each variable.

  Kp = 3; // Feedback gains (proportional)
  Kd = 0; // Feedback gains (delta)
  Ki = 0; // Feedback gains (sum)
  maxSum = 0; // Optional integral term
  pwm_offset = 60;
  
  //Update pastError
  for (int i = pastSize - 1; i > 0; i--) {
    pastError[i] = pastError[i - 1];
  }
  
  // Calculate the error and assign the error as the recent error
  float error = ang - desired;
  pastError[0] = error;

  // Calculate the derivative for the derivative controller and calculate the sum for integral controller
  float delta = (pastError[0] - pastError[pastSize - 1]) / (pastSize * float(duration) * 1e-3);
  sum = min(max(sum + error * float(deltaT) * 1e-6, -1 * maxSum), maxSum);


  // Compute and limit the motor output command.
  int motorCmd = int(Kp * error + Ki * sum + Kd * delta); //
  motorCmd = abs(motorCmd);
  
  if (error >= 0) {
    analogWrite(left_motor_pwm, pwm_offset + pwm_corr);
    analogWrite(right_motor_pwm, min(pwm_offset + motorCmd, dacMax) );
    initial_start();

  } else {
    analogWrite(right_motor_pwm, pwm_offset);
    analogWrite(left_motor_pwm, min(pwm_offset + motorCmd + pwm_corr, dacMax) );
    initial_start();

  }



}

void initial_start () {
  digitalWrite(left_motor_back, LOW);
  digitalWrite(left_motor_forward, HIGH);
  digitalWrite(right_motor_forward, HIGH);
  digitalWrite(right_motor_back, LOW);
}
