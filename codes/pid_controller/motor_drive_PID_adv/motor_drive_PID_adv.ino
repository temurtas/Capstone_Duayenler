//EE494 Capstone Project Arduino code
//All rights are reserved
// DUAYENLER Ltd. Şti

#include <SerialCommand.h>
SerialCommand scmd;
#define NOP __asm__ __volatile__ ("nop\n\t")
#define left_motor_back 7
#define left_motor_forward 6
#define right_motor_forward 9
#define right_motor_back 8
#define left_motor_pwm 5
#define right_motor_pwm 10

// Variable Declerations
unsigned long deltaT = 1000; // time between samples (usecs) 1000->50000
const int pastSize = 3; // used for integrating
float duration = 0; // The duration between two loops
double t = 0;
double p_time = 0; // A variable to hold past time
// ********************
float pastError[pastSize]; //Holds the past errors
float sum; //Holds the summation of the errors
// ********************
float maxSum = 1; // Optional integral term
unsigned long delta; //Calculates the derivative of the error
unsigned long t; //time variable
// ********************
int dacMax = 255; // Arduino dac is eight bits.
int adcMax = 1024; //Arduino uses a 10 bit dac, 10th power of 2 = 1024
int adcCenter = 512; // 1024/2
// ********************
int pwm_offset = 0; // Direct motor command from host.
int pwm_corr = 0;
int reference = 0; // Desired value (speed or angle) from host.
// ********************
float Kp = 0; // Feedback gains (proportional)
float Kd = 0; // Feedback gains (delta)
float Ki = 0; // Feedback gains (sum)
// ********************
char *arg;
int dist_pix = 0;
float dist_mm=0;
float error=0;
int motorCmd;
int pastmotorCmd;
float Ts = 0;


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

  scmd.addCommand("A",getanglePos);
  scmd.addCommand("B",getangleNeg);
  scmd.addDefaultHandler(defaultt);
}
void getanglePos(){
  char *arg;
  arg=scmd.next();
  if(arg !=NULL){
    dist_pix=atoi(arg);
  }
} 

void getangleNeg(){
  char *arg;
  arg=scmd.next();
  if(arg !=NULL){
    dist_pix=atoi(arg);
    dist_pix = - dist_pix;
  }
} 

void defaultt(){
   NOP;
}


void loop() {
  scmd.readSerial();
  // ********************
  t = millis();
  duration = t - p_time;
  p_time = t;
  // ********************
  Kp = 2; // Feedback gains (proportional)
  Kd = 0; // Feedback gains (delta)
  Ki = 0; // Feedback gains (sum)
  // ********************
  maxSum = 5; // Optional integral term
  pwm_offset = 100;
  // ********************
  // ********************
  // Update pastError
  for (int i = pastSize - 1; i > 0; i--) {
    pastError[i] = pastError[i - 1];
  }
  // Update pastEmotorCmdrror
  for (int i = 1; i > 0; i--) {
    pastError[i] = pastError[i - 1];
  }
  // Calculate the error 
  dist_mm = dist_pix/1.65;
  error = reference - dist_mm;
  pastError[0] = error;
  // ********************
  Ts= pastSize * duration * 1e-3;
  // Calculate the delta and sum
  float delta = (pastError[0] -  pastError[1] ) / Ts;
  sum = min(max(sum + (pastError[0] + pastError[1])/2*Ts, -1 * maxSum), maxSum);
  // ********************
  // ********************
  if (error >= 0) {
    motorCmd = int(Kp * error+ Ki * sum + Kd * delta); 
    motorCmd = abs(motorCmd);
    analogWrite(right_motor_pwm, pwm_offset  );
    analogWrite(left_motor_pwm, min(pwm_offset + motorCmd, dacMax) ); // Left PWM is Pin 5 
    initial_start();
  } 
  
  else {
    motorCmd = int(Kp * error + Ki * sum + Kd * delta); 
    motorCmd = abs(motorCmd);
    analogWrite(left_motor_pwm, pwm_offset);
    analogWrite(right_motor_pwm, min(pwm_offset + motorCmd , dacMax) ); // Right PWM is Pin 10
    initial_start();
  }

}

void initial_start () {
  digitalWrite(left_motor_back, LOW);
  digitalWrite(left_motor_forward, HIGH);
  digitalWrite(right_motor_forward, HIGH);
  digitalWrite(right_motor_back, LOW);
}