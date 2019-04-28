//EE494 Capstone Project Arduino code
//All rights are reserved
// DUAYENLER Ltd. Şti

#include <SerialCommand.h>
SerialCommand scmd;
#define NOP __asm__ __volatile__ ("nop\n\t")
#define left_motor_back  7// 4
#define left_motor_forward 8 // 2
#define right_motor_forward 10 // 6
#define right_motor_back 12 // 7
#define left_motor_pwm 9 // 3
#define right_motor_pwm 11 // 5


// Variable Declerations
unsigned long deltaT = 1000; // time between samples (usecs) 1000->50000
const int pastSize = 3; // used for integrating
double duration = 0; // The duration between two loops
double t = 0;
double p_time = 0; // A variable to hold past time
// ********************
float pastError[pastSize]; //Holds the past errors
float sum; //Holds the summation of the errors
// ********************
float maxSum = 0; // Optional integral term
float delta; //Calculates the derivative of the error
// ********************
int dacMax = 255; // Arduino dac is eight bits.
int adcMax = 1024; //Arduino uses a 10 bit dac, 10th power of 2 = 1024
int adcCenter = 512; // 1024/2
// ********************
int pwm_offset = 0; // Direct motor command from host.
int left_motor_offset = 0; // PWM offset for Left Motor
int motorCmd=0; // Base motor command to motors
int halfmotorCmd=0; // half of the motorCmd
int reference = 0; // Desired value (speed or angle) from host.
// ********************
float Kp = 0; // Feedback gains (proportional)
float Kd = 0; // Feedback gains (delta)
float Ki = 0; // Feedback gains (sum)
// ********************
char *arg;
int next_dist_pix = 0;
int curr_dist_pix = 0;
float next_dist_mm=0;
float curr_dist_mm=0;
float next_error=0;
float curr_error=0;
float error=0;
float beta=0;
float Ts=0;
// ********************

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

  scmd.addCommand("R",getnextdistPos);  // next_dist_pix (y2) is positive, thus next_error is negative , turn right
  scmd.addCommand("L",getnextdistNeg);  // next_dist_pix (y2) is negative, thus next_error is positive , turn left
  scmd.addCommand("P",getcurrdistPos);  // curr_dist_pix (y1) is positive, thus curr_error is negative , turn right
  scmd.addCommand("N",getcurrdistNeg);  // curr_dist_pix (y1) is negative, thus curr_error is positive , turn left
  scmd.addCommand("B",getbeta);         // beta is the error angle from global frame and lane angle from car frame
  scmd.addDefaultHandler(defaultt);
}
void getnextdistPos(){
  char *arg;
  arg=scmd.next();
  if(arg !=NULL){
    next_dist_pix=atoi(arg);
  }
} 

void getnextdistNeg(){
  char *arg;
  arg=scmd.next();
  if(arg !=NULL){
    next_dist_pix = atoi(arg);
    next_dist_pix = - next_dist_pix;
  }
} 

void getcurrdistPos(){
  char *arg;
  arg=scmd.next();
  if(arg !=NULL){
    curr_dist_pix=atoi(arg);
  }
} 

void getcurrdistNeg(){
  char *arg;
  arg=scmd.next();
  if(arg !=NULL){
    curr_dist_pix = atoi(arg);
    curr_dist_pix = - curr_dist_pix;
  }
} 


void getbeta(){
  char *arg;
  arg=scmd.next();
  if(arg !=NULL){
    beta=atoi(arg);
    if (beta>180){
      beta=360-beta;
      beta=abs(beta);}
    else {
      beta=abs(beta);}
  }
} 

void defaultt(){
   NOP;
}

void loop()
{
  scmd.readSerial();
  // ********************
  t = millis();
  duration = t - p_time;
  p_time = t;
  // ********************
  Kp = 0.095; // Feedback gains (proportional)
  Kd = 0.11 ; // Feedback gains (delta)
  Ki = 0.06 ; // Feedback gains (sum)
  // ********************
  maxSum = 5; // Optional integral term,
  // pwm_offset = 28;//-0.7*angle;
  pwm_offset = max ( 28 - 0.2*beta , 24) ;
  // ********************
  // ********************
  // Update pastError
  for (int i = pastSize - 1; i > 0; i--) {
    pastError[i] = pastError[i - 1];
    }
  // Calculate the error and delta
  next_dist_mm = next_dist_pix/1.65;
  curr_dist_mm = curr_dist_pix/1.65; 
  next_error = reference - next_dist_mm;
  curr_error = reference - curr_dist_mm; 
  error=curr_error;
  pastError[0] = error;
  // ********************
  // Ts= pastSize * duration * 1e-3;
  Ts=100;
  delta = (pastError[0] - pastError[1]) / Ts ;
  sum = min(max(sum + (pastError[0] + pastError[1] + pastError[2]) / 3 * Ts, -1 * maxSum), maxSum);
  // ********************
  // ********************
  left_motor_offset = 3;
  motorCmd = int(Kp * error+ Kd * delta + Ki * sum); 
  motorCmd = abs(motorCmd);
  halfmotorCmd = motorCmd/2;
  // ********************
  // ********************  
  if (error >= 0) {
    analogWrite(left_motor_pwm, pwm_offset + left_motor_offset  );
    analogWrite(right_motor_pwm, min(pwm_offset  + motorCmd, dacMax)  )  ; // Right PWM is Pin 10
    initial_start();    
  } 
  
  else {
    analogWrite(right_motor_pwm, pwm_offset );
    analogWrite(left_motor_pwm, min(pwm_offset + left_motor_offset + motorCmd, dacMax) ) ; // Left PWM is Pin 5 
    initial_start();
    
  }

}

void initial_start () {
  digitalWrite(left_motor_back, LOW);
  digitalWrite(left_motor_forward, HIGH);
  digitalWrite(right_motor_forward, HIGH);
  digitalWrite(right_motor_back, LOW);
}
