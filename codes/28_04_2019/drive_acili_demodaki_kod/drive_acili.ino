//EE494 Capstone Project Arduino code
//All rights are reserved
// DUAYENLER Ltd. Şti
// 10.91 Volts is crutial for the regulator output

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
int pwm_delta = 0;  // Direct+motorCmd;
int left_motor_offset = 0; // PWM offset for Left Motor
int motorCmd = 0; // Base motor command to motors
int halfmotorCmd = 0; // half of the motorCmd
int reference = 0; // Desired value (speed or angle) from host.
// ********************
float Kp = 0; // Feedback gains (proportional)
float Kd = 0; // Feedback gains (delta)
float Ki = 0; // Feedback gains (sum)
int Kmin1=0;
int Kmin2=0;
// ********************
char *arg;
int next_dist_pix = 0;
int curr_dist_pix = 0;
float next_dist_mm = 0;
float curr_dist_mm = 0;
float next_error = 0;
float curr_error = 0;
float error = 0;
float beta = 0;
float Ts = 0;
int stop_state = 0;
int yon;
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

  scmd.addCommand("R", getnextdistPos); // next_dist_pix (y2) is positive, thus next_error is negative , turn right
  scmd.addCommand("L", getnextdistNeg); // next_dist_pix (y2) is negative, thus next_error is positive , turn left
  scmd.addCommand("P", getcurrdistPos); // curr_dist_pix (y1) is positive, thus curr_error is negative , turn right
  scmd.addCommand("N", getcurrdistNeg); // curr_dist_pix (y1) is negative, thus curr_error is positive , turn left
  scmd.addCommand("B", getbeta);        // beta is the error angle from global frame and lane angle from car frame
  scmd.addCommand("S", getstop);        // Gets stop signal from Handshake protocol
  scmd.addDefaultHandler(defaultt);
}
void getnextdistPos() {
  char *arg;
  arg = scmd.next();
  if (arg != NULL) {
    next_dist_pix = atoi(arg);
    stop_state = 0;
  }
}

void getnextdistNeg() {
  char *arg;
  arg = scmd.next();
  if (arg != NULL) {
    next_dist_pix = atoi(arg);
    next_dist_pix = - next_dist_pix;
    stop_state = 0;
  }
}

void getcurrdistPos() {
  char *arg;
  arg = scmd.next();
  if (arg != NULL) {
    curr_dist_pix = atoi(arg);
    stop_state = 0;
  }
}

void getcurrdistNeg() {
  char *arg;
  arg = scmd.next();
  if (arg != NULL) {
    curr_dist_pix = atoi(arg);
    curr_dist_pix = - curr_dist_pix;
    stop_state = 0;
  }
}


void getbeta() {
  char *arg;
  arg = scmd.next();
  if (arg != NULL) {
    beta = atoi(arg);
    if (beta > 180) {
      beta = 360 - beta;
      beta = abs(beta);
      yon=0;
    }
    else {
      beta = abs(beta);
      yon=1;
    }
    stop_state = 0;
  }
}


void getstop() {
  char *arg;
  arg = scmd.next();
  if (arg != NULL) {
    stop_state = 1;
  }
}


void defaultt() {
  NOP;
}

// p =0.12 - 0.14
// i =0.02 - 0.02
// d = 0.091 - 0.0915

// saat 11.29 24 nisan
// Kp = 0.1358
// Kd = 0.1255
// Ki = 0.025
void loop()
{
  scmd.readSerial();
  // ********************
  t = millis();
  duration = t - p_time;
  p_time = t;
  // ********************
  Kp = 0.181; // Feedback gains (proportional) , ilk çalışan=0.15,16 , hizli=19, gün sonu=18 , uzunvid=19,205, bidön=18 , cp1=145, cp3=1453
  Kd = 0.152 ; // Feedback gains (delta) , ilk çalışan=0.11 , hizli=0.12 , gün sonu=13 , uzunvid=158,165, bidön=13 , cp1=1325,124,126, cp3=1345
  Ki = 0.028 ; // Feedback gains (sum) , ilk çalışan=0.06 , hizli=0.075 ,gün sonu=06 , uzunvid= 058,615, bidön=06 , cp1=02, cp3=028
  // ********************
  // Kp = 0.18; // Feedback gains (proportional) , ilk çalışan=0.15,16 , hizli=19, gün sonu=18 , uzunvid=19,205, bidön=18
  // Kd = 0.13 ; // Feedback gains (delta) , ilk çalışan=0.11 , hizli=0.12 , gün sonu=13 , uzunvid=158,165, bidön=13
  // Ki = 0.06 ; // Feedback gains (sum) , ilk çalışan=0.06 , hizli=0.075 ,gün sonu=06 , uzunvid= 058,615, bidön=06
  // ********************
  maxSum = 5; // Optional integral term,
  // pwm_offset = 28;//-0.7*angle;
  // ********************
  // ********************
  // Update pastError
  if (stop_state == 1) {
    analogWrite(right_motor_pwm, 0 );
    analogWrite(left_motor_pwm, 0 ) ;

  } else {
      for (int i = pastSize - 1; i > 0; i--) {
        pastError[i] = pastError[i - 1];
      }
      // Calculate the error and delta
      next_dist_mm = next_dist_pix / 1.65;
      curr_dist_mm = curr_dist_pix / 1.65;
      next_error = reference - next_dist_mm;
      curr_error = reference - curr_dist_mm;
      error = curr_error;
      
      pastError[0] = error;
      // ********************
      // pwm_offset = max ( 36 - 0.7*beta , 26) ; // 29,25 , hizli=36,0.7,25,26 
      // ********************
      // Ts= pastSize * duration * 1e-3;
      Ts = 54e-3;
      delta = (pastError[0] - pastError[1]) / Ts ;
      sum = min(max(sum + (pastError[0] + pastError[1] + pastError[2]) / 3 * Ts, -1 * maxSum), maxSum);
      // ********************
      // ********************
      left_motor_offset = 0;
      motorCmd = int(Kp * error + Kd * delta + Ki * sum);
      motorCmd = abs(motorCmd);
      halfmotorCmd = motorCmd / 2;
      // ********************
      
      if (beta > 27.5){
        Kmin1=26;
        Kmin2=32;}
      else{
        Kmin1=29;
        Kmin2=35;}
        
      if(yon == 0){
        pwm_offset = max ( 52 - 1.65 * beta , Kmin1) ; // 29,25 , hizli=36,0.7,25,26 gün sonu=34,.7,26, uzunvid=54,1.875,26-56,1.875,26 ,bidön=50,1.6,26
        // cp1=44,1.2,26, cp3=44,1.2,26,28
        //if (abs(error) > 70){pwm_offset=26;}
        }
        else{
        pwm_offset = max ( 62 - 1.41 * beta , Kmin2) ; // 29,25 , hizli=36,0.7,25,26 , gün sonu=39,.7,29, uzunvid=60,1.78,30-61,1.76,30, bidön=54,1.575,30
        // cp1=47,1.15,29, cp3=30,32,1.15,51
        //if (abs(error) > 70){pwm_offset=29;} 
       }
        
      // ********************
      if (error >= 0) {
        
        pwm_delta  = min( pwm_offset  + motorCmd, dacMax);
        analogWrite(left_motor_pwm, pwm_offset   );
        analogWrite(right_motor_pwm, pwm_delta  )  ; // Right PWM is Pin 10
        initial_start();
      }
  
      else {
        pwm_delta  = min( pwm_offset  + motorCmd, dacMax);
        analogWrite(right_motor_pwm, pwm_offset );
        analogWrite(left_motor_pwm, pwm_delta + left_motor_offset) ; // Left PWM is Pin 5
        initial_start();
      }
  }

}

void initial_start () {
  digitalWrite(left_motor_back, LOW);
  digitalWrite(left_motor_forward, HIGH);
  digitalWrite(right_motor_forward, HIGH);
  digitalWrite(right_motor_back, LOW);
}
