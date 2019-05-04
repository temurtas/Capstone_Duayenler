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
int pwm_delta = 0;  // Direct+motorCmd;
int left_motor_offset = 0; // PWM offset for Left Motor
int motorCmd = 0; // Base motor command to motors
int halfmotorCmd = 0; // half of the motorCmd
int reference = 0; // Desired value (speed or angle) from host.
// ********************
float Kp = 0; // Feedback gains (proportional)
float Kd = 0; // Feedback gains (delta)
float Ki = 0; // Feedback gains (sum)
// ********************
float Kp_cw = 0; // Feedback gains (proportional)
float Kd_cw = 0; // Feedback gains (delta)
float Ki_cw = 0; // Feedback gains (sum)
// ********************
float Kp_ccw = 0; // Feedback gains (proportional)
float Kd_ccw = 0; // Feedback gains (delta)
float Ki_ccw = 0; // Feedback gains (sum)
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
// ****************
int stop_state = 0;
const int yonSize = 5;
float pastyon[yonSize]; // Holds Past Yons
float yonSum=0;
float yonAvg=0;
float yon = 0;
int yonFin=0;
int equal_pid=0;
int Kmin_ccw=0;
int Kmin_cw=0;
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
      yon = 0;
    }
    else {
      beta = abs(beta);
      yon = 1;
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

// Equal PID :: NO
// Regulator Voltage at the time of Tuning :: 10.44 V
// Lipo Cell Voltages at the time of Tuning :: 3.95 Vs
// Is Camera at right position :: YES
// Is Sampling time still accaptable :: 55-85?
// Whose path is it :: OURS



void loop()
{ // Stop State
  if (stop_state == 1) {
    analogWrite(right_motor_pwm, 0 );
    analogWrite(left_motor_pwm, 0 ) ;
    scmd.readSerial();
  } 
  else {  // Run State
    equal_pid=1;
    scmd.readSerial();
    // ********************
    t = millis();
    duration = t - p_time;
    p_time = t;
    // ********************
    Kp = 0.205; // Feedback gains (proportional) , ilk çalışan=0.15,16 , hizli=19, gün sonu=18 , uzunvid=19
    Kd = 0.07 ; // Feedback gains (delta) , ilk çalışan=0.11 , hizli=0.12 , gün sonu=13 , uzunvid=158
    Ki = 0.25 ; // Feedback gains (sum) , ilk çalışan=0.06 , hizli=0.075 ,gün sonu=06 , uzunvid= 058
    // ********************
    maxSum = 5; // Optional integral term,
    // pwm_offset = 28;//-0.7*angle;
    // ********************
    // ********************
    // Update pastError
    for (int i = pastSize - 1; i > 0; i--) {
      pastError[i] = pastError[i - 1];
    }
    // Update pastYon
    for (int i = yonSize - 1; i > 0; i--) {
      pastyon[i] = pastyon[i - 1];
    }
    // Calculate the error and delta
    pastyon[0]=yon;
    next_dist_mm = next_dist_pix / 1.65;
    curr_dist_mm = curr_dist_pix / 1.65;
    next_error = reference - next_dist_mm;
    curr_error = reference - curr_dist_mm;
    error = curr_error;
    pastError[0] = error;
    // Ts= pastSize * duration * 1e-3;
    Ts = 54e-3;
    // ********************
    delta = (pastError[0] - pastError[1]) / Ts ;
    sum = min(max(sum + (pastError[0] + pastError[1] + pastError[2]) / 3 * Ts, -1 * maxSum), maxSum);
    // ********************
    yonSum=0;
    for (int k = 0; k < (yonSize-1) ; k++)
    {
      yonSum = yonSum + pastyon[k];
      }
    yonAvg=yonSum/yonSize; 
    if (yonAvg < 0.5){
      yonFin=0;  
    }   
    else {
      yonFin=1;  
    }
    // ********************
    if (beta > 22.5 || (abs(error) > 70)){
        Kmin_cw=36;
        Kmin_ccw=32;
    }
    else{
        Kmin_cw=37;
        Kmin_ccw=34;
    }
    // *******************
    if (yonFin == 1 ){  // CCW Movement on the path
      // ********************
      Kp_cw = 0.178; // Feedback gains (proportional) , ilk çalışan=0.15,16 , hizli=19, gün sonu=18 , uzunvid=19
      Kd_cw = 0.19 ; // Feedback gains (delta) , ilk çalışan=0.11 , hizli=0.12 , gün sonu=13 , uzunvid=158
      Ki_cw = 0.4 ; // Feedback gains (sum) , ilk çalışan=0.06 , hizli=0.075 ,gün sonu=06 , uzunvid= 058
      // ********************
      left_motor_offset = 0;
      if (equal_pid == 1){  // Equal PID Mode for Both Direction
        motorCmd = int(Kp * error + Kd * delta + Ki * sum);
      }
      else {  // Different PID Mode for Both Direction
        motorCmd = int(Kp_cw * error + Kd_cw * delta + Ki_cw * sum);
      }
      motorCmd = abs(motorCmd);
      motorCmd =  min(motorCmd,25); 
      halfmotorCmd = motorCmd / 2;
      // ******************** 
      // pwm_offset = max ( 30 - 1.375 * beta , Kmin_ccw) ; // 29,25 , hizli=36,0.7,25,26 gün sonu=34,.7,26, uzunvid=54,1.875,26
      pwm_offset = max ( 39 - 1.475 * beta , Kmin_cw) ;
      pwm_delta  = min( pwm_offset  + motorCmd, dacMax);
    } 
    else{  // CW Movement on the path
      // ********************
      Kp_ccw = 0.295; // Feedback gains (proportional) , ilk çalışan=0.15,16 , hizli=19, gün sonu=18 , uzunvid=19
      Kd_ccw = 0.18 ; // Feedback gains (delta) , ilk çalışan=0.11 , hizli=0.12 , gün sonu=13 , uzunvid=158
      Ki_ccw = 0.4 ; // Feedback gains (sum) , ilk çalışan=0.06 , hizli=0.075 ,gün sonu=06 , uzunvid= 058
      // ********************
      left_motor_offset = 0;
      if (equal_pid == 1){  // Equal PID Mode for Both Direction
        motorCmd = int(Kp * error + Kd * delta + Ki * sum);
      }
      else {  // Different PID Mode for Both Direction
        motorCmd = int(Kp_ccw * error + Kd_ccw * delta + Ki_ccw * sum);
      }
      motorCmd = abs(motorCmd);
      motorCmd =  min(motorCmd,25); 
      halfmotorCmd = motorCmd / 2;
      // ******************** 
      // pwm_offset = max ( 36 - 1.34* beta , Kmin_cw) ; // 29,25 , hizli=36,0.7,25,26 , gün sonu=39,.7,29, uzunvid=60,1.78,30
      pwm_offset=max ( 36 - 1.84* beta , Kmin_ccw) ;;
      pwm_delta  = min( pwm_offset  + motorCmd, dacMax);
    }
    // ********************
    if (error > -(1e-5)) {
      analogWrite(left_motor_pwm, pwm_offset   );
      analogWrite(right_motor_pwm, pwm_delta  )  ; // Right PWM is Pin 10
      initial_start();
     }
    else {
      analogWrite(right_motor_pwm, pwm_offset );
      analogWrite(left_motor_pwm, pwm_delta + left_motor_offset) ; // Left PWM is Pin 5
      initial_start();
    }
    // ********************
    delay(38);
  }
  
  // ********************
}

void initial_start () {
  digitalWrite(left_motor_back, LOW);
  digitalWrite(left_motor_forward, HIGH);
  digitalWrite(right_motor_forward, HIGH);
  digitalWrite(right_motor_back, LOW);
}
