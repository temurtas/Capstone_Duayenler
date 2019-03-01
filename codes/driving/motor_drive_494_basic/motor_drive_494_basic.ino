// EE494 Capstone Project Arduino code
// All rights are reserved
// DUAYENLER Ltd. Şti


#define left_motor_back 7
#define left_motor_forward 6
#define right_motor_forward 9
#define right_motor_back 8
#define left_motor_pwm 5
#define right_motor_pwm 10

// Variable Declerations

int pwm_offset = 0; // Direct motor command from host.
int pwm_corr = 0;
int desired = 0; // Desired value (speed or angle) from host.


int dacMax = 255; // Arduino dac is eight bits.
int adcMax = 1024; //Arduino uses a 10 bit dac, 10th power of 2 = 1024
int adcCenter = 512; // 1024/2


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
    ang = ang/5;
  }
} 

void defaultt(){
   NOP;
}

void loop() {
  int desired =0;
  float error = ang - desired;

  
  motorCmd = error/20;
  
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

