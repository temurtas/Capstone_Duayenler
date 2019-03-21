#include <SerialCommand.h>

//motor1 pins
#define pwm1 2
#define in1A 3
#define in1B 4

//motor2 pins
#define pwm2 5
#define in2A 6
#define in2B 7
int pwm_m1;
int pwm_m2;

SerialCommand SCmd;

void setup() {
  // put your setup code here, to run once:
  pinMode(pwm1,OUTPUT);
  pinMode(in1A,OUTPUT);
  pinMode(in1B,OUTPUT);
  pinMode(pwm2,OUTPUT);
  pinMode(in2A,OUTPUT);
  pinMode(in2B,OUTPUT);
  digitalWrite(pwm1,LOW);
  digitalWrite(pwm2,LOW);

  Serial.begin(9600);
  SCmd.addCommand("PWM",setpwm);      
  SCmd.addCommand("START",startt);  
  SCmd.addDefaultHandler(unrecognized);
  Serial.println("Ready"); 
}


void loop() {
  SCmd.readSerial();
  startt();

  pwm_m1=255;    
  pwm_m2=255;
  analogWrite(pwm1,pwm_m1);
  analogWrite(pwm2,pwm_m2);
  
}


void setpwm(){
  int pwm_m1;
  int pwm_m2;
  char *arg1;
  char *arg2; 
  arg1 = SCmd.next(); 
  arg2 = SCmd.next(); 
  
  if (arg1 != NULL) 
  {
    pwm_m1=atoi(arg1);    // Converts a char string to an integer
    pwm_m2=atoi(arg2);
    analogWrite(pwm1,pwm_m1);
    analogWrite(pwm2,pwm_m2);
  } 
  
}


void startt(){
  digitalWrite(in1B,HIGH);
  digitalWrite(in1A,LOW);

  digitalWrite(in2A,HIGH);
  digitalWrite(in2B,LOW);
}

void unrecognized(){
  Serial.println("What?"); 
}
