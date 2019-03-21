// EE494 Capstone Project Arduino code
// All rights are reserved
// DUAYENLER Ltd. Şti


#define left_motor_back 4
#define left_motor_forward 2
#define right_motor_forward 5
#define right_motor_back 7
#define left_motor_pwm 3
#define right_motor_pwm 6

// Variable Declerations
// int pwm_offset = 0; // Direct motor command from host.


void setup() {
  // Pin initialization
  pinMode(left_motor_back, OUTPUT);
  pinMode(left_motor_forward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_back, OUTPUT);
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(right_motor_pwm, OUTPUT);
  Serial.begin(19200);
  digitalWrite(left_motor_back, LOW);
  digitalWrite(left_motor_forward, LOW);
  digitalWrite(right_motor_back, LOW);
  digitalWrite(right_motor_forward, LOW);
  Serial.begin(57600);
}


void loop() {
  int pwm_offset = 40;
  analogWrite(left_motor_pwm, pwm_offset);
  analogWrite(right_motor_pwm, pwm_offset);
  initial_start();
}

void initial_start () {
  digitalWrite(left_motor_back, LOW);
  digitalWrite(left_motor_forward, HIGH);
  digitalWrite(right_motor_forward, HIGH);
  digitalWrite(right_motor_back, LOW);
}

