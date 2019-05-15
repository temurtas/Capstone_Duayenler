#define NOP __asm__ __volatile__ ("nop\n\t")
#define left_motor_back  7// 4
#define left_motor_forward 8 // 2
#define right_motor_forward 10 // 6
#define right_motor_back 12 // 7
#define left_motor_pwm 9 // 3
#define right_motor_pwm 11 // 5


void setup() {
  // put your setup code here, to run once:
  pinMode(left_motor_back, OUTPUT);
  pinMode(left_motor_forward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_back, OUTPUT);
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(right_motor_pwm, OUTPUT);
  //Serial.begin(9600);
  digitalWrite(left_motor_back, LOW);
  digitalWrite(left_motor_forward, LOW);
  digitalWrite(right_motor_back, LOW);
  digitalWrite(right_motor_forward, LOW);
  delay(3000);
  initial_start();
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(right_motor_pwm, 77 );
  analogWrite( left_motor_pwm, 70  ) ; 
}


void initial_start () {
  digitalWrite(left_motor_back, LOW);
  digitalWrite(left_motor_forward, HIGH);
  digitalWrite(right_motor_forward, HIGH);
  digitalWrite(right_motor_back, LOW);
}
