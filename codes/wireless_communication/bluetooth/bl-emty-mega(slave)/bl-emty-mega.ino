#include <SoftwareSerial.h>
#define button 8
#define pwm_pin 7
#define in1 6
#define in2 5
SoftwareSerial BTSerial(10, 11); // RX | TX

int state = 0;
int buttonState = 0;


void setup() 
{
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in2,HIGH);
  digitalWrite(in1,LOW);
  pinMode(pwm_pin, OUTPUT);
  pinMode(button, INPUT);
  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  BTSerial.begin(38400);  // HC-05 default speed in AT command more
}

void loop()
{
  
  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (BTSerial.available()){
    //Serial.write(BTSerial.read());
    state=BTSerial.read();
  }
  /*if (Serial.available()){
    BTSerial.write(Serial.read());
  }*/
  analogWrite(pwm_pin,state);
  
  
  buttonState = digitalRead(button);
  if (buttonState == HIGH) {
  BTSerial.write('1'); // Sends '1' to the master to turn on LED
 }
 else {
   BTSerial.write('0');
 }  
}
