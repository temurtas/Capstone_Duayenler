#include <SoftwareSerial.h>
#define ledPin 9
int state = 0;
int potValue = 0;
SoftwareSerial BTSerial(10, 11); // RX | TX

void setup() 
{
  pinMode(9, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode
  digitalWrite(9, HIGH); 
  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  BTSerial.begin(38400);  // HC-05 default speed in AT command more
}

void loop()
{
  
  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (BTSerial.available()){
    state=BTSerial.read();
  }
  if (state == '1') {
  digitalWrite(ledPin, HIGH); // LED ON
  state = 0;
 }
 else if (state == '0') {
  digitalWrite(ledPin, LOW); // LED ON
  state = 0;
 }

 potValue = analogRead(A5);
 int potValueMapped = map(potValue, 0, 1023, 0, 255);
 BTSerial.write(potValueMapped); // Sends potValue to servo motor
 delay(10);

}
