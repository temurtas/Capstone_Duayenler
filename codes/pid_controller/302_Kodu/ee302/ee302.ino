//EE302 Term Project Arduino code
//Written by Banu Çiçek BÜYÜKER, İlkay ATAOL, Gözde KORKMAZ and Barkın Tuncer (tuncer.barkin@gmail.com)
//Please send an email if you have any suggestions or find any bugs
//All rights are reserved
///Arduino pin 6 and pin 7 goes to the Motor Drivers Input pins. Arduino pin 9 goes to the Motor Driver Enable pin.

#define in1 6
#define in2 7
#define motorOutputPin 9

// Likely User Modified Variables ******************************
unsigned long deltaT = 1000; // time between samples (usecs) 1000->50000
const int pastSize = 3; // used for integrating

//These two values have to be set according to your setup
float angle_multip = 0.26; // Multiplies the analog read value that is coming from the POT
float angle_base = 130; // The bias term to adjust the angle accordingly

float duration = 0; // The duration between two loops
double p_time = 0; // A variable to hold past time
// End Likely User Modified Variables***************************

int angleSensorPin = A0; // Arduino Analog 0 pin should be connected to POT's middle leg.
float pastError[pastSize]; //Holds the past errors
float sum; //Holds the summation of the errors

int dacMax = 255; // Arduino dac is eight bits.
int adcMax = 1024; //Arduino uses a 10 bit dac, 10th power of 2 = 1024
int adcCenter = 512; // 1024/2

float direct = 0; // Direct motor command from host.
float desired = 0; // Desired value (speed or angle) from host.
float Kp = 0; // Feedback gains (proportional)
float Kd = 0; // Feedback gains (delta)

float Ki = 0; // Feedback gains (sum)
float maxSum = 1; // Optional integral term

unsigned long delta; //Calculates the derivative of the error


unsigned long t; //time variable
int incoming[12]; // array that holds the variabels coming from GUI
char pos_neg; // defines being positive or negative for "Desired"

void setup() {
  // Pin initialization
  pinMode(motorOutputPin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  Serial.begin(9600);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}


void loop() {
  t = millis();
  duration = t - p_time;
  p_time = t;
  // Reads the data coming from the GUI. Until ****************
  if (Serial.available() >= 12) {
    for (int i = 0; i < 12; i++) {
      incoming[i] = Serial.read();
    }
  }
  //Assignment of each variable.
  Kp = incoming[0] + incoming[1] / 100.0; // Feedback gains (proportional)
  Kd = incoming[2] + incoming[3] / 100.0; // Feedback gains (delta)
  Ki = incoming[4] + incoming[5] / 100.0; // Feedback gains (sum)
  desired = incoming[6] + incoming[7] / 100.0 - 90;
  direct = incoming[8] + incoming[9] / 100.0;
  maxSum = incoming[10] + incoming[11] / 100.0; // Optional integral term
  // ******************
  // User modifiable code between stars!!
  /***********************************************/
  // Measure angle three times and sum, then by twelve. Averaging reduces noise.
  // Factor of four is ratio of adc and dac.
  //float ang = float (analogRead(angleSensorPin)+ analogRead(angleSensorPin) + analogRead(angleSensorPin) - 3*adcCenter)/12.0;
  float ang = float (analogRead(angleSensorPin) * angle_multip) - angle_base;

  //Update pastError
  for (int i = pastSize - 1; i > 0; i--) {
    pastError[i] = pastError[i - 1];
  }
  // Calculate the error and assign the error as the recent error
  float error = desired - ang;
  pastError[0] = error;



  // Calculate the derivative for the derivative controller and calculate the sum for integral controller
  float delta = (pastError[0] - pastError[pastSize - 1]) / (pastSize * duration * 1e-3);
  //float delta = (pastError[0] - pastError[pastSize-1])/(pastSize*(float(deltaT)*1e-6));
  sum = min(max(sum + error * float(deltaT) * 1e-6, -1 * maxSum), maxSum);

  // Compute and limit the motor output command.
  int motorCmd = int(direct + Kp * error + Ki * sum + Kd * delta); //
  motorCmd = min(max(motorCmd, 0), dacMax);

  analogWrite(motorOutputPin, motorCmd); //Send PWM signal to L298N Enable

  /***********************************************/
  //Do not use any other Serial.print code while using the GUI.
  //If you want to see any variable changes through serial monitor of Arduino IDE,
  //do not use the GUI.
  //Only use this(**) line and comment the
  //belows when you are tuning the angle of the arm.
  //**Serial.println(ang);
  Serial.print(t / 1000.0); // Sends the data to serial port.
  Serial.print(" , ");
  Serial.print(error); // Sends the data to serial port.
  Serial.print(" , ");
  Serial.print(ang);// Sends the data to serial port.
  Serial.print(" , ");
  Serial.println(motorCmd); // Sends the data to serial port.
  delay(100);
}


