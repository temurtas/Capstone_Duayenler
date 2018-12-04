#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX
String isim = "Enes Bluetooth";
int sifre = 11235;
String uart = "9600,0,0";
int led1 = 6;/*
int led2 = 6;
int led3 = 5;
int led4 = 4;*/
//char c=' ';
String c=" ";
void setup() {
  Serial.begin(9600);
  Serial.println("HC-05 Modul Ayarlaniyor...");
  Serial.println("Lutfen 5 sn icinde HC-05 modulun uzerindeki butona basili tutarak baglanti yapiniz.");
  mySerial.begin(38400);
  delay(5000);
  mySerial.print("AT+NAME=");
  mySerial.println(isim);
  Serial.print("Isim ayarlandi: ");
  Serial.println(isim);
  delay(1000);
  mySerial.print("AT+PSWD=");
  mySerial.println(sifre);
  Serial.print("Sifre ayarlandi: ");
  Serial.println(sifre);
  delay(1000);
  mySerial.print("AT+UART=");
  mySerial.println(uart);
  Serial.print("Baud rate ayarlandi: ");
  Serial.println(uart);
  delay(2000);
  Serial.println("Islem tamamlandi.");
  pinMode(led1, OUTPUT);/*
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);*/
  mySerial.begin(9600);
  mySerial.println("LED uygulamasi");
}


void loop() {
  // put your main code here, to run repeatedly
  
  if (mySerial.available())
    {  
        c = mySerial.readString();
        Serial.print(c);
        if(c=='1') digitalWrite(led1,HIGH);
        else if(c=='0') digitalWrite(led1,LOW);
        else analogWrite(led1,c.toInt());
        
    }
 
    // Keep reading from Arduino Serial Monitor and send to HC-05
    if (Serial.available())
    {
        c =  Serial.readString();
        mySerial.print(c);  
        if(c=='1') digitalWrite(led1,HIGH);
        if(c=='0') digitalWrite(led1,LOW);
    }
}
