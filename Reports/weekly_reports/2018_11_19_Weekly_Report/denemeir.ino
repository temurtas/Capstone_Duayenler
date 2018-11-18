    /*
    * Arduino Wireless Communication Tutorial
    *       Example 1 - Receiver Code
    *                
    * by Dejan Nedelkovski, www.HowToMechatronics.com
    * 
    * Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
    */
    #include <SPI.h>
    #include <nRF24L01.h>
    #include <RF24.h>
    
    RF24 radio(9, 10); // CE, CSN
    char text;
    const byte addresses[][6] = {"00001", "00002"};
    
    void setup() {
      Serial.begin(9600);
      radio.begin();
      radio.openWritingPipe(addresses[1]); // 00001
      radio.openReadingPipe(1, addresses[0]); // 00002
   
      radio.setPALevel(RF24_PA_MIN);
      //radio.startListening();
   
    }
    void loop() {
      if(Serial.available()){
      radio.stopListening();
      text=(char)Serial.read();
      radio.write(&text, sizeof(text));
      
      radio.startListening();
      delay(5);
      }
      if (radio.available()) {
        
        char text;
        radio.read(&text, sizeof(text));
        Serial.println(text);
        delay(5);
      }
    }
