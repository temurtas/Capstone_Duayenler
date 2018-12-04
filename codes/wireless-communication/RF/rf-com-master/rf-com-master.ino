
// MasterSwapRoles

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define CE_PIN   7
#define CSN_PIN 8

const byte slaveAddress[5] = {'R','x','A','A','A'};
const byte masterAddress[5] = {'T','X','a','a','a'};


RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

char dataToSend[10] = "Message 0";
char txNum = '0';
int dataReceived[2]; // to hold the data from the slave - must match replyData[] in the slave
bool newData = false;

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 1000; // send once per second

//============

void setup() {

    Serial.begin(9600);

    Serial.println("MasterSwapRoles Starting");

    radio.begin();
    radio.setDataRate( RF24_250KBPS );

    radio.openWritingPipe(slaveAddress);
    radio.openReadingPipe(1, masterAddress);

    radio.setRetries(3,5); // delay, count
    send(); // to get things started
    prevMillis = millis(); // set clock
}

//=============

void loop() {
    currentMillis = millis();
    if (currentMillis - prevMillis >= txIntervalMillis) {
        send();
        prevMillis = millis();
    }
    getData();
    showData();
}

//====================

void send() {

        radio.stopListening();
            bool rslt;
            rslt = radio.write( &dataToSend, sizeof(dataToSend) );
        radio.startListening();
        Serial.print("Data Sent ");
        Serial.print(dataToSend);
        if (rslt) {
            Serial.println("  Acknowledge received");
            updateMessage();
        }
        else {
            Serial.println("  Tx failed");
        }
}

//================

void getData() {

    if ( radio.available() ) {
        radio.read( &dataReceived, sizeof(dataReceived) );
        newData = true;
    }
}

//================

void showData() {
    if (newData == true) {
        Serial.print("Data received ");
        Serial.print(dataReceived[0]);
        Serial.print(", ");
        Serial.println(dataReceived[1]);
        Serial.println();
        newData = false;
    }
}

//================

void updateMessage() {
        // so you can see that new data is being sent
    txNum += 1;
    if (txNum > '9') {
        txNum = '0';
    }
    dataToSend[8] = txNum;
}



