// Author: Justice Amoh
// Date: 03/29/2017
// Description: Pmod AD1 - AD7476A SPI ADC - Teensy 3.2 Code

#include <Arduino.h>
#include <SPI.h>

/* -------------------------------------------------------------------- */
/*				Global Variables		        */
/* -------------------------------------------------------------------- */

#define BUFFLENGTH 5000

char sMsg[6];  // character string to keep message that is displayed on serial monitor
uint16_t datBuf[BUFFLENGTH] = {0};
char a;
int n;
int timewaiting;
char Handshake = 0;

const int CSPin = 10;
const int MOSIPin = 11;
const int MISOPin = 12;
const int SCKPin = 14;

int led = 13;

// SPI Settings: speed, mode and endianness
SPISettings settings(1000000, MSBFIRST, SPI_MODE2);  // 1MHz, MSB,

int getADC();

void setup() {
    pinMode(led, OUTPUT);

    pinMode(CSPin, OUTPUT);
    Serial.begin(9600);

    // Configure SPI Pins
    SPI.begin();
    SPI.setMISO(MISOPin);
    SPI.setMOSI(MOSIPin);
    SPI.setSCK(SCKPin);
}

void loop() {
    digitalWrite(led, HIGH);  // turn the LED on (HIGH is the voltage level)

    while (!Handshake) {
        timewaiting = 0;
        sprintf(sMsg, "%u", 16383);  // format text to be displayed
        Serial.println(sMsg);        // display text on serial monitor
        Serial.flush();

        // wait for ACK
        a = 'b';
        while ((a != 'a') & (timewaiting < 70)) {
            a = Serial.read();

            digitalWrite(led, HIGH);  // turn the LED on (HIGH is the voltage level)
            delay(100);               // wait for a second
            digitalWrite(led, LOW);   // turn the LED off by making the voltage LOW
            delay(900);               // wait for a second

            timewaiting++;
        }

        if (timewaiting < 70) {
            Handshake = 1;
        } else {
            for (n = 0; n < 5; n = n + 1) {
                digitalWrite(led, HIGH);  // turn the LED on (HIGH is the voltage level)
                delay(50);                // wait for a second
                digitalWrite(led, LOW);   // turn the LED off by making the voltage LOW
                delay(50);                // wait for a second
            }
            Serial.end();
            Serial.begin(9600);  // reset the port
        }
    }

    // read into datBuf
    for (n = 0; n < BUFFLENGTH; n = n + 1) {
        datBuf[n] = getADC();  // read physical value
    }

    // transmit
    digitalWrite(led, HIGH);  // turn the LED on (HIGH is the voltage level)
    for (n = 0; n < BUFFLENGTH; n = n + 1) {
        sprintf(sMsg, "%u", datBuf[n]);  // format text to be displayed
        Serial.println(sMsg);            // display text on serial monitor
        Serial.flush();
    }
    digitalWrite(led, LOW);  // turn the LED off by making the voltage LOW
    Handshake = 0;
}

int getADC() {
    int data = 0, data_LB = 0, data_HB = 0;

    SPI.beginTransaction(settings);
    digitalWrite(CSPin, LOW);         // pull CSPin low to start SPI communication
    data_HB = SPI.transfer(0);        // grab upper byte
    data_LB = SPI.transfer(0);        // grab lower byte
    data = (data_HB << 8) | data_LB;  // concatenate data into a 16-bit word
    digitalWrite(CSPin, HIGH);        // set CSPin high to end SPI communication
    SPI.endTransaction();

    return data;
}