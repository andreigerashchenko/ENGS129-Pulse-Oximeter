// Proof of concept test for reading channels 1 and 2 from the Pmod AD1

#include <Arduino.h>
// #include "adc.h"
#include "SPI.h"

#define SERIAL_BAUD 250000
#define RECORDING_INTERVAL 1   // seconds
#define RECORDING_FREQUENCY 2  // Hz

#define ADC_MISO_1 8
#define ADC_MISO_2 12
#define ADC_CS 10
#define ADC_SCK 27
#define ADC_MOSI 11

#define PIN_LED_RED 35
#define PIN_LED_IR 36
#define PIN_SWITCH_RED 29
#define PIN_SWITCH_IR 30

typedef struct {
    uint32_t timestamp; // Timestamp in milliseconds
    uint8_t data[2]; // 12-bit ADC data, leading 4 bits are 0
    uint8_t channel; // 1 for channel 1, 2 for channel 2
} adc_data_t;

SPISettings settings(1000000, MSBFIRST, SPI_MODE2);  // 1MHz, MSB, mode 2
uint8_t state = 0;
uint32_t last_read_time = 0;
// uint16_t send_buffer[RECORDING_INTERVAL * RECORDING_FREQUENCY * 2];
adc_data_t send_buffer[RECORDING_INTERVAL * RECORDING_FREQUENCY * 2];
uint16_t buffer_index = 0;

void pwm_pulseox(uint16_t frequency, uint8_t dutyCycleRed, uint8_t dutyCycleBlack);

/**
 * @brief Sends a handshake signal to the computer to indicate that the Arduino is ready to send data.
 * Repeatedly sends the string "READY" over serial, until the computer sends "START" back.
 * Returns when "START" is received.
 */
void handshake() {
    char received[6] = {0};
    while (strcmp(received, "START") != 0) {
        Serial.write("READY\n");
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
        if (Serial.available() >= 5) {
            Serial.readBytes(received, 5);
        }
    }
    digitalWrite(LED_BUILTIN, HIGH);
}

void setup() {
    SPI.begin();
    SPI.setSCK(ADC_SCK);
    // SPI.setCS(ADC_CS);
    SPI.setMISO(ADC_MISO_1);
    SPI.setMOSI(ADC_MOSI);

    // Configure LED pin
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_IR, OUTPUT);
    pinMode(PIN_SWITCH_RED, OUTPUT);
    pinMode(PIN_SWITCH_IR, OUTPUT);

    pinMode(ADC_CS, OUTPUT);
    // pinMode(ADC_SCK, OUTPUT);
    // pinMode(ADC_MISO_1, INPUT);
    // pinMode(ADC_MISO_2, INPUT);
    // pinMode(ADC_MOSI, OUTPUT);

    Serial.begin(SERIAL_BAUD);
    handshake();
}

void loop() {
    pwm_pulseox(1000, 25, 25);

    if (millis() - last_read_time > (1000 / RECORDING_FREQUENCY)) {
        last_read_time = millis();
        uint8_t MSB;
        uint8_t LSB;


        // Read channel 1
        SPI.setMISO(ADC_MISO_1);
        SPI.beginTransaction(settings);
        digitalWrite(ADC_CS, LOW);
        MSB = SPI.transfer(0);  // grab upper byte
        LSB = SPI.transfer(0);  // grab lower byte
        digitalWrite(ADC_CS, HIGH);
        SPI.endTransaction();
        send_buffer[buffer_index].data[0] = MSB;
        send_buffer[buffer_index].data[1] = LSB;
        send_buffer[buffer_index].channel = 1;
        send_buffer[buffer_index].timestamp = millis();
        buffer_index++;

        // Read channel 2
        SPI.setMISO(ADC_MISO_2);
        SPI.beginTransaction(settings);
        digitalWrite(ADC_CS, LOW);
        MSB = SPI.transfer(0);  // grab upper byte
        LSB = SPI.transfer(0);  // grab lower byte
        digitalWrite(ADC_CS, HIGH);
        SPI.endTransaction();
        send_buffer[buffer_index].data[0] = MSB;
        send_buffer[buffer_index].data[1] = LSB;
        send_buffer[buffer_index].channel = 2;
        send_buffer[buffer_index].timestamp = millis();
        buffer_index++;

        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        if (buffer_index == RECORDING_INTERVAL * RECORDING_FREQUENCY) {
            Serial.write((uint8_t *)send_buffer, sizeof(send_buffer));
            buffer_index = 0;
        }
    }
}

/**
 * @brief PWM signal generator for pulse oximeter LEDs
 *
 * @param frequency Frequency of the PWM signal in Hz
 * @param dutyCycleRed Duty cycle percentage of the red LED
 * @param dutyCycleBlack Duty cycle percentage of the IR LED
 */
void pwm_pulseox(uint16_t frequency, uint8_t dutyCycleRed, uint8_t dutyCycleBlack) {
    static uint32_t period = 1000000 / frequency;                   // Period in milliseconds
    static uint32_t highRed = period * dutyCycleRed / 100;          // High time of the red LED in milliseconds
    static uint32_t highBlack = period * dutyCycleBlack / 100;      // High time of the IR LED in milliseconds
    static uint32_t deadTime = (period - highRed - highBlack) / 2;  // Total dead time between both signals in milliseconds
    static uint32_t lastTime = 0;
    static bool setup = false;

    // States:
    // 0: Red high, black low
    // 1: Both low
    // 2: Red low, black high
    // 3: Both low

    if (!setup) {
        digitalWrite(PIN_LED_RED, HIGH);
        digitalWrite(PIN_LED_IR, LOW);
        digitalWrite(PIN_SWITCH_RED, HIGH);
        digitalWrite(PIN_SWITCH_IR, LOW);
        setup = true;
        lastTime = micros();
    }

    uint32_t currentTime = micros();
    switch (state) {
        case 0:
            if (currentTime - lastTime >= highRed) {
                digitalWrite(PIN_LED_RED, LOW);
                digitalWrite(PIN_SWITCH_RED, LOW);
                state = 1;
                lastTime = currentTime;
            }
            break;
        case 1:
            if (currentTime - lastTime >= deadTime) {
                digitalWrite(PIN_LED_IR, HIGH);
                digitalWrite(PIN_SWITCH_IR, HIGH);
                state = 2;
                lastTime = currentTime;
            }
            break;
        case 2:
            if (currentTime - lastTime >= highBlack) {
                digitalWrite(PIN_LED_IR, LOW);
                digitalWrite(PIN_SWITCH_IR, LOW);
                state = 3;
                lastTime = currentTime;
            }
            break;
        case 3:
            if (currentTime - lastTime >= deadTime) {
                digitalWrite(PIN_LED_RED, HIGH);
                digitalWrite(PIN_SWITCH_RED, HIGH);
                state = 0;
                lastTime = currentTime;
            }
            break;
    }
}