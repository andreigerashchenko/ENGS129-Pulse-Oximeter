#include <Arduino.h>
#include <stdbool.h>
#include <unistd.h>

#include "SPI.h"

#define SERIAL_BAUD 9600
#define RECORDING_INTERVAL 1     // seconds
#define RECORDING_FREQUENCY 100  // Hz

#define ADC_MISO_1 8
#define ADC_MISO_2 12
#define ADC_CS 10
#define ADC_SCK 27
#define ADC_MOSI 11

#define PIN_LED_RED 35
#define PIN_LED_IR 36
#define PIN_SWITCH_RED 29
#define PIN_SWITCH_IR 30

// #define BUFFER_SIZE 1024 * 3  // 1024 3 byte messages
#define BUFFER_SIZE 5 * 3  // 5 3 byte messages

typedef struct {
    uint8_t reading[2];  // 2 bytes for the ADC reading (12 bits)
    uint8_t channel;     // 1 or 2
} adc_reading_t;

uint32_t last_read_time = 0;
uint8_t state = 0;  // 0 = red high black low, 1 = both low, 2 = red low black high, 3 = both low
// adc_reading_t send_buffer[BUFFER_SIZE] = {0};  // Buffer to store the ADC values [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, ...
uint16_t send_buffer[RECORDING_INTERVAL * RECORDING_FREQUENCY * 2];
uint16_t buffer_index = 0;
SPISettings settings(1000000, MSBFIRST, SPI_MODE2);  // 1MHz, MSB, mode 2

void pwm_pulseox(uint16_t frequency, uint8_t dutyCycleRed, uint8_t dutyCycleBlack);

void setup() {
    Serial.begin(9600);
    // adc_init();
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_IR, OUTPUT);
    pinMode(PIN_SWITCH_RED, OUTPUT);
    pinMode(PIN_SWITCH_IR, OUTPUT);

    pinMode(ADC_CS, OUTPUT);
    pinMode(ADC_SCK, OUTPUT);
    pinMode(ADC_MISO_1, INPUT);
    pinMode(ADC_MISO_2, INPUT);
    pinMode(ADC_MOSI, OUTPUT);

    // Set up the timer for PWM signal generation
    // IntervalTimer timerPWM;
    // timerPWM.begin(timer_PWM_ISR, 1); // Run PWM function at 1kHz (1000us period)
    SPI.begin();
    SPI.setSCK(ADC_SCK);
    // SPI.setCS(ADC_CS);
    SPI.setMISO(ADC_MISO_1);
    SPI.setMOSI(ADC_MOSI);
}

void loop() {
    pwm_pulseox(100, 25, 25);

    if (millis() - last_read_time >= (1000 / RECORDING_FREQUENCY)) {
        last_read_time = millis();
        uint8_t data_MSB;
        uint8_t data_LSB;
        // adc_reading_t reading;
        // reading.channel = 1;
        // reading.reading[0] = 0;
        // reading.reading[1] = 0;
        // send_buffer[buffer_index] = reading;
        // buffer_index++;
        // reading.channel = 2;
        // reading.reading[0] = 0;
        // reading.reading[1] = 0;
        // send_buffer[buffer_index] = reading;
        // buffer_index++;
        
        if (buffer_index >= BUFFER_SIZE) {
            Serial.write((uint8_t *)send_buffer, BUFFER_SIZE * sizeof(adc_reading_t));
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