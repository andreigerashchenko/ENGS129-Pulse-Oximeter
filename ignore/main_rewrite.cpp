#include <Arduino.h>
#include <stdbool.h>

#define PWM_FREQUENCY 1000 // 1kHz
#define PWM_RESOLUTION 16 // 16-bit resolution

#define PIN_LED_RED 35
#define PIN_LED_IR 36
#define PIN_SWITCH_RED 29
#define PIN_SWITCH_IR 30

void pulseox_pwm(uint8_t dutyCycleRed, uint8_t dutyCycleIR, uint8_t phaseShift);

void setup() {
    
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_IR, OUTPUT);

    pinMode(PIN_SWITCH_RED, OUTPUT);
    pinMode(PIN_SWITCH_IR, OUTPUT);

    analogWriteFrequency(PIN_LED_RED, PWM_FREQUENCY);
    analogWriteFrequency(PIN_LED_IR, PWM_FREQUENCY);

    analogWriteResolution(PWM_RESOLUTION);

    analogWrite(PIN_LED_RED, 32768);
    // Introduce 180 degree phase shift between the red and IR LEDs
    delayMicroseconds(500); // Phase shift (500us = 180 degrees at 1kHz frequency)
    analogWrite(PIN_LED_IR, 32768);


    // analogWriteFrequency(PIN_LED_RED, PWM_FREQUENCY);
    // analogWriteFrequency(PIN_LED_IR, PWM_FREQUENCY);
    // analogWriteResolution(PWM_RESOLUTION);

    // pulseox_pwm(25, 25, 180);
}

void loop() {

}

/**
 * @brief PWM signal generator for pulse oximeter LEDs
 * 
 * @param dutyCycleRed Duty cycle percentage of the red LED
 * @param dutyCycleIR Duty cycle percentage of the IR LED
 * @param phaseShift Phase shift between the red and IR LEDs in degrees
 */
void pulseox_pwm(uint8_t dutyCycleRed, uint8_t dutyCycleIR, uint8_t phaseShift) {
    // 50% = 2^16 * 0.5 = 32768
    uint16_t red_PWM_value = (uint16_t)(pow(2, PWM_RESOLUTION) * (dutyCycleRed / 100.0) - 1);
    uint16_t ir_PWM_value = (uint16_t)(pow(2, PWM_RESOLUTION) * (dutyCycleIR / 100.0) - 1);
    uint32_t phaseshift_delay = (uint32_t)(phaseShift / 360.0 * (1.0 / PWM_FREQUENCY) * 1000000); // Phase shift in microseconds
    analogWriteFrequency(PIN_LED_RED, red_PWM_value);
    delayMicroseconds(phaseshift_delay); // Phase shift
    analogWriteFrequency(PIN_LED_IR, ir_PWM_value);
}

// #include <Arduino.h>

// #define PWM_PIN 35

// void setup() {
//     pinMode(PWM_PIN, OUTPUT);
//     analogWriteFrequency(PWM_PIN, 1000);
//     analogWriteResolution(16);
//     analogWrite(PWM_PIN, 32768);
// }

// void loop() {
// }