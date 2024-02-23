#include <Arduino.h>
#include <stdbool.h>

#define PIN_LED_RED 35
#define PIN_LED_IR 36
#define PIN_SWITCH_RED 29
#define PIN_SWITCH_IR 30

void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_IR, OUTPUT);
    pinMode(PIN_SWITCH_RED, OUTPUT);
    pinMode(PIN_SWITCH_IR, OUTPUT);
}

/**
 * @brief PWM signal generator for pulse oximeter LEDs
 * 
 * @param frequency Frequency of the PWM signal in Hz
 * @param dutyCycleRed Duty cycle percentage of the red LED
 * @param dutyCycleBlack Duty cycle percentage of the IR LED
 */
void pwm_pulseox(uint16_t frequency, uint8_t dutyCycleRed, uint8_t dutyCycleBlack) {
    static uint32_t period = 1000000 / frequency; // Period in milliseconds
    static uint32_t highRed = period * dutyCycleRed / 100; // High time of the red LED in milliseconds
    static uint32_t highBlack = period * dutyCycleBlack / 100; // High time of the IR LED in milliseconds
    static uint32_t deadTime = (period - highRed - highBlack) / 2;  // Total dead time between both signals in milliseconds
    static uint8_t state = 0; // 0 = red high black low, 1 = both low, 2 = red low black high, 3 = both low
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

void loop() {
    pwm_pulseox(1000, 25, 25);
}
