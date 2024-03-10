#include <Arduino.h>

#include "SPI.h"
#include "TimerThree.h"

#define SERIAL_BAUD 2000000
#define SPI_RATE 1000000
#define SAMPLE_RATE 60         // Hz
#define RECORDING_PERIOD 3000  // milliseconds
#define PEAK_DETECTOR_DECAY 0.4
// Max expected heart rate is 200 bpm, so the minimum peak interval = 60 / (200 * 1000) = 300 ms
#define MIN_PEAK_INTERVAL 300  // milliseconds

#define AVERAGE_WINDOW 15  // Number of samples to average for heart rate and oxygen saturation

#define ADC_MISO_1 8
#define ADC_MISO_2 12
#define ADC_CS 10
#define ADC_SCK 27
#define ADC_MOSI 11

#define PIN_LED_RED 35
#define PIN_LED_IR 36
#define PIN_SWITCH_RED 29
#define PIN_SWITCH_IR 30

#define MAX_DUTY_CYCLE 49
#define MIN_DUTY_CYCLE 10
#define DUTY_CYCLE_STEP 1
#define AGC_UPPER_LIMIT 2.4       // Upper limit of input signal (Volts)
#define AGC_LOWER_LIMIT 0.9       // Lower limit of input signal (Volts)
#define AGC_TOLERANCE 0.05       // Tolerance for AGC (Volts)
#define AGC_UPDATE_INTERVAL 2500  // milliseconds to wait before updating duty cycle again

#define LED_BLINK_DURATION 100  // milliseconds

// DATA_BUFFER_SIZE is samples per second * number of seconds for one channel
#define DATA_BUFFER_SIZE ((SAMPLE_RATE * RECORDING_PERIOD) / 1000)

typedef struct {
    uint32_t timestamp;  // Timestamp in milliseconds
    uint8_t data_MSB;    // Most significant byte of the ADC reading
    uint8_t data_LSB;    // Least significant byte of the ADC reading
    uint8_t channel;     
    char end = '\n';     // Newline character to delimit data
} adc_data_t;

SPISettings SPI_SETTINGS(SPI_RATE, MSBFIRST, SPI_MODE2);  // 1MHz, MSB, mode 2

// Global PWM variables
IntervalTimer timer;
volatile uint8_t red_duty_cycle = 15;
volatile uint8_t ir_duty_cycle = 15;

// Global data variables
adc_data_t adc_data_1[DATA_BUFFER_SIZE];
adc_data_t adc_data_2[DATA_BUFFER_SIZE];
adc_data_t red_peaks[DATA_BUFFER_SIZE];
adc_data_t ir_peaks[DATA_BUFFER_SIZE];
adc_data_t red_troughs[DATA_BUFFER_SIZE];
adc_data_t ir_troughs[DATA_BUFFER_SIZE];
uint16_t buffer_index = 0;
uint32_t last_ADC_sample_time = 0;
uint32_t last_AGC_update_time = 0;

// Peak and trough variables for AGC
const uint16_t agc_upper_limit = 4095 * AGC_UPPER_LIMIT / 3.3;
const uint16_t agc_lower_limit = 4095 * AGC_LOWER_LIMIT / 3.3;
const uint16_t agc_tolerance = 4095 * AGC_TOLERANCE / 3.3;
const uint16_t red_peak_decay = 4095 * PEAK_DETECTOR_DECAY / SAMPLE_RATE;
const uint16_t ir_peak_decay = 4095 * PEAK_DETECTOR_DECAY / SAMPLE_RATE;
const uint16_t red_trough_decay = 4095 * PEAK_DETECTOR_DECAY / SAMPLE_RATE;
const uint16_t ir_trough_decay = 4095 * PEAK_DETECTOR_DECAY / SAMPLE_RATE;
uint16_t red_peak_value = 0;
uint16_t red_trough_value = 4095;
uint16_t ir_peak_value = 0;
uint16_t ir_trough_value = 4095;
uint32_t red_peak_time = 0;
uint32_t ir_peak_time = 0;
uint32_t red_trough_time = 0;
uint32_t ir_trough_time = 0;
bool just_increased_red = false;
bool just_decreased_red = false;
bool just_increased_ir = false;
bool just_decreased_ir = false;
bool red_peak = false;
bool ir_peak = false;
bool red_trough = false;
bool ir_trough = false;

// Heart rate and blood oxygen saturation variables
uint32_t heart_period = 0;
uint32_t heart_periods[AVERAGE_WINDOW];
uint8_t heart_period_index = 0;
uint8_t oxygen_values[AVERAGE_WINDOW];
uint8_t oxygen_index = 0;

uint32_t heartbeat_blink_time = 0;
bool heartbeat_blink = false;

void ISR(void);

void pwm_pulseox(uint16_t frequency, uint8_t red, uint8_t ir);

void collect_adc_data(void);

void setup() {
    Serial.begin(SERIAL_BAUD);
    SPI.begin();
    SPI.setSCK(ADC_SCK);
    SPI.setMOSI(ADC_MOSI);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_IR, OUTPUT);
    pinMode(PIN_SWITCH_RED, OUTPUT);
    pinMode(PIN_SWITCH_IR, OUTPUT);
    pinMode(ADC_CS, OUTPUT);
    // Make ISR run every 1 us
    timer.begin(ISR, 1);
}

void loop() {
    if (heartbeat_blink && (millis() - heartbeat_blink_time >= LED_BLINK_DURATION)) {
        digitalWrite(LED_BUILTIN, LOW);
        heartbeat_blink = false;
    }
    if (millis() - last_ADC_sample_time >= 1000 / SAMPLE_RATE) {
        last_ADC_sample_time = millis();

        if (buffer_index < DATA_BUFFER_SIZE) {
            // Get data from ADC
            collect_adc_data();

            // Update peak and trough values for AGC
            uint16_t red_value = (adc_data_1[buffer_index - 1].data_MSB << 8) | adc_data_1[buffer_index - 1].data_LSB;
            uint16_t ir_value = (adc_data_2[buffer_index - 1].data_MSB << 8) | adc_data_2[buffer_index - 1].data_LSB;

            if (red_value > red_peak_value) {
                red_peak_value = red_value;
                just_increased_red = true;
            } else {
                if (just_increased_red) {
                    if (millis() - red_peak_time >= MIN_PEAK_INTERVAL) {
                        // Calculate heartbeat period
                        heart_period = millis() - red_peak_time;
                        red_peak = true;
                        red_peak_time = millis();
                        // Update heart rate
                        heart_periods[heart_period_index] = heart_period;
                        heart_period_index = (heart_period_index + 1) % AVERAGE_WINDOW;
                        digitalWrite(LED_BUILTIN, HIGH);
                        heartbeat_blink_time = millis();
                        heartbeat_blink = true;
                    }
                    just_increased_red = false;
                } else {
                    red_peak = false;
                    red_peak_value = max(red_peak_value - red_peak_decay, 0);
                }
            }

            if (red_value <= red_trough_value) {
                red_trough_value = red_value;
                just_decreased_red = true;
                red_trough = false;
            } else {
                if (just_decreased_red) {
                    if (millis() - red_trough_time >= MIN_PEAK_INTERVAL) {
                        red_trough = true;
                        red_trough_time = millis();
                    }
                    just_decreased_red = false;
                } else {
                    red_trough = false;
                    red_trough_value = min(red_trough_value + red_trough_decay, 4095);
                }
            }

            if (ir_value >= ir_peak_value) {
                ir_peak_value = ir_value;
                just_increased_ir = true;
                ir_peak = false;
            } else {
                if (just_increased_ir) {
                    if (millis() - ir_peak_time >= MIN_PEAK_INTERVAL) {
                        ir_peak = true;
                        ir_peak_time = millis();
                    }
                    just_increased_ir = false;
                } else {
                    ir_peak = false;
                    ir_peak_value = max(ir_peak_value - ir_peak_decay, 0);
                }
            }

            if (ir_value <= ir_trough_value) {
                ir_trough_value = ir_value;
                just_decreased_ir = true;
                ir_trough = false;
            } else {
                if (just_decreased_ir) {
                    if (millis() - ir_trough_time >= MIN_PEAK_INTERVAL) {
                        ir_trough = true;
                        ir_trough_time = millis();
                    }
                    just_decreased_ir = false;
                } else {
                    ir_trough = false;
                    ir_trough_value = min(ir_trough_value + ir_trough_decay, 4095);
                }
            }

            red_peaks[buffer_index - 1].data_MSB = red_peak_value >> 8;
            red_peaks[buffer_index - 1].data_LSB = red_peak_value & 0xFF;
            red_peaks[buffer_index - 1].channel = 7;
            red_peaks[buffer_index - 1].timestamp = millis();

            ir_peaks[buffer_index - 1].data_MSB = ir_peak_value >> 8;
            ir_peaks[buffer_index - 1].data_LSB = ir_peak_value & 0xFF;
            ir_peaks[buffer_index - 1].channel = 9;
            ir_peaks[buffer_index - 1].timestamp = millis();

            red_troughs[buffer_index - 1].data_MSB = red_trough_value >> 8;
            red_troughs[buffer_index - 1].data_LSB = red_trough_value & 0xFF;
            red_troughs[buffer_index - 1].channel = 11;
            red_troughs[buffer_index - 1].timestamp = millis();

            ir_troughs[buffer_index - 1].data_MSB = ir_trough_value >> 8;
            ir_troughs[buffer_index - 1].data_LSB = ir_trough_value & 0xFF;
            ir_troughs[buffer_index - 1].channel = 12;
            ir_troughs[buffer_index - 1].timestamp = millis();
        } else {
            // Calculate oxygen saturation
            uint16_t red_ac = red_peak_value - red_trough_value;
            uint16_t ir_ac = ir_peak_value - ir_trough_value;
            uint8_t spo2 = 100 * (log10(red_ac) / log10(ir_ac));

            // Add oxygen value to average window
            oxygen_values[oxygen_index] = spo2;
            oxygen_index = (oxygen_index + 1) % AVERAGE_WINDOW;

            // Calculate average oxygen value
            uint16_t sum = 0;
            for (uint8_t i = 0; i < AVERAGE_WINDOW; i++) {
                sum += oxygen_values[i];
            }
            spo2 = sum / AVERAGE_WINDOW;

            // Calculate average heart period
            sum = 0;
            for (uint8_t i = 0; i < AVERAGE_WINDOW; i++) {
                sum += heart_periods[i];
            }
            heart_period = sum / AVERAGE_WINDOW;

            // Calculate heart rate
            uint8_t bpm = 60000 / heart_period;

            adc_data_t bpm_spo2_packet;
            bpm_spo2_packet.data_MSB = bpm;
            bpm_spo2_packet.data_LSB = spo2;
            bpm_spo2_packet.channel = 5;

            adc_data_t heart_period_packet;
            heart_period_packet.data_MSB = heart_period >> 8;
            heart_period_packet.data_LSB = heart_period & 0xFF;
            heart_period_packet.channel = 8;

            adc_data_t duty_cycle_packet;
            duty_cycle_packet.data_MSB = red_duty_cycle;
            duty_cycle_packet.data_LSB = ir_duty_cycle;
            duty_cycle_packet.channel = 3;

            // Send data over serial

            // Channels list
            // 1: Red ADC data
            // 2: IR ADC data
            // 3: Red and IR duty cycles
            // 4: Red peak values
            // 5: Heartrate and oxygen saturation
            // 6: Unused
            // 7: Red peak values
            // 8: Heart period
            // 9: IR peak values
            // 10: Unused
            // 11: Red trough values
            // 12: IR trough values

            Serial.print("DATA_START\n");
            Serial.write((uint8_t *)adc_data_1, sizeof(adc_data_1));
            Serial.write((uint8_t *)adc_data_2, sizeof(adc_data_2));
            Serial.write((uint8_t *)red_peaks, sizeof(red_peaks));
            Serial.write((uint8_t *)ir_peaks, sizeof(ir_peaks));
            Serial.write((uint8_t *)red_troughs, sizeof(red_troughs));
            Serial.write((uint8_t *)ir_troughs, sizeof(ir_troughs));
            Serial.write((uint8_t *)&bpm_spo2_packet, sizeof(bpm_spo2_packet));
            Serial.write((uint8_t *)&heart_period_packet, sizeof(heart_period_packet));
            Serial.write((uint8_t *)&duty_cycle_packet, sizeof(duty_cycle_packet));
            Serial.print("DATA_END\n");
            buffer_index = 0;
        }
    }

    // if (millis() - last_AGC_update_time >= agc_update_interval) {
    if (millis() - last_AGC_update_time >= AGC_UPDATE_INTERVAL) {
        last_AGC_update_time = millis();
        bool can_increase_red = red_peak_value < agc_upper_limit - agc_tolerance && red_trough_value > agc_lower_limit + agc_tolerance;
        bool should_decrease_red = red_peak_value > agc_upper_limit || red_trough_value < agc_lower_limit;
        bool can_increase_ir = ir_peak_value < agc_upper_limit - agc_tolerance && ir_trough_value > agc_lower_limit + agc_tolerance;
        bool should_decrease_ir = ir_peak_value > agc_upper_limit || ir_trough_value < agc_lower_limit;

        if (can_increase_red && !should_decrease_red) {
            noInterrupts();
            red_duty_cycle = min(red_duty_cycle + DUTY_CYCLE_STEP, MAX_DUTY_CYCLE);
            interrupts();
        } else if (should_decrease_red) {
            noInterrupts();
            red_duty_cycle = max(red_duty_cycle - DUTY_CYCLE_STEP, MIN_DUTY_CYCLE);
            interrupts();
        }

        if (can_increase_ir && !should_decrease_ir) {
            noInterrupts();
            ir_duty_cycle = min(ir_duty_cycle + DUTY_CYCLE_STEP, MAX_DUTY_CYCLE);
            interrupts();
        } else if (should_decrease_ir) {
            noInterrupts();
            ir_duty_cycle = max(ir_duty_cycle - DUTY_CYCLE_STEP, MIN_DUTY_CYCLE);
            interrupts();
        }
    }
}

void ISR() {
    pwm_pulseox(1000, red_duty_cycle, ir_duty_cycle);
}

/**
 * @brief PWM signal generator for pulse oximeter LEDs
 *
 * @param frequency Frequency of the PWM signal in Hz
 * @param red Duty cycle for red LED in percent
 * @param ir Duty cycle for IR LED in percent
 */
void pwm_pulseox(uint16_t frequency, uint8_t red, uint8_t ir) {
    // States:
    // 1: Both low, Both LEDs off
    // 2: Red low, IR high, IR LED on
    // 3: Both low, Both LEDs off
    // 4: Red high, IR low, RED LED on

    static bool setup = false;
    static uint32_t lastTime = 0;
    static uint8_t state = 1;

    // Start in state 1 (Both LEDs off)
    if (!setup) {
        digitalWrite(PIN_LED_RED, HIGH);
        digitalWrite(PIN_LED_IR, LOW);
        digitalWrite(PIN_SWITCH_RED, HIGH);
        digitalWrite(PIN_SWITCH_IR, LOW);
        setup = true;
        lastTime = micros();
    }

    // Calculate total period in microseconds and high and low time for each of 4 states
    uint32_t period = 1000000 / frequency;  // Period in microseconds

    // Calculate high times for red and IR LEDs
    uint32_t state2_time = period * ir / 100;
    uint32_t state4_time = period * red / 100;
    // Calculate low times for red and IR LEDs
    uint32_t state3_time = (period * 0.5) - state2_time;
    uint32_t state1_time = (period * 0.5) - state4_time;

    uint32_t currentTime = micros();

    switch (state) {
        case 1:
            // Enter state 2
            if (currentTime - lastTime >= state1_time) {
                digitalWrite(PIN_LED_IR, HIGH);
                digitalWrite(PIN_SWITCH_IR, HIGH);
                state = 2;
                lastTime = currentTime;
            }
            break;
        case 2:
            // Enter state 3
            if (currentTime - lastTime >= state2_time) {
                digitalWrite(PIN_LED_IR, LOW);
                digitalWrite(PIN_SWITCH_IR, LOW);
                state = 3;
                lastTime = currentTime;
            }
            break;
        case 3:
            // Enter state 4
            if (currentTime - lastTime >= state3_time) {
                digitalWrite(PIN_LED_RED, HIGH);
                digitalWrite(PIN_SWITCH_RED, HIGH);
                state = 4;
                lastTime = currentTime;
            }
            break;
        case 4:
            // Enter state 1
            if (currentTime - lastTime >= state4_time) {
                digitalWrite(PIN_LED_RED, LOW);
                digitalWrite(PIN_SWITCH_RED, LOW);
                state = 1;
                lastTime = currentTime;
            }
            break;
    }
}

/**
 * @brief Reads channels 1 and 2 of the ADC and stores the data
 */
void collect_adc_data(void) {
    uint8_t MSB;
    uint8_t LSB;

    // Read channel 1
    SPI.setMISO(ADC_MISO_1);
    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(ADC_CS, LOW);
    MSB = SPI.transfer(0);  // grab upper byte
    LSB = SPI.transfer(0);  // grab lower byte
    digitalWrite(ADC_CS, HIGH);
    SPI.endTransaction();
    adc_data_1[buffer_index].data_MSB = MSB;
    adc_data_1[buffer_index].data_LSB = LSB;
    adc_data_1[buffer_index].channel = 1;
    adc_data_1[buffer_index].timestamp = millis();

    // Read channel 2
    SPI.setMISO(ADC_MISO_2);
    SPI.beginTransaction(SPI_SETTINGS);
    digitalWrite(ADC_CS, LOW);
    MSB = SPI.transfer(0);  // grab upper byte
    LSB = SPI.transfer(0);  // grab lower byte
    digitalWrite(ADC_CS, HIGH);
    SPI.endTransaction();
    adc_data_2[buffer_index].data_MSB = MSB;
    adc_data_2[buffer_index].data_LSB = LSB;
    adc_data_2[buffer_index].channel = 2;
    adc_data_2[buffer_index].timestamp = millis();
    buffer_index++;
}