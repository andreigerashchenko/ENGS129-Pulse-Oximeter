#include <Arduino.h>
#include <TimerThree.h>
// #include "adc.h"
#include "SPI.h"

#define SERIAL_BAUD 2000000
#define SPI_RATE 1000000
#define SAMPLE_RATE 60         // Hz
#define RECORDING_PERIOD 2000  // milliseconds

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
#define AGC_UPPER_LIMIT 3.0            // Upper limit of input signal (Volts)
#define AGC_LOWER_LIMIT 0.3            // Lower limit of input signal (Volts)
#define AGC_TOLERANCE 0.075            // Tolerance for AGC (Volts)
#define DUTY_CYCLE_UPDATE_PERIOD 2500  // milliseconds to wait before updating duty cycle again

// Convert limits and tolerance to 12-bit ADC values
const uint16_t AGC_upper = (uint16_t)(AGC_UPPER_LIMIT * 4095 / 3.3);
const uint16_t AGC_lower = (uint16_t)(AGC_LOWER_LIMIT * 4095 / 3.3);
const uint16_t AGC_tolerance = (uint16_t)(AGC_TOLERANCE * 4095 / 3.3);

typedef struct {
    uint32_t timestamp;  // Timestamp in milliseconds
    uint8_t data_MSB;    // Most significant byte of the ADC reading
    uint8_t data_LSB;    // Least significant byte of the ADC reading
    uint8_t channel;     // 1 for channel 1, 2 for channel 2
    char end = '\n';     // Newline character to delimit data
} adc_data_t;

// SEND_BUFFER_SIZE is samples collected per second * number of seconds
#define SEND_BUFFER_SIZE ((SAMPLE_RATE * RECORDING_PERIOD) / 1000)

SPISettings settings(SPI_RATE, MSBFIRST, SPI_MODE2);  // 1MHz, MSB, mode 2

volatile uint8_t state = 0;
IntervalTimer pwm_timer;

uint16_t last_duty_cycle_update = 0;

uint32_t last_read_time = 0;
adc_data_t send_buffer_1[SEND_BUFFER_SIZE];
adc_data_t send_buffer_2[SEND_BUFFER_SIZE];
uint16_t buffer_index = 0;

uint16_t peak_red_value = AGC_lower;
uint16_t peak_ir_value = AGC_lower;
uint16_t min_red_value = AGC_upper;
uint16_t min_ir_value = AGC_upper;
uint32_t peak_red_time = 0;
volatile uint8_t red_duty_cycle = 25;
volatile uint8_t ir_duty_cycle = 25;

void pwm_pulseox(uint16_t frequency, uint8_t red, uint8_t black);
void pwm_interrupt_handler(void);

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
    // handshake();
    // Period of 100 kHz in microseconds = 1 / 100 kHz * 1e6 = 10
    // pwm_timer.begin(pwm_interrupt_handler, 100);
    // Timer3.initialize(25);
    // 250 millisecond period
    Timer3.initialize(250000);
    Timer3.attachInterrupt(pwm_interrupt_handler);
}

void read_adc() {
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
    send_buffer_1[buffer_index].data_MSB = MSB;
    send_buffer_1[buffer_index].data_LSB = LSB;
    send_buffer_1[buffer_index].channel = 1;
    send_buffer_1[buffer_index].timestamp = millis();

    // Read channel 2
    SPI.setMISO(ADC_MISO_2);
    SPI.beginTransaction(settings);
    digitalWrite(ADC_CS, LOW);
    MSB = SPI.transfer(0);  // grab upper byte
    LSB = SPI.transfer(0);  // grab lower byte
    digitalWrite(ADC_CS, HIGH);
    SPI.endTransaction();
    send_buffer_2[buffer_index].data_MSB = MSB;
    send_buffer_2[buffer_index].data_LSB = LSB;
    send_buffer_2[buffer_index].channel = 2;
    send_buffer_2[buffer_index].timestamp = millis();
    buffer_index++;
}

void loop() {
    // Data has been collected, process and send results over serial
    // if (buffer_index == SEND_BUFFER_SIZE) {
    //     uint16_t max_red_value = 0;
    //     uint16_t max_ir_value = 0;
    //     uint16_t min_red_value = 4095;
    //     uint16_t min_ir_value = 4095;

    //     uint32_t heartrate_period = 0;
    //     uint32_t current_red_peak = 0;

    //     for (uint16_t i = 2; i < SEND_BUFFER_SIZE; i++) {
    //         // Find maximum value of red signal
    //         if ((send_buffer_1[i].data_MSB << 8 | send_buffer_1[i].data_LSB) > max_red_value) {
    //             max_red_value = send_buffer_1[i].data_MSB << 8 | send_buffer_1[i].data_LSB;
    //             current_red_peak = send_buffer_1[i].timestamp;
    //         }

    //         // Find maximum value of IR signal
    //         if ((send_buffer_2[i].data_MSB << 8 | send_buffer_2[i].data_LSB) > max_ir_value) {
    //             max_ir_value = send_buffer_2[i].data_MSB << 8 | send_buffer_2[i].data_LSB;
    //         }

    //         // Find minimum value of red signal
    //         if ((send_buffer_1[i].data_MSB << 8 | send_buffer_1[i].data_LSB) < min_red_value) {
    //             min_red_value = send_buffer_1[i].data_MSB << 8 | send_buffer_1[i].data_LSB;
    //         }

    //         // Find minimum value of IR signal
    //         if ((send_buffer_2[i].data_MSB << 8 | send_buffer_2[i].data_LSB) < min_ir_value) {
    //             min_ir_value = send_buffer_2[i].data_MSB << 8 | send_buffer_2[i].data_LSB;
    //         }
    //     }

    //     heartrate_period = current_red_peak - peak_red_time;
    //     peak_red_time = current_red_peak;

    //     if (millis() - last_duty_cycle_update >= DUTY_CYCLE_UPDATE_PERIOD) {
    //         last_duty_cycle_update = millis();

    //         if (max_red_value > AGC_upper + AGC_tolerance) {
    //             // Values are too high, decrease duty cycle
    //             noInterrupts();
    //             red_duty_cycle = max(MIN_DUTY_CYCLE, red_duty_cycle - DUTY_CYCLE_STEP);
    //             interrupts();
    //         } else if (max_red_value < AGC_upper - AGC_tolerance) {
    //             // Values are too low, increase duty cycle
    //             noInterrupts();
    //             red_duty_cycle = min(MAX_DUTY_CYCLE, red_duty_cycle + DUTY_CYCLE_STEP);
    //             interrupts();
    //         }
    //     }

    //     adc_data_t duty_cycle_packet;
    //     duty_cycle_packet.timestamp = 0;
    //     duty_cycle_packet.data_MSB = red_duty_cycle;
    //     duty_cycle_packet.data_LSB = ir_duty_cycle;
    //     duty_cycle_packet.channel = 3;
    //     adc_data_t peak_packet;
    //     peak_packet.timestamp = 0;
    //     peak_packet.data_MSB = max_red_value >> 8;
    //     peak_packet.data_LSB = max_red_value & 0xFF;
    //     peak_packet.channel = 4;

    //     adc_data_t heartrate_packet;
    //     heartrate_packet.timestamp = 0;
    //     uint8_t heartrate = (uint8_t)(60000 / heartrate_period);
    //     heartrate_packet.data_MSB = 0;
    //     heartrate_packet.data_LSB = heartrate;
    //     heartrate_packet.channel = 5;

    //     adc_data_t oxygen_packet;
    //     oxygen_packet.timestamp = 0;
    //     uint8_t oxygen = (uint8_t)(100 * (peak_ir_value - min_ir_value) / (peak_red_value - min_red_value));
    //     oxygen_packet.data_MSB = 0;
    //     oxygen_packet.data_LSB = oxygen;
    //     oxygen_packet.channel = 6;

    //     Serial.println("DATA_START\n");
    //     Serial.write((uint8_t*)send_buffer_1, sizeof(send_buffer_1));
    //     Serial.write((uint8_t*)send_buffer_2, sizeof(send_buffer_2));
    //     Serial.write((uint8_t*)&duty_cycle_packet, sizeof(duty_cycle_packet));
    //     Serial.write((uint8_t*)&peak_packet, sizeof(peak_packet));
    //     Serial.write((uint8_t*)&heartrate_packet, sizeof(heartrate_packet));
    //     Serial.write((uint8_t*)&oxygen_packet, sizeof(oxygen_packet));
    //     buffer_index = 0;
    //     peak_red_value = 0;
    //     Serial.println("DATA_END\n");
    // } else if ((millis() - last_read_time) >= (1000 / SAMPLE_RATE)) {
    //     last_read_time = millis();

    //     // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // }
}

/**
 * @brief
 *
 */
void pwm_interrupt_handler() {
    volatile uint8_t red = red_duty_cycle;
    volatile uint8_t ir = ir_duty_cycle;
    // pwm_pulseox(1000, red, ir);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

/**
 * @brief PWM signal generator for pulse oximeter LEDs
 *
 * @param frequency Frequency of the PWM signal in Hz
 */
void pwm_pulseox(uint16_t frequency, uint8_t red, uint8_t black) {
    static bool setup = false;
    static uint32_t lastTime = 0;
    // Calculate total period in microseconds and high and low time for each of 4 states
    static uint32_t period = 1000000 / frequency;  // Period in microseconds

    // Calculate time in microseconds for red LED to be on based on duty cycle
    uint32_t state1_time = period * red / 100;
    // Calculate time in microseconds for IR LED to be on based on duty cycle
    uint32_t state3_time = period * black / 100;
    // Calculate time between red on and IR on
    uint32_t state2_time = period * 0.5 - state1_time;
    // Calculate time between IR on and red on
    uint32_t state4_time = period * 0.5 - state3_time;

    // States:
    // 1: Red high, black low, RED LED on
    // 2: Both low, Both LEDs off
    // 3: Red low, black high, IR LED on
    // 4: Both low, Both LEDs off

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
        case 1:
            if (currentTime - lastTime >= state1_time) {
                digitalWrite(PIN_LED_RED, LOW);
                digitalWrite(PIN_SWITCH_RED, LOW);
                state = 2;
                lastTime = currentTime;
            }
            break;
        case 2:
            if (currentTime - lastTime >= state2_time) {
                digitalWrite(PIN_LED_IR, HIGH);
                digitalWrite(PIN_SWITCH_IR, HIGH);
                state = 3;
                lastTime = currentTime;
            }
            break;
        case 3:
            if (currentTime - lastTime >= state3_time) {
                digitalWrite(PIN_LED_IR, LOW);
                digitalWrite(PIN_SWITCH_IR, LOW);
                state = 4;
                lastTime = currentTime;
            }
            break;
        case 4:
            if (currentTime - lastTime >= state4_time) {
                digitalWrite(PIN_LED_RED, HIGH);
                digitalWrite(PIN_SWITCH_RED, HIGH);
                state = 1;
                lastTime = currentTime;
            }
            break;
    }
}