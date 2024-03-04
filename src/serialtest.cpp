#include <Arduino.h>
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
// Want to keep ADC peak reading at 3.0V, 12-bit ADC, 4095 * (3.0 / 3.3) = 3723
#define SETPOINT_READING 3723
// Tolerance for the setpoint reading = 4095 * (0.075 / 3.3) = 93
#define SETPOINT_TOLERANCE 93
#define DUTY_CYCLE_UPDATE_PERIOD 2500  // milliseconds to wait before updating duty cycle again
uint16_t last_duty_cycle_update = 0;

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

uint32_t last_read_time = 0;
adc_data_t send_buffer_1[SEND_BUFFER_SIZE];
adc_data_t send_buffer_2[SEND_BUFFER_SIZE];
uint16_t buffer_index = 0;

uint16_t peak_value = SETPOINT_READING;
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
    // Period of 10 kHz in microseconds = 1 / 10 kHz * 1e6 = 100
    pwm_timer.begin(pwm_interrupt_handler, 100);
}

void loop() {
    // pwm_pulseox(1000, red_duty_cycle, ir_duty_cycle);
    // Read from the ADC at the specified sample rate and save the data to the send buffer
    // Send when the buffer is full
    if (buffer_index == SEND_BUFFER_SIZE) {
        // Adjust duty cycle based on peak value
        // Find highest value that lasts for at least 3 samples
        uint16_t max_value = 0;

        for (uint16_t i = 2; i < SEND_BUFFER_SIZE; i++) {
            // If current value and previous two values are greater than the max value, update max value and index
            if ((send_buffer_1[i].data_MSB << 8 | send_buffer_1[i].data_LSB) > max_value &&
                (send_buffer_1[i - 1].data_MSB << 8 | send_buffer_1[i - 1].data_LSB) > max_value &&
                (send_buffer_1[i - 2].data_MSB << 8 | send_buffer_1[i - 2].data_LSB) > max_value) {
                max_value = send_buffer_1[i].data_MSB << 8 | send_buffer_1[i].data_LSB;
            }
        }

        if (millis() - last_duty_cycle_update >= DUTY_CYCLE_UPDATE_PERIOD) {
            last_duty_cycle_update = millis();

            if (max_value > SETPOINT_READING + SETPOINT_TOLERANCE) {
                // Values are too high, decrease duty cycle
                noInterrupts();
                red_duty_cycle = max(MIN_DUTY_CYCLE, red_duty_cycle - DUTY_CYCLE_STEP);
                ir_duty_cycle = max(MIN_DUTY_CYCLE, ir_duty_cycle - DUTY_CYCLE_STEP);
                interrupts();
            } else if (max_value < SETPOINT_READING - SETPOINT_TOLERANCE) {
                // Values are too low, increase duty cycle
                noInterrupts();
                red_duty_cycle = min(MAX_DUTY_CYCLE, red_duty_cycle + DUTY_CYCLE_STEP);
                ir_duty_cycle = min(MAX_DUTY_CYCLE, ir_duty_cycle + DUTY_CYCLE_STEP);
                interrupts();
            }
        }

        adc_data_t duty_cycle_packet;
        duty_cycle_packet.timestamp = 0;
        duty_cycle_packet.data_MSB = red_duty_cycle;
        duty_cycle_packet.data_LSB = ir_duty_cycle;
        duty_cycle_packet.channel = 3;
        adc_data_t peak_packet;
        peak_packet.timestamp = 0;
        peak_packet.data_MSB = max_value >> 8;
        peak_packet.data_LSB = max_value & 0xFF;
        peak_packet.channel = 4;
        Serial.println("DATA_START\n");
        // Serial.write((uint8_t*)send_buffer_1, sizeof(send_buffer_1));
        // Serial.write((uint8_t*)send_buffer_2, sizeof(send_buffer_2));
        Serial.write((uint8_t*)&duty_cycle_packet, sizeof(duty_cycle_packet));
        Serial.write((uint8_t*)&peak_packet, sizeof(peak_packet));
        buffer_index = 0;
        peak_value = 0;
        Serial.println("DATA_END\n");
    } else if ((millis() - last_read_time) >= (1000 / SAMPLE_RATE)) {
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
        send_buffer_1[buffer_index].data_MSB = MSB;
        send_buffer_1[buffer_index].data_LSB = LSB;
        send_buffer_1[buffer_index].channel = 1;
        send_buffer_1[buffer_index].timestamp = millis();

        if ((MSB << 8 | LSB) > peak_value) {
            peak_value = MSB << 8 | LSB;
        }

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

        // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

/**
 * @brief
 *
 */
void pwm_interrupt_handler() {
    volatile uint8_t red = red_duty_cycle;
    volatile uint8_t ir = ir_duty_cycle;
    pwm_pulseox(1000, red, ir);
    // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

/**
 * @brief PWM signal generator for pulse oximeter LEDs
 *
 * @param frequency Frequency of the PWM signal in Hz
 */
void pwm_pulseox(uint16_t frequency, uint8_t red, uint8_t black) {
    static bool setup = false;
    static uint32_t period = 1000000 / frequency;  // Period in milliseconds
    static uint32_t lastTime = 0;
    uint32_t highRed = period * red / 100;                   // High time of the red LED in milliseconds
    uint32_t highBlack = period * black / 100;               // High time of the IR LED in milliseconds
    uint32_t deadTime = (period - highRed - highBlack) / 2;  // Total dead time between both signals in milliseconds

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