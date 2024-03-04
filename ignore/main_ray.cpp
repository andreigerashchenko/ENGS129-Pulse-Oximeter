#include <Arduino.h>
#include <stdbool.h>
#include <math.h>
#include <PeakDetection.h> // https://github.com/leandcesar/PeakDetection/tree/master

PeakDetection peakDetection;                     // create PeakDetection object

#define PIN_LED_RED 35
#define PIN_LED_IR 36
#define PIN_SWITCH_RED 29
#define PIN_SWITCH_IR 30
#define deltaDuty 1
#define THRESHOLD (pow(3.28,2))/2
#define UPPER (pow(3.279,2))/2
#define LOWER (pow(3.24,2))/2

int DutyRed = 25;
int DutyIR = 25;
double LED_RED_Value = 0;
double LED_IR_Value = 0;
double times[] = {0,0};

void data_extraction(double time1, double time2, double amp1, double amp2);
double findPeak(double data);
double findPeakTime(double data);

void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_IR, OUTPUT);
    pinMode(PIN_SWITCH_RED, OUTPUT);
    pinMode(PIN_SWITCH_IR, OUTPUT);
    peakDetection.begin(48, 2, 0.6);               // sets the lag, threshold and influence
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
       

    pwm_pulseox(1000, DutyRed, DutyIR);
    
    double liveData_Red[] = ; // collect data from the ADC
    double liveData_IR[] = ; // collect data from the ADC
    double data_time[] = ; // collect time 


    // Collect max value of the received (squared) signal  (peak detection alogrithm found online?)
    LED_RED_Value = findPeak(liveData_Red);// utilize data collected once Andrei gets that data max peak value for the last 3 seconds (take the last peak value)
    LED_IR_Value = findPeak(liveData_IR); // same as above (max function?) (squared signal so intensity)
    

    // If either of the (squared) LED signals are saturated, reduce the duty cycle
    if (LED_RED_Value >= THRESHOLD && LED_IR_Value >= THRESHOLD) {
        DutyIR -= deltaDuty;
        DutyRed -= deltaDuty;
        }
    else if (LED_RED_Value >= THRESHOLD) {
        DutyRed -= deltaDuty;
        }
    else if (LED_IR_Value >= THRESHOLD) {
        DutyIR -= deltaDuty;
        }
    // If the (squared) signals are just right, keep the duty cycle the same
    else if (LED_RED_Value <= UPPER && LED_RED_Value >= LOWER && LED_IR_Value <= UPPER && LED_IR_Value >= LOWER) {
        DutyRed = DutyRed;
        DutyIR = DutyIR;
    }
    else if (LED_RED_Value <= UPPER && LED_RED_Value >= LOWER) {
        DutyRed = DutyRed;
    }
    else if (LED_IR_Value <= UPPER && LED_IR_Value >= LOWER) {
        DutyIR = DutyIR;
    }
    // If either of the (squared) signals are too weak, increase the duty cycle
    else {
        DutyRed += deltaDuty;
        DutyIR += deltaDuty;
    }
    // Ensure the duty cycle is within the acceptable range
    if (DutyRed + DutyIR > 80) {
        DutyRed = 25;
        DutyIR = 25;
    }
    else if (DutyRed + DutyIR <= 0) {
        DutyRed = 25;
        DutyIR = 25;
    }

    // add something here to extract time
    // time list should only hold two values, once Peak detection code is made, it will be added here
    times[1], times[0] = findPeakTime(data_time); // time of the last peak and time of the peak before the last peak
    
    data_extraction(times[0], times[1], LED_RED_Value, LED_IR_Value);
    
}


void data_extraction(double time1, double time2, double amp1, double amp2) {
    // extract the heart rate by collecting the time in seconds of each peak and taking the difference between each peak
    // Heart Rate (BPM) = 60 / (Time between consecutive R waves in seconds)
    // extract the oxygen saturation by taking the ratio of the red and IR LED signals
    // ratio of the light intensity to find oxygen saturation (R * 100) R = log10(Intensity of light of AC Red) divided by
    // log10(Intensity of light of AC IR); (amplitude should already be in intensity from squared signal)
    // Amp 1 is the red LED signal intensity and Amp 2 is the IR LED signal instensity
    // time2 is the time of the last peak and time1 is the time of the peak before the last peak

    double heart_rate = 60/(time2-time1);

    printf("Heart Rate: %f\n", heart_rate);
    
    double oxygen_saturation = (log10(amp1/amp2))*100;

    printf("Oxygen Saturation: %f\n", oxygen_saturation);

    return;
}

double findPeak(double data) {
    // Peak detection algorithm
    int peak = 0;
    while (peak != 1) {
        peakDetection.add(data);                     // adds a new data point
        peak = peakDetection.getPeak();          // 0, 1 or -1
        double filtered = peakDetection.getFilt();   // moving average
        Serial.print(data);                          // print data
        Serial.print(",");
        Serial.print(peak);                          // print peak status
        Serial.print(",");
        Serial.println(filtered);                    // print moving average

        if (peak == 1) {
            double peak_live = data;
        }
    }
     // return the peak value

    return peak_live;
}

double findPeakTime(double data) {
    // Peak detection algorithm
    int peak = 0;
    while (peak != 1) {
        peakDetection.add(data);                     // adds a new data point
        peak = peakDetection.getPeak();          // 0, 1 or -1
        double filtered = peakDetection.getFilt();   // moving average
        Serial.print(data);                          // print data
        Serial.print(",");
        Serial.print(peak);                          // print peak status
        Serial.print(",");
        Serial.println(filtered);                    // print moving average

        if (peak == 1) {
            double peak1 = data;
            
        }
    }
    int peak = 0;
    while (peak != 1) {
        peakDetection.add(data);                     // adds a new data point
        peak = peakDetection.getPeak();          // 0, 1 or -1
        double filtered = peakDetection.getFilt();   // moving average
        Serial.print(data);                          // print data
        Serial.print(",");
        Serial.print(peak);                          // print peak status
        Serial.print(",");
        Serial.println(filtered);                    // print moving average

        if (peak == 1) {
            double peak2 = data;
            
        }
    }
     // return the peak values
    return {peak1, peak2};
}

  




