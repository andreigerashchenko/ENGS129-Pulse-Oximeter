#ifndef ADC_H
#define ADC_H

// This file is for reading from the Pmod AD1
// The Pmod AD1 is a 12-bit ADC with 2 channels
// Connect everything to SPI0 pins
// Data pins should be connected to separate MISO pins

#define ADC_CS 10 // Chip select
#define ADC_SCK 27 // Clock
#define ADC_MISO_1 8 // Channel 1 data
#define ADC_MISO_2 12 // Channel 2 data
#define ADC_MOSI 11 // Unused MOSI pin to keep SPI library happy

#include <Arduino.h>

void adc_init();
uint16_t adc_read(uint8_t channel);

#endif // ADC_H