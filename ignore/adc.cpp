// This file is for reading from the Pmod AD1
// The Pmod AD1 is a 12-bit ADC with 2 channels
// Send as MSB first, with first 4 bits as 0

#include "adc.h"
#include "SPI.h"

void adc_init() {
    // Set up the SPI interface
    SPI.begin();
    SPI.setSCK(ADC_SCK);
    SPI.setCS(ADC_CS);
    SPI.setMISO(ADC_MISO_1);
    SPI.setMOSI(ADC_MOSI);
}

uint16_t adc_read(uint8_t channel) {
    SPISettings spisettings(1000000, MSBFIRST, SPI_MODE2);  // 1MHz, MSB, mode 2
    uint16_t data = 0x0000;
    // Start transaction
    SPI.beginTransaction(spisettings);
    // Lower the chip select and begin clocking data
    // Data is clocked out on the falling edge of the clock
    digitalWrite(ADC_CS, LOW);
    data = SPI.transfer(0x00);
    data = (data << 8) | SPI.transfer(0x00);
    // Raise the chip select to end the transaction
    digitalWrite(ADC_CS, HIGH);
    // End transaction
    SPI.endTransaction();

    // Mask the data to get the last 12 bits
    return data & 0x0FFF;
}