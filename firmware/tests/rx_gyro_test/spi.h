#include <SPI.h>

uint8_t readRegister(uint8_t reg, uint8_t CS)
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(CS, LOW);

    SPI.transfer(reg | 0x80);  // Read bit
    uint8_t value = SPI.transfer(0x00);

    digitalWrite(CS, HIGH);
    SPI.endTransaction();

    return value;
}

int16_t read2Bytes(uint8_t reg, uint8_t CS)
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(CS, LOW);

    SPI.transfer(reg | 0x80);  // Read bit
    uint8_t value_H = SPI.transfer(0x00);
    uint8_t value_L = SPI.transfer(0x00);

    digitalWrite(CS, HIGH);
    SPI.endTransaction();

    return (int16_t)(value_H << 8 | value_L);
}

void writeRegister(uint8_t reg, uint8_t hexValue, uint8_t CS)
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(CS, LOW);

    SPI.transfer(reg);
    SPI.transfer(hexValue);

    digitalWrite(CS, HIGH);
    SPI.endTransaction();
}