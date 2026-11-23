#include <SPI.h>
#include "spi.h"

struct Gyro {
  float x;
  float y;
  float z;
};

constexpr uint8_t CS_IMU = 10;

float gyro_prescaler = 500.0;
float scale = (2*gyro_prescaler/65535.0);

Gyro gyroBias;


Gyro readGyro()
{
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(CS_IMU, LOW);

    SPI.transfer(0x43 | 0x80);  // Read
    uint8_t gX_out_H = SPI.transfer(0x00);
    uint8_t gX_out_L = SPI.transfer(0x00);
    uint8_t gY_out_H = SPI.transfer(0x00);
    uint8_t gY_out_L = SPI.transfer(0x00);
    uint8_t gZ_out_H = SPI.transfer(0x00);
    uint8_t gZ_out_L = SPI.transfer(0x00);

    digitalWrite(CS_IMU, HIGH);
    SPI.endTransaction();

    int16_t gX_out = (int16_t)(gX_out_H << 8 | gX_out_L);
    int16_t gY_out = (int16_t)(gY_out_H << 8 | gY_out_L);
    int16_t gZ_out = (int16_t)(gZ_out_H << 8 | gZ_out_L);

    // Serial.print("gX_out = ");
    // Serial.println(gX_out);
    // Serial.print("gY_out = ");
    // Serial.println(gY_out);
    // Serial.print("gZ_out = ");
    // Serial.println(gZ_out);

    float gyroX = gX_out * scale;
    float gyroY = gY_out * scale;
    float gyroZ = gZ_out * scale;

    Gyro gyro = {gyroX, gyroY, gyroZ};

    return gyro;
}

void computeBiasGyro()
{
    const float iterations = 1000.0;

    Gyro gyro = readGyro();
    float sumX = 0.0;
    float sumY = 0.0;
    float sumZ = 0.0;

    for(uint i = 0; i < iterations; i++){
        gyro = readGyro();
        sumX += gyro.x;
        sumY += gyro.y;
        sumZ += gyro.z;
        delayMicroseconds(1000);
    }

    gyroBias.x = sumX / iterations;
    gyroBias.y = sumY / iterations;
    gyroBias.z = sumZ / iterations;
}

Gyro deadBandFilter(Gyro gyro)
{
  Gyro filtered_gyro = gyro;
  if (fabs(gyro.x) < 0.05) filtered_gyro.x = 0;
  if (fabs(gyro.y) < 0.05) filtered_gyro.y = 0;
  if (fabs(gyro.z) < 0.05) filtered_gyro.z = 0;

  return filtered_gyro;
}

Gyro lowPassFilter(Gyro gyro)
{
  static Gyro state = {0,0,0};
  float alpha = 0.85f;

  state.x = alpha * state.x + (1.0f - alpha) * gyro.x;
  state.y = alpha * state.y + (1.0f - alpha) * gyro.y;
  state.z = alpha * state.z + (1.0f - alpha) * gyro.z;

  return state;
}

void setupGyro()
{
    // test communication with gyro
    uint8_t whoami = readRegister(0x75, CS_IMU);
    Serial.print("WHO_AM_I = 0x");
    Serial.println(whoami, HEX);

    //configure power of IMU
    writeRegister(0x6B, 0x01, CS_IMU);
    delay(50);
    uint8_t power2_cfg = readRegister(0x6C, CS_IMU);
    writeRegister(0x6C, 0b11000000 & power2_cfg, CS_IMU);
    delay(50);
    power2_cfg = readRegister(0x6C, CS_IMU);
    Serial.print("power2_cfg = 0x");
    Serial.println(power2_cfg, HEX);


    writeRegister(0x1B, 0b00001000, CS_IMU); // set gyro FS
    // writeRegister(0x1C, 0b00011000, CS_IMU); // set accelerometer FS
    delay(100);

    // check gyro config
    uint8_t gyro_cfg = readRegister(0x1B, CS_IMU);
    Serial.print("GYRO_CONFIG = 0x");
    Serial.println(gyro_cfg, HEX);

    uint8_t config = readRegister(0x1A, CS_IMU);
    writeRegister(0x1A, 0b00000001, CS_IMU);
    delay(1);
    config = readRegister(0x1A, CS_IMU);
    Serial.print("config = 0b");
    Serial.println(config, BIN);

    // compute bias of the gyro
    computeBiasGyro();
}

Gyro filterGyro(Gyro raw)
{
  Gyro unbiasedGyro = {raw.x - gyroBias.x, raw.y - gyroBias.y, raw.z - gyroBias.z};
  Gyro lowPassFilteredGyro = lowPassFilter(unbiasedGyro);
  // Gyro deadBandFilteredGyro = deadBandFilter(lowPassFilteredGyro);
  
  return lowPassFilteredGyro;
}

void printGyro(bool printG, Gyro filteredGyro = {0,0,0})
{
    if(printG){
        Serial.print(filteredGyro.x);
        Serial.print(",");
        Serial.print(filteredGyro.y);
        Serial.print(",");
        Serial.println(filteredGyro.z);
    }
}