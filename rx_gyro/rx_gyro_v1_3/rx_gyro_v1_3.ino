#include "gyro.h"
#include "rx.h"
#include "sd_card.h"

constexpr uint32_t LOOP_PERIOD_US = 1000; // 1 kHz

void stopProgram()
{
  if (Serial.available())
  {
    char c = Serial.read();

    if (c == 'q')
    {
      logFile.flush();
      logFile.close();

      while(1);
    }
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(CS_IMU, OUTPUT);
  digitalWrite(CS_IMU, HIGH);

  SPI.begin();
  delay(50);

  // Gyro setup
  setupGyro();

  // SD card setup
  setupSD();

  // Gyro setup
  setupRx();
}

void loop()
{
  // rx
  if(readSBUS())
  {
    decodeSBUS(frame);
    printRx(0);
  }

  // Gyro
  static uint32_t nextLoop = micros() + LOOP_PERIOD_US;
  if ((int32_t)(micros() - nextLoop) < 0)
    return;

  nextLoop += LOOP_PERIOD_US;

  Gyro raw = readGyro();
  Gyro filteredGyro = filterGyro(raw);

  loopSD(filteredGyro);

  printGyro(0, filteredGyro);

  static uint32_t t = micros();
  uint32_t dt = micros() - t;
  t = micros();
  if(dt > 1006)
    Serial.println(dt);

  stopProgram();
}