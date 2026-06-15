#include "gyro.h"
#include "rx.h"
#include "sd_card.h"
#include "esc.h"
#include "pid.h"
#include "motor_mix.h"

constexpr uint32_t LOOP_PERIOD_US = 1000; // 1 kHz

void stopProgram()
{
  if (Serial.available())
  {
    char c = Serial.read();

    if(c == 'q')
    {
      logFile.flush();
      logFile.close();
      rxFlushSD = false;

      while(1);
    }
  }
  if(rxFlushSD)
  {
    logFile.flush();
    logFile.close();
    rxFlushSD = false;

    while(1);
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

  setupESC();
}

void loop()
{
  // rx
  if(readSBUS())
  {
    decodeSBUS(frame);
    printRx(0);
  }

  // Control loop
  static uint32_t nextLoop = micros() + LOOP_PERIOD_US;
  if ((int32_t)(micros() - nextLoop) < 0)
    return;

  nextLoop += LOOP_PERIOD_US;

  Gyro raw = readGyro();
  Gyro filteredGyro = filterGyro(raw);

  loopSD(filteredGyro);

  printGyro(0, filteredGyro);

  if(rx.armSwitch == 0)
  {
    sendThrottle(MOTOR_STOP, MOTOR_PIN1);
    sendThrottle(MOTOR_STOP, MOTOR_PIN2);
    sendThrottle(MOTOR_STOP, MOTOR_PIN3);
    sendThrottle(MOTOR_STOP, MOTOR_PIN4);
  }
  else
  {
    computePid(rx.roll, -rx.pitch, -rx.yaw, filteredGyro.y, filteredGyro.x, filteredGyro.z);
    printPID(0);

    MOTOR_POWER motor_power = computePower(rx.throttle, pid.roll, pid.pitch, pid.yaw);
    printMPower(motor_power, 0);
    
    sendThrottle(motor_power.motor1, MOTOR_PIN1);
    sendThrottle(motor_power.motor2, MOTOR_PIN2);
    sendThrottle(motor_power.motor3, MOTOR_PIN3);
    sendThrottle(motor_power.motor4, MOTOR_PIN4);
  }

  // static uint32_t t = micros();
  // uint32_t dt = micros() - t;
  // t = micros();
  // if(dt > 1006)
  //   Serial.println(dt);

  stopProgram();
}