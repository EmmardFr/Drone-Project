#include "gyro.h"
#include "rx.h"
#include "esc.h"
#include "pid.h"
#include "motor_mix.h"
#include "sd_card.h"

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

    if (c == 'd')
    {
      // Never allow SD download while armed.
      if (rx.armSwitch == 0)
      {
        downloadAllFiles();
      }
      else
      {
        Serial.println("ERROR ARMED");
      }
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
  setupESC();

  ////////////////////////////////////////
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  ////////////////////////////////////////

  // delay(2000);
  
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

  // Control loop
  static uint32_t nextLoop = micros() + LOOP_PERIOD_US;
  if ((int32_t)(micros() - nextLoop) < 0)
    return;
  
  nextLoop += LOOP_PERIOD_US;
  // nextLoop = micros() + LOOP_PERIOD_US;

  uint32_t now = micros();
  static uint32_t prev_time = now - 1000;
  float dt = (now - prev_time) * 1e-6f;
  prev_time = now;

  // Gyro read and filter
  Gyro raw = readGyro();
  Gyro filteredGyro = filterGyro(raw);
  printGyro(0, filteredGyro);

  // Compute PID and send commands to motors
  MOTOR_POWER motor_power;

  if(rx.armSwitch == 0 && !rx.failsafe && !rx.frameLost)
  {
    sendThrottle(MOTOR_STOP, MOTOR_PIN1);
    sendThrottle(MOTOR_STOP, MOTOR_PIN2);
    sendThrottle(MOTOR_STOP, MOTOR_PIN3);
    sendThrottle(MOTOR_STOP, MOTOR_PIN4);

    motor_power = {0,0,0,0};
  }
  else if(failsafeConfirmed() || rxSignalLost() || frameLostConfirmed())
  {
    sendThrottle(MOTOR_STOP, MOTOR_PIN1);
    sendThrottle(MOTOR_STOP, MOTOR_PIN2);
    sendThrottle(MOTOR_STOP, MOTOR_PIN3);
    sendThrottle(MOTOR_STOP, MOTOR_PIN4);

    motor_power = {0, 0, 0, 0};

    rxFlushSD = true;
  }
  else if(rx.failsafe || rx.frameLost)
  {
    sendThrottle(prev_motor1, MOTOR_PIN1);
    sendThrottle(prev_motor2, MOTOR_PIN2);
    sendThrottle(prev_motor3, MOTOR_PIN3);
    sendThrottle(prev_motor4, MOTOR_PIN4);

    motor_power = {prev_motor1, prev_motor2, prev_motor3, prev_motor4};
  }
  else
  {
    if(firstArm && rx.throttle > 0.05)
    {
      sendThrottle(MOTOR_STOP, MOTOR_PIN1);
      sendThrottle(MOTOR_STOP, MOTOR_PIN2);
      sendThrottle(MOTOR_STOP, MOTOR_PIN3);
      sendThrottle(MOTOR_STOP, MOTOR_PIN4);

      motor_power = {0, 0, 0, 0};

      rxFlushSD = true;
    }
    else
    {
      computePid(rx.roll, -rx.pitch, -rx.yaw, filteredGyro.y, filteredGyro.x, filteredGyro.z, dt);
      printPID(0);

      motor_power = computePower(rx.throttle, pid.roll, pid.pitch, pid.yaw);
      printMPower(motor_power, 0);
      
      sendThrottle(motor_power.motor1, MOTOR_PIN1);
      sendThrottle(motor_power.motor2, MOTOR_PIN2);
      sendThrottle(motor_power.motor3, MOTOR_PIN3);
      sendThrottle(motor_power.motor4, MOTOR_PIN4);
    }
  }

  // log in sd card
  loopSD(filteredGyro, dt);

  // static uint32_t t = micros();
  // uint32_t dt = micros() - t;
  // t = micros();
  // if(dt > 1006)
  //   Serial.println(dt);

  stopProgram();
}