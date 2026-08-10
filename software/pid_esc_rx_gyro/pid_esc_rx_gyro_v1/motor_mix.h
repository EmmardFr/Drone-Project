constexpr uint16_t MOTOR_STOP     = 0;
constexpr uint16_t MOTOR_IDLE     = 130;
constexpr uint16_t MOTOR_MAX      = 2047;

struct MOTOR_POWER{
  uint32_t motor1;
  uint32_t motor2;
  uint32_t motor3;
  uint32_t motor4;
};

MOTOR_POWER computePower(float throttle, float roll, float pitch, float yaw)
{
  roll  = constrain(roll,  -1000, 1000);
  pitch = constrain(pitch, -1000, 1000);
  yaw   = constrain(yaw,   -2000, 2000);

  MOTOR_POWER pwr = {0, 0, 0, 0};
  
  float throttleCmd = MOTOR_IDLE + throttle * (MOTOR_MAX - MOTOR_IDLE);

  pwr.motor1 = constrain(throttleCmd - roll - pitch - yaw, MOTOR_IDLE, MOTOR_MAX);
  pwr.motor2 = constrain(throttleCmd - roll + pitch + yaw, MOTOR_IDLE, MOTOR_MAX);
  pwr.motor3 = constrain(throttleCmd + roll - pitch + yaw, MOTOR_IDLE, MOTOR_MAX);
  pwr.motor4 = constrain(throttleCmd + roll + pitch - yaw, MOTOR_IDLE, MOTOR_MAX);

  return pwr;
}

void printMPower(MOTOR_POWER pwr, bool doIt = true)
{
  if(doIt)
  {
    Serial.print(pwr.motor1);
    Serial.print('\t');
    Serial.print(pwr.motor2);
    Serial.print('\t');
    Serial.print(pwr.motor3);
    Serial.print('\t');
    Serial.println(pwr.motor4);
  }
}