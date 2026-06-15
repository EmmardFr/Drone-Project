#define MOTOR_PIN1 2
#define MOTOR_PIN2 3
#define MOTOR_PIN3 4
#define MOTOR_PIN4 5

constexpr uint32_t TOTAL_DSHOT_CYCLES = 2000;
constexpr uint32_t ONE_HIGH_DSHOT_CYCLES = 1200;
constexpr uint32_t ZERO_HIGH_DSHOT_CYCLES = 600;

uint16_t makeDShotFrame(uint16_t throttle, bool telemetry = false)
{
  // throttle = constrain(throttle, 48, 2047);

  if(throttle > 2047)
  {
    throttle = 2047;
  }

  uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);

  uint16_t csum = 0;
  uint16_t csum_data = packet;

  for(int i = 0; i < 3; i++) {
    csum ^= csum_data;
    csum_data >>= 4;
  }
  csum &= 0x0F;

  return (packet << 4) | csum;
}

// static inline void waitCyclesOne()
// {
//   digitalWriteFast(MOTOR_PIN, HIGH);
//   while((ARM_DWT_CYCCNT - start) < ONE_HIGH_DSHOT_CYCLES);
//   digitalWriteFast(MOTOR_PIN, HIGH);
//   while((ARM_DWT_CYCCNT - start) < TOTAL_DSHOT_CYCLES);
// }

// static inline void waitCyclesZero()
// {
//   digitalWriteFast(MOTOR_PIN, HIGH);
//   while((ARM_DWT_CYCCNT - ZERO_HIGH_DSHOT_CYCLES) < );
//   digitalWriteFast(MOTOR_PIN, HIGH);
//   while((ARM_DWT_CYCCNT - start) < TOTAL_DSHOT_CYCLES);
// }

void sendDShotFrame(uint16_t packet, uint8_t motor_pin)
{
  bool bits[16];

  for(int i = 0; i < 16; i++)
  {
    bits[i] = packet & (1 << (15 - i));
  }

  noInterrupts();

  uint32_t start = ARM_DWT_CYCCNT;

  for(int i = 0; i < 16; i++)
  {
    if (bits[i])
    {
      digitalWriteFast(motor_pin, HIGH);
      while((ARM_DWT_CYCCNT - start) < ONE_HIGH_DSHOT_CYCLES);
      digitalWriteFast(motor_pin, LOW);
      while((ARM_DWT_CYCCNT - start) < TOTAL_DSHOT_CYCLES);
    }
    else
    {
      digitalWriteFast(motor_pin, HIGH);
      while((ARM_DWT_CYCCNT - start) < ZERO_HIGH_DSHOT_CYCLES);
      digitalWriteFast(motor_pin, LOW);
      while((ARM_DWT_CYCCNT - start) < TOTAL_DSHOT_CYCLES);
    }
    start += TOTAL_DSHOT_CYCLES;
  }
  digitalWriteFast(motor_pin, LOW);
  interrupts();
}

void sendThrottle(uint16_t throttle, uint8_t motor, bool telemetry = false)
{
  uint16_t dshotFrame = makeDShotFrame(throttle, telemetry);
  sendDShotFrame(dshotFrame, motor);
}

void sendCommand(uint16_t cmd, uint8_t motor)
{
  for(int i = 0; i < 10; i++)
  {
    sendThrottle(cmd, motor);
    // delayMicroseconds(500);
  }
  delay(50);
  for(int i = 0; i < 10; i++)
  {
    sendThrottle(12, motor);
    // delayMicroseconds(500);
  }
  delay(50);
}

void setupESC()
{
  pinMode(MOTOR_PIN1, OUTPUT);
  digitalWriteFast(MOTOR_PIN1, LOW);
  pinMode(MOTOR_PIN2, OUTPUT);
  digitalWriteFast(MOTOR_PIN2, LOW);
  pinMode(MOTOR_PIN3, OUTPUT);
  digitalWriteFast(MOTOR_PIN3, LOW);
  pinMode(MOTOR_PIN4, OUTPUT);
  digitalWriteFast(MOTOR_PIN4, LOW);
  delay(100);

  ARM_DEMCR |= ARM_DEMCR_TRCENA;   // Enable DWT
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  for(uint8_t i = 0; i < 100; i++)
  {
    sendThrottle(0, MOTOR_PIN1);
    delay(1);
    sendThrottle(0, MOTOR_PIN2);
    delay(1);
    sendThrottle(0, MOTOR_PIN3);
    delay(1);
    sendThrottle(0, MOTOR_PIN4);
    delay(1);
  }

  // sendCommand(27, MOTOR_PIN2);  // reverse direction
  // sendCommand(27, MOTOR_PIN3);  // reverse direction
}

// void encodeDShot(uint16_t packet, uint8_t *buffer)
// {
//   for(int i = 0; i < 16; i++)
//   {
//     bool bit = packet & (1 << (15 - i));

//     if (bit)
//     {
//       buffer[i * 2]     = DS1_HIGH;        // high duration
//       buffer[i * 2 + 1] = DS_TOTAL - DS1_HIGH;
//     }
//     else
//     {
//       buffer[i * 2]     = DS0_HIGH;
//       buffer[i * 2 + 1] = DS_TOTAL - DS0_HIGH;
//     }
//   }
// }

// void comDshot(uint8_t *buffer)
// {
//   disableInterrupts;

//   for(int i = 0; i < 16; i++)
//   {
//     setPin(HIGH);
//     waitCycles(buffer[i * 2]);

//     setPin(LOW);
//     waitCycles(buffer[i * 2 + 1]);
//   }

//   enableInterrupts();
// }