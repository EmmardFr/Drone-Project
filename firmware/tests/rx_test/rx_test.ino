struct RCChannels {
    uint16_t throttle;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;

    uint16_t armSwitch;
    uint16_t switch2;
    uint16_t switch3;
    uint16_t switch4;
    uint16_t switch5;
    uint16_t switch6;

    uint16_t pot1;
    uint16_t pot2;

    uint16_t ch13;
    uint16_t ch14;
    uint16_t ch15;
    uint16_t ch16;

    bool frameLost;
    bool failsafe;
};

RCChannels rx;

uint8_t frame[25];

bool readSBUS()
{
  while (Serial1.available())
  {
    uint8_t b = Serial1.read();

    if (b != 0x0F)
      continue;

    frame[0] = b;

    if (Serial1.readBytes(&frame[1], 24) != 24)
      return false;
  
    if (frame[24] != 0x00)
      continue;

    return true;
  }
  return false;
}

void decodeSBUS(uint8_t frame[25])
{
  rx.throttle   = ((frame[1]       | frame[2] << 8)                     & 0x07FF);
  rx.roll       = ((frame[2] >> 3  | frame[3] << 5)                     & 0x07FF);
  rx.pitch      = ((frame[3] >> 6  | frame[4] << 2 | frame[5] << 10)    & 0x07FF);
  rx.yaw        = ((frame[5] >> 1  | frame[6] << 7)                     & 0x07FF);
  
  rx.armSwitch  = ((frame[6] >> 4  | frame[7] << 4)                     & 0x07FF);
  rx.switch2    = ((frame[7] >> 7  | frame[8] << 1 | frame[9] << 9)     & 0x07FF);
  rx.switch3    = ((frame[9] >> 2  | frame[10] << 6)                    & 0x07FF);
  rx.switch4    = ((frame[10] >> 5 | frame[11] << 3)                    & 0x07FF);
  rx.switch5    = ((frame[12]      | frame[13] << 8)                    & 0x07FF);
  rx.switch6    = ((frame[13] >> 3 | frame[14] << 5)                    & 0x07FF);

  rx.pot1       = ((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10)  & 0x07FF);
  rx.pot2       = ((frame[16] >> 1 | frame[17] << 7)                    & 0x07FF);

  rx.ch13       = ((frame[17] >> 4 | frame[18] << 4)                    & 0x07FF);
  rx.ch14       = ((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9)   & 0x07FF);
  rx.ch15       = ((frame[20] >> 2 | frame[21] << 6)                    & 0x07FF);
  rx.ch16       = ((frame[21] >> 5 | frame[22] << 3)                    & 0x07FF);

  rx.frameLost  =   frame[23] & (1 << 2);
  rx.failsafe   =   frame[23] & (1 << 3);
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E2_RXINV);
}

void loop()
{
  uint16_t t = micros();
  if(readSBUS())
  {
    uint16_t dt1 = micros() - t;
    Serial.print("dt1 ");
    Serial.println(dt1);
    
    decodeSBUS(frame);

    uint16_t dt2 = micros() - t;
    Serial.print("dt2 ");
    Serial.println(dt2);

    // Serial.print(rx.throttle);
    // Serial.print('\t');
    // Serial.print(rx.roll);
    // Serial.print('\t');
    // Serial.print(rx.pitch);
    // Serial.print('\t');
    // Serial.print(rx.yaw);
    // Serial.print('\t');

    // Serial.print(rx.armSwitch);
    // Serial.print('\t');
    // Serial.print(rx.switch3);
    // Serial.print('\t');
    // Serial.print(rx.switch4);
    // Serial.print('\t');
    // Serial.print(rx.switch5);
    // Serial.print('\t');
    // Serial.print(rx.switch6);
    // Serial.println('\t');

    // for(uint8_t i = 0; i < 25; ++i)
    // {
    //   Serial.print(frame[i], HEX);
    //   Serial.print(' ');
    // }
    uint16_t dt = micros() - t;
    Serial.print("dt ");
    Serial.println(dt);
    // Serial.println();
  }
  // uint16_t dt = t - micros();
  // Serial.println(dt);
}