struct RCChannels {
  float throttle;
  float roll;
  float pitch;
  float yaw;

  int armSwitch;
  int switch2;
  int switch3;
  int switch4;
  int switch5;
  int switch6;

  float pot1;
  float pot2;

  float ch13;
  float ch14;
  float ch15;
  float ch16;

  bool frameLost;
  bool failsafe;
};

RCChannels rx;

uint8_t frame[25];


void setupRx()
{
  Serial1.begin(100000, SERIAL_8E2_RXINV);
}

bool readSBUS()
{
  static uint8_t idx = 0;

  while(Serial1.available())
  {
    uint8_t b = Serial1.read();

    if(idx == 0 && b != 0x0F)
      continue;

    frame[idx++] = b;

    if(idx == 25)
    {
      idx = 0;

      if(frame[24] == 0x00)
        return true;

      continue;
    }
  }

  return false;
}

void normalizeChannels()
{
  rx.throttle   =       rx.throttle / 2000.0;
  rx.roll       =   (rx.roll - 992) / 1000.0;
  rx.pitch      =  (rx.pitch - 992) / 1000.0;
  rx.yaw        =    (rx.yaw - 992) / 1000.0;

  rx.armSwitch  = (rx.armSwitch > 1000) ? 1 : 0;
  rx.switch2    = (rx.switch2 > 1500) ? 1 : ((rx.switch2 < 500) ? -1 : 0);
  rx.switch3    = (rx.switch3 > 1500) ? 1 : ((rx.switch3 < 500) ? -1 : 0);
  rx.switch4    = (rx.switch4 > 1500) ? 1 : ((rx.switch4 < 500) ? -1 : 0);
  rx.switch5    = (rx.switch5 > 1500) ? 1 : ((rx.switch5 < 500) ? -1 : 0);
  rx.switch6    = (rx.switch6 > 1500) ? 1 : ((rx.switch6 < 500) ? -1 : 0);
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

  normalizeChannels();
}

void printRx(bool doIt)
{
  if(doIt)
  {
    Serial.print(rx.throttle);
    Serial.print('\t');
    Serial.print(rx.roll);
    Serial.print('\t');
    Serial.print(rx.pitch);
    Serial.print('\t');
    Serial.print(rx.yaw);
    Serial.print('\t');

    Serial.print(rx.armSwitch);
    Serial.print('\t');
    Serial.print(rx.switch3);
    Serial.print('\t');
    Serial.print(rx.switch4);
    Serial.print('\t');
    Serial.print(rx.switch5);
    Serial.print('\t');
    Serial.print(rx.switch6);
    Serial.print('\t');

    Serial.print(rx.pot1);
    Serial.print('\t');
    Serial.print(rx.pot2);
    Serial.print('\t');

    Serial.print(rx.frameLost);
    Serial.print('\t');
    Serial.print(rx.failsafe);
    Serial.println('\t');
  }
}