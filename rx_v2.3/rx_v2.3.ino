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

void setup()
{
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E2_RXINV);
}

void loop()
{
  if(readSBUS())
  {
    for(uint8_t i = 0; i < 25; ++i)
    {
      Serial.print(frame[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
}