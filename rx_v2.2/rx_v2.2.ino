uint8_t frame[25];

bool readSBUS()
{
  while(Serial1.available() >= 25)
  {
    Serial1.readBytes(&frame[0], 25);

    if(frame[0] != 0x0F || frame[24] != 0x00)
    {
      Serial.println("No good bytes");
      continue;
    }

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