void setup()
{
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E2_RXINV);
}

void loop()
{
  static int count = 0;

  while (Serial1.available())
  {
    uint8_t b = Serial1.read(); 

    if (b < 16) Serial.print('0');
    Serial.print(b, HEX);
    Serial.print(' ');

    count++;

    if (count == 25)
    {
      Serial.println();
      count = 0;
    }
  }
}