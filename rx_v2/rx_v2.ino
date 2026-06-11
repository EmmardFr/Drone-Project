uint8_t frame[25];

bool readRxByte() {
  while(Serial1.available())
  {
    uint8_t b = Serial1.read();

    if(b != 0x0F)
      continue;

    if(Serial1.available() < 24)
    {
      // Serial.println("Not available");
      return false;
    }

    frame[0] = b;
    size_t n = Serial1.readBytes(&frame[1], 24);

    if (n != 24) {
      Serial.println("Bad Length");
      return false;
    }

    if (frame[0] != 0x0F || frame[24] != 0x00) {
      Serial.println("No good bytes");
      return false;
    }

    return true;
  }
  // Serial.println("No Serial1");
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E2_RXINV);

  Serial.println("Set up");
}

void loop() {
  if (readRxByte()) {
    for (uint8_t i = 0; i < 25; ++i) {
      Serial.print(frame[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}