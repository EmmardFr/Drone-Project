#include "gyro.h"
#include "sd_card.h"

constexpr uint32_t LOOP_PERIOD_US = 1000; // 1 kHz

void printGyro(bool printG, Gyro filteredGyro = {0,0,0})
{
    if(printG){
        Serial.print(filteredGyro.x);
        Serial.print(",");
        Serial.print(filteredGyro.y);
        Serial.print(",");
        Serial.println(filteredGyro.z);
    }
}

void stopProgram()
{
    if (Serial.available())
    {
        char c = Serial.read();

        if (c == 'q')
        {
            logFile.flush();
            logFile.close();

            while(1);
        }
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(CS_IMU, OUTPUT);
    digitalWrite(CS_IMU, HIGH);

    SPI.begin();
    delay(50);

    setupGyro();

    // SD card setup
    setupSD();

    delay(500);
}


void loop()
{
    static uint32_t nextLoop = micros() + LOOP_PERIOD_US;
    if ((int32_t)(micros() - nextLoop) < 0)
        return;

    nextLoop += LOOP_PERIOD_US;

    Gyro raw = readGyro();
    Gyro filteredGyro = filterGyro(raw);

    loopSD(filteredGyro);

    printGyro(0, filteredGyro);

    // type q in the serial monitor to stop the program
    // and flush the data in the sd card
    stopProgram();
}