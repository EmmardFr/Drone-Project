#include <SD.h>

File logFile;

String createLogFileName()
{
    char filename[20];

    for (int i = 0; i < 10000; i++)
    {
        sprintf(filename, "gyro%04d.csv", i);

        if (!SD.exists(filename))
        {
            return String(filename);
        }
    }

    return "";
}


void setupSD()
{
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD init failed!");
        while (1);
    }

    String filename = createLogFileName();
    logFile = SD.open(filename.c_str(), FILE_WRITE);

    if (!logFile) {
        Serial.println("Failed to open file");
        while (1);
    }

    logFile.println("time_us,gyroX,gyroY,gyroZ");
    logFile.flush();
}

void loopSD(Gyro gyro)
{
    static uint32_t t0_us = micros();
    uint32_t t = micros() - t0_us;

    logFile.print(t);
    logFile.print(",");
    logFile.print(gyro.x);
    logFile.print(",");
    logFile.print(gyro.y);
    logFile.print(",");
    logFile.println(gyro.z);

    // // Reduce SD wear + ensure data is written
    // static int counter = 0;
    // if (++counter % 1000 == 0) {
    //     logFile.flush();
    // }
}