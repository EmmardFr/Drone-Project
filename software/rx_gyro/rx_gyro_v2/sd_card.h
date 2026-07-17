#include <SD.h>

File logFileIMU;
File logFileRX;
uint32_t logStartUs = 0;

bool createLogFilesNames(char filenameIMU[20], char filenameRX[20])
{
    for (int i = 0; i < 10000; i++)
    {
        sprintf(filenameIMU, "gyro%04d.csv", i);
        sprintf(filenameRX, "rx%04d.csv", i);

        if (!SD.exists(filenameIMU) && !SD.exists(filenameRX))
        {
            return 1;
        }
    }
    return 0;
}


void setupSD()
{
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD init failed!");
        while (1);
    }

    char filenameIMU[20];
    char filenameRX[20];
    if (!createLogFilesNames(filenameIMU, filenameRX))
    {
        Serial.println("No free filename found");
        while(1);
    }
    logFileIMU = SD.open(filenameIMU, FILE_WRITE);
    logFileRX = SD.open(filenameRX, FILE_WRITE);

    if (!logFileIMU || !logFileRX) {
        Serial.println("Failed to open file");
        while(1);
    }

    logFileIMU.println("time_us,gyroX,gyroY,gyroZ");
    logFileIMU.flush();

    // logFileRX.println("time_us,throttle,roll,pitch,yaw,armSwitch,switch2,switch3,switch4,switch5,switch6,pot1,pot2,ch13,ch14,ch15,ch16,frameLost,failsafe");
    logFileRX.println("time_us,throttle,roll,pitch,yaw");
    logFileRX.flush();

    logStartUs = micros();
}

void loopSD_IMU(Gyro gyro)
{
    uint32_t t = micros() - logStartUs;

    // logFileIMU.println(t+","+String(gyro.x)+","+String(gyro.y)+","+String(gyro.z));

    // logFileIMU.print(t);
    // logFileIMU.print(",");
    // logFileIMU.print(gyro.x);
    // logFileIMU.print(",");
    // logFileIMU.print(gyro.y);
    // logFileIMU.print(",");
    // logFileIMU.println(gyro.z);

    logFileIMU.print(t);
    logFileIMU.print(",");
    logFileIMU.print(gyro.x);
    logFileIMU.print(",");
    logFileIMU.print(gyro.y);
    logFileIMU.print(",");
    logFileIMU.print(gyro.z);

    logFileIMU.print(t);
    logFileIMU.print(",");
    logFileIMU.print(rx.throttle);
    logFileIMU.print(",");
    logFileIMU.print(rx.roll);
    logFileIMU.print(",");
    logFileIMU.print(rx.pitch);
    logFileIMU.print(",");
    logFileIMU.println(rx.yaw);
}

void loopSD_RX(RCChannels rx)
{
    uint32_t t = micros() - logStartUs;
    
    logFileRX.print(t);
    logFileRX.print(",");
    logFileRX.print(rx.throttle);
    logFileRX.print(",");
    logFileRX.print(rx.roll);
    logFileRX.print(",");
    logFileRX.print(rx.pitch);
    logFileRX.print(",");
    logFileRX.println(rx.yaw);

    // logFileRX.print(t);
    // logFileRX.print(",");
    // logFileRX.print(rx.throttle);
    // logFileRX.print(",");
    // logFileRX.print(rx.roll);
    // logFileRX.print(",");
    // logFileRX.print(rx.pitch);
    // logFileRX.print(",");
    // logFileRX.print(rx.yaw);

    // logFileRX.print(",");
    // logFileRX.print(rx.armSwitch);
    // logFileRX.print(",");
    // logFileRX.print(rx.switch2);
    // logFileRX.print(",");
    // logFileRX.print(rx.switch3);
    // logFileRX.print(",");
    // logFileRX.print(rx.switch4);
    // logFileRX.print(",");
    // logFileRX.print(rx.switch5);
    // logFileRX.print(",");
    // logFileRX.print(rx.switch6);

    // logFileRX.print(",");
    // logFileRX.print(rx.pot1);
    // logFileRX.print(",");
    // logFileRX.print(rx.pot2);

    // logFileRX.print(",");
    // logFileRX.print(rx.ch13);
    // logFileRX.print(",");
    // logFileRX.print(rx.ch14);
    // logFileRX.print(",");
    // logFileRX.print(rx.ch15);
    // logFileRX.print(",");
    // logFileRX.print(rx.ch16);

    // logFileRX.print(",");
    // logFileRX.print(rx.frameLost);
    // logFileRX.print(",");
    // logFileRX.println(rx.failsafe);
}