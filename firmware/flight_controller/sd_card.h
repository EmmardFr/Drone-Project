#include <SD.h>

File logFile;

struct LogRecord
{
    uint32_t time_us;

    float gyroRoll;
    float gyroPitch;
    float gyroYaw;
    
    float rxThrottle;
    float desiredRollRate;
    float desiredPitchRate;
    float desiredYawRate;
    
    uint8_t armSwitch;
    uint8_t frameLost;
    uint8_t failsafe;
    
    float pRoll;
    float iRoll;
    float dRoll;
    
    float pPitch;
    float iPitch;
    float dPitch;

    float pYaw;
    float iYaw;
    float dYaw;
};

String createLogFileName()
{
    char filename[20];

    for (int i = 0; i < 10000; i++)
    {
        // sprintf(filename, "flight_logs%04d.csv", i);
        sprintf(filename, "logs%04d.bin", i);

        if (!SD.exists(filename))
        {
            return String(filename);
        }
    }

    return "";
}


void setupSD()
{
    // Serial.println(sizeof(LogRecord));

    if (!SD.begin(BUILTIN_SDCARD))
    {
        Serial.println("ERROR SD_INIT");
        while(1);
    }

    String filename = createLogFileName();
    logFile = SD.open(filename.c_str(), FILE_WRITE);

    if (!logFile)
    {
        Serial.println("Failed to open file");
        while(1);
    }

    logFile.println("time_us,gyroRoll,gyroPitch,gyroYaw,rxThrottle,desiredRollRate,desiredPitchRate,desiredYawRate,armSwitch,frameLost,failsafe,pRoll,iRoll,dRoll,pPitch,iPitch,dPitch,pYaw,iYaw,dYaw");

    logFile.write((const uint8_t*)&roll_coeff.p, sizeof(float));
    logFile.write((const uint8_t*)&roll_coeff.i, sizeof(float));
    logFile.write((const uint8_t*)&roll_coeff.d, sizeof(float));
    
    logFile.write((const uint8_t*)&pitch_coeff.p, sizeof(float));
    logFile.write((const uint8_t*)&pitch_coeff.i, sizeof(float));
    logFile.write((const uint8_t*)&pitch_coeff.d, sizeof(float));
    
    logFile.write((const uint8_t*)&yaw_coeff.p, sizeof(float));
    logFile.write((const uint8_t*)&yaw_coeff.i, sizeof(float));
    logFile.write((const uint8_t*)&yaw_coeff.d, sizeof(float));

    logFile.flush();
}

void loopSD(const Gyro &gyro)
{
    static uint32_t t0_us = micros();

    LogRecord record;

    record.time_us = micros() - t0_us;

    record.gyroRoll  = gyro.y;
    record.gyroPitch = gyro.x;
    record.gyroYaw   = gyro.z;

    record.rxThrottle      = rx.throttle;
    record.desiredRollRate = rx.roll;
    record.desiredPitchRate = -rx.pitch;
    record.desiredYawRate   = -rx.yaw;

    record.armSwitch = rx.armSwitch;
    record.frameLost = rx.frameLost;
    record.failsafe  = rx.failsafe;

    record.pRoll = roll_pid.p;
    record.iRoll = roll_pid.i;
    record.dRoll = roll_pid.d;

    record.pPitch = pitch_pid.p;
    record.iPitch = pitch_pid.i;
    record.dPitch = pitch_pid.d;

    record.pYaw = yaw_pid.p;
    record.iYaw = yaw_pid.i;
    record.dYaw = yaw_pid.d;

    // Write the complete structure directly as binary.
    logFile.write((const uint8_t*)&record, sizeof(LogRecord));
}

void downloadAllFiles()
{
    // Make sure the current log is completely written.
    if (logFile)
    {
        logFile.flush();
        logFile.close();
    }

    File root = SD.open("/");

    if (!root)
    {
        Serial.println("ERROR SD_OPEN_ROOT");
        return;
    }

    while (true)
    {
        File entry = root.openNextFile();

        if (!entry)
            break;

        if (!entry.isDirectory())
        {
            String filename = entry.name();

            if (filename.endsWith(".bin"))
            {
                uint32_t fileSize = entry.size();

                // Header:
                // FILE <filename> <size>\n
                Serial.print("FILE ");
                Serial.print(filename);
                Serial.print(" ");
                Serial.println(fileSize);

                // Wait until the PC tells us what to do.
                while (!Serial.available())
                {
                    // Do nothing
                }

                char response = Serial.read();

                if (response == 'S')
                {
                    // PC already has this file.
                    // Skip sending it.
                    entry.close();
                    continue;
                }

                if (response != 'R')
                {
                    entry.close();
                    root.close();

                    Serial.println("ERROR");

                    while (1);
                }

                // Send exactly fileSize bytes.
                uint8_t buffer[512];

                while (entry.available())
                {
                    int bytesRead = entry.read(buffer, sizeof(buffer));

                    if (bytesRead > 0)
                    {
                        Serial.write(buffer, bytesRead);
                    }
                }

                entry.close();

                // Make sure everything has physically gone through USB.
                Serial.flush();

                // Tell PC the file is finished.
                Serial.print("END\n");
            }
        }

        entry.close();
    }

    root.close();

    Serial.println("DONE");

    // Stop here. The drone must be restarted to return to flight mode.
    while (1)
    {
    }
}