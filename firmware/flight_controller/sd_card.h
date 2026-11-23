// ============================================================
// SD CARD CODE
// ============================================================

#include <SD.h>

File logFile;


struct LogRecord
{
    uint32_t time_us;
    float dt;

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


    // ========================================================
    // Text column header
    // ========================================================

    logFile.println(
        "time_us,dt,"
        "gyroRoll,gyroPitch,gyroYaw,"
        "rxThrottle,"
        "desiredRollRate,desiredPitchRate,desiredYawRate,"
        "armSwitch,frameLost,failsafe,"
        "pRoll,iRoll,dRoll,"
        "pPitch,iPitch,dPitch,"
        "pYaw,iYaw,dYaw"
    );


    // ========================================================
    // PID gains
    // ========================================================

    logFile.write(
        (const uint8_t*)&roll_coeff.p,
        sizeof(float)
    );

    logFile.write(
        (const uint8_t*)&roll_coeff.i,
        sizeof(float)
    );

    logFile.write(
        (const uint8_t*)&roll_coeff.d,
        sizeof(float)
    );


    logFile.write(
        (const uint8_t*)&pitch_coeff.p,
        sizeof(float)
    );

    logFile.write(
        (const uint8_t*)&pitch_coeff.i,
        sizeof(float)
    );

    logFile.write(
        (const uint8_t*)&pitch_coeff.d,
        sizeof(float)
    );


    logFile.write(
        (const uint8_t*)&yaw_coeff.p,
        sizeof(float)
    );

    logFile.write(
        (const uint8_t*)&yaw_coeff.i,
        sizeof(float)
    );

    logFile.write(
        (const uint8_t*)&yaw_coeff.d,
        sizeof(float)
    );

    logFile.flush();
}


void loopSD(const Gyro &gyro, float dt)
{
    static uint32_t t0_us = micros();

    LogRecord record;

    record.time_us = micros() - t0_us;
    record.dt = dt;

    record.gyroRoll  = gyro.y;
    record.gyroPitch = gyro.x;
    record.gyroYaw   = gyro.z;

    record.rxThrottle       = rx.throttle;
    record.desiredRollRate  = rx.roll;
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

    // Write complete record as binary.
    logFile.write(
        (const uint8_t*)&record,
        sizeof(LogRecord)
    );
}


void downloadAllFiles()
{
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

                Serial.print("FILE ");
                Serial.print(filename);
                Serial.print(" ");
                Serial.println(fileSize);

                while (!Serial.available())
                {
                }

                char response = Serial.read();

                if (response == 'S')
                {
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

                uint8_t buffer[512];

                while (entry.available())
                {
                    int bytesRead =
                        entry.read(buffer, sizeof(buffer));

                    if (bytesRead > 0)
                    {
                        Serial.write(buffer, bytesRead);
                    }
                }

                entry.close();

                Serial.flush();

                Serial.print("END\n");
            }
        }

        entry.close();
    }

    root.close();

    Serial.println("DONE");

    while (1)
    {
    }
}