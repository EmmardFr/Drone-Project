#include <SD.h>

File logFile;

String createLogFileName()
{
    char filename[20];

    for (int i = 0; i < 10000; i++)
    {
        sprintf(filename, "logs%04d.csv", i);

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
        while(1);
    }

    String filename = createLogFileName();
    logFile = SD.open(filename.c_str(), FILE_WRITE);

    if (!logFile) {
        Serial.println("Failed to open file");
        while(1);
    }

    // logFile.println("time_us,gyroX,gyroY,gyroZ,rxThrottle,rxRoll,rxPitch,rxYaw,armSwitch,frameLost,failsafe,pidRoll,pidPitch,pidYaw,motor1,motor2,motor3,motor4");
    logFile.println("time_us,gyroRoll,gyroPitch,gyroYaw,rxThrottle,desiredRollRate,desiredPitchRate,desiredYawRate,armSwitch,frameLost,failsafe,pRoll,iRoll,dRoll,pPitch,iPitch,dPitch,pYaw,iYaw,dYaw,roll_coeff_p,roll_coeff_i,roll_coeff_d,pitch_coeff_p,pitch_coeff_i,pitch_coeff_d,yaw_coeff_p,yaw_coeff_i,yaw_coeff_d");
    logFile.flush();
}

void loopSD(const Gyro &gyro, const MOTOR_POWER &pwr)
{
    static uint32_t t0_us = micros();
    uint32_t t = micros() - t0_us;

    logFile.print(t);
    logFile.print(",");
    logFile.print(gyro.y);
    logFile.print(",");
    logFile.print(gyro.x);
    logFile.print(",");
    logFile.print(gyro.z);
    logFile.print(",");

    logFile.print(rx.throttle);
    logFile.print(",");
    logFile.print(rx.roll);
    logFile.print(",");
    logFile.print(-rx.pitch);
    logFile.print(",");
    logFile.print(-rx.yaw);
    logFile.print(",");
    logFile.print(rx.armSwitch);
    logFile.print(",");
    logFile.print(rx.frameLost);
    logFile.print(",");
    logFile.print(rx.failsafe);
    logFile.print(",");

    logFile.print(roll_pid.p);
    logFile.print(",");
    logFile.print(roll_pid.i);
    logFile.print(",");
    logFile.print(roll_pid.d);
    logFile.print(",");

    logFile.print(pitch_pid.p);
    logFile.print(",");
    logFile.print(pitch_pid.i);
    logFile.print(",");
    logFile.print(pitch_pid.d);
    logFile.print(",");

    logFile.print(yaw_pid.p);
    logFile.print(",");
    logFile.print(yaw_pid.i);
    logFile.print(",");
    logFile.print(yaw_pid.d);
    logFile.print(",");

    logFile.print(roll_coeff.p);
    logFile.print(",");
    logFile.print(roll_coeff.i);
    logFile.print(",");
    logFile.print(roll_coeff.d);
    logFile.print(",");

    logFile.print(pitch_coeff.p);
    logFile.print(",");
    logFile.print(pitch_coeff.i);
    logFile.print(",");
    logFile.print(pitch_coeff.d);
    logFile.print(",");

    logFile.print(yaw_coeff.p);
    logFile.print(",");
    logFile.print(yaw_coeff.i);
    logFile.print(",");
    logFile.println(yaw_coeff.d);

    // logFile.print(pwr.motor1);
    // logFile.print(",");
    // logFile.print(pwr.motor2);
    // logFile.print(",");
    // logFile.print(pwr.motor3);
    // logFile.print(",");
    // logFile.println(pwr.motor4);
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

            if (filename.endsWith(".csv"))
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