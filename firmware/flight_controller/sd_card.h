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

// ---------------------------------------------------------------------
// Ring buffer RAM : découple l'échantillonnage (rapide, jamais bloquant)
// de l'écriture SD (potentiellement lente, parfois bloquante).
//
// Placé en DMAMEM (RAM2) pour ne pas grignoter la RAM1 (tightly-coupled,
// utilisée pour la pile/le tas critique). Le Teensy 4.1 a 512 Ko de RAM2 ;
// ~432 Ko laissent de la marge. Si le linker se plaint (RAM2 overflow),
// réduis LOG_BUFFER_RECORDS.
// ---------------------------------------------------------------------
static constexpr size_t LOG_BUFFER_RECORDS = 6000; // ~6s de log @ 1kHz

DMAMEM static LogRecord logBuffer[LOG_BUFFER_RECORDS];

static volatile size_t bufHead  = 0; // prochain index d'écriture (producteur : loopSD)
static volatile size_t bufTail  = 0; // prochain index de lecture (consommateur : SD writer)
static volatile size_t bufCount = 0;

static uint32_t droppedSamples = 0;
static uint32_t lastDroppedReportedAt = 0;

// Ne tente une écriture SD que s'il reste au moins cette marge (µs)
// avant le prochain tick de contrôle. Choisi confortablement au-dessus
// du temps de transfert nominal d'un petit bloc (qui est de l'ordre de
// quelques dizaines de µs en SDIO natif), pour laisser de la place au
// reste du travail du cycle (lecture RX, PID, etc.).
static constexpr uint32_t SD_WRITE_TIME_MARGIN_US = 400;

// Taille d'un "chunk" d'écriture opportuniste, en nombre de records.
// Choisi proche d'un multiple de 512 octets (taille de bloc SD typique)
// pour rester efficace sans monopoliser trop longtemps un seul write().
static constexpr size_t SD_WRITE_CHUNK_RECORDS = 7;

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
        while (1);
    }

    String filename = createLogFileName();
    logFile = SD.open(filename.c_str(), FILE_WRITE);

    if (!logFile)
    {
        Serial.println("Failed to open file");
        while (1);
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

    logFile.flush(); // OK ici : setup(), pas dans le chemin temps-réel

    bufHead = bufTail = bufCount = 0;
    droppedSamples = 0;
}

// Appelée depuis le control loop (1kHz). Ne fait jamais d'I/O SD :
// juste un memcpy dans le ring buffer. Coût quasi nul et constant.
void loopSD(const Gyro &gyro)
{
    static uint32_t t0_us = micros();

    if (bufCount >= LOG_BUFFER_RECORDS)
    {
        // Le buffer n'a pas eu le temps de se vider : on préfère perdre
        // cet échantillon plutôt que de bloquer le control loop.
        droppedSamples++;
        return;
    }

    LogRecord &record = logBuffer[bufHead];

    record.time_us = micros() - t0_us;

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

    bufHead = (bufHead + 1) % LOG_BUFFER_RECORDS;
    bufCount++;

    // Rapport occasionnel (throttlé) des échantillons perdus, utile en debug.
    // Ne s'exécute que juste après un drop donc coût négligeable en moyenne.
    if (droppedSamples > 0 && (millis() - lastDroppedReportedAt) > 2000)
    {
        Serial.print("SD_WARN dropped=");
        Serial.println(droppedSamples);
        lastDroppedReportedAt = millis();
    }
}

// Écriture SD "opportuniste" et bornée en temps.
// À appeler à CHAQUE passage dans loop() (y compris pendant le busy-wait
// avant le prochain tick de contrôle) — c'est ce qui permet d'utiliser le
// temps CPU autrement perdu à "spinner" pour vider le buffer.
//
// nextLoopDeadline : la valeur de la variable `nextLoop` du control loop
// (deadline absolue en µs, comparable à micros()).
//
// Best-effort : borne la FRÉQUENCE et la TAILLE des écritures pour
// réduire drastiquement la probabilité qu'un stall carte tombe pendant
// un cycle de contrôle. Ne peut pas garantir qu'un write() individuel ne
// stallera jamais (ça dépend du firmware interne de la carte SD) — c'est
// la limite intrinsèque de l'API SdFat synchrone. En pratique, avec ce
// pattern, les stalls occasionnels ne coïncident (quasi) plus avec les
// moments où readSBUS()/le control loop en ont besoin.
void serviceSDWriter(uint32_t nextLoopDeadline)
{
    if (bufCount == 0)
        return;

    int32_t slack = (int32_t)(nextLoopDeadline - micros());

    if (slack < (int32_t)SD_WRITE_TIME_MARGIN_US)
        return; // pas assez de marge, on retentera au prochain passage

    size_t chunk = (bufCount < SD_WRITE_CHUNK_RECORDS) ? bufCount : SD_WRITE_CHUNK_RECORDS;
    size_t tailIdx = bufTail;

    // Gère le wrap-around du buffer circulaire en au plus 2 écritures.
    size_t firstPart = LOG_BUFFER_RECORDS - tailIdx;
    if (firstPart > chunk) firstPart = chunk;

    logFile.write((const uint8_t*)&logBuffer[tailIdx], firstPart * sizeof(LogRecord));

    if (chunk > firstPart)
    {
        logFile.write((const uint8_t*)&logBuffer[0], (chunk - firstPart) * sizeof(LogRecord));
    }

    bufTail = (bufTail + chunk) % LOG_BUFFER_RECORDS;
    bufCount -= chunk;
}

// Vidange complète et BLOQUANTE du buffer restant. À n'appeler que quand
// on est déjà en train de s'arrêter volontairement (désarmé, 'q', etc.) :
// bloquer est acceptable dans ce contexte, contrairement au control loop.
void flushAllPendingSD()
{
    while (bufCount > 0)
    {
        size_t chunk = (bufCount < SD_WRITE_CHUNK_RECORDS) ? bufCount : SD_WRITE_CHUNK_RECORDS;
        size_t tailIdx = bufTail;

        size_t firstPart = LOG_BUFFER_RECORDS - tailIdx;
        if (firstPart > chunk) firstPart = chunk;

        logFile.write((const uint8_t*)&logBuffer[tailIdx], firstPart * sizeof(LogRecord));

        if (chunk > firstPart)
        {
            logFile.write((const uint8_t*)&logBuffer[0], (chunk - firstPart) * sizeof(LogRecord));
        }

        bufTail = (bufTail + chunk) % LOG_BUFFER_RECORDS;
        bufCount -= chunk;
    }

    if (droppedSamples > 0)
    {
        Serial.print("SD_INFO total_dropped=");
        Serial.println(droppedSamples);
    }
}

void downloadAllFiles()
{
    // Assure que le log courant est complètement écrit.
    if (logFile)
    {
        flushAllPendingSD();
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
                    // Do nothing
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
                    int bytesRead = entry.read(buffer, sizeof(buffer));

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

    flushAllPendingSD();

    while (1)
    {
    }
}