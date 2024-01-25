#include "logging.h"


Logging::Logging(minerva_II_packet packet)
{
    delay(1000);
    byte attempts = 1;
    this->sdExists = false;
    while (attempts <= 10 && !this->sdExists) {
        if (!this->sd.begin(SdioConfig(FIFO_SDIO))) {
            packet.status |= 1<<7;
            Serial.print("SD Begin Failed, Attempting "); Serial.print(10 - attempts++); Serial.println(" more tries ...");
            delay(1000);
        } else {
            packet.status &= ~(1<<7);
            Serial.println("\nFIFO SDIO mode.");

            // INIT DATA FILE

            uint16_t n_file = 0;
            this->dataFileName = DATA_FILE_BASE_NAME + "0000" + DATA_FILE_BASE_TYPE;
            while(sd.exists(this->dataFileName))
            {
                n_file++;
                char fIdx[10];
                sprintf(fIdx, "%04d", n_file);
                this->dataFileName = DATA_FILE_BASE_NAME + fIdx + DATA_FILE_BASE_TYPE;
            }

            this->dataFile = sd.open(this->dataFileName, FILE_WRITE);
            Serial.print("Writing data to: ");
            Serial.println(this->dataFileName);
            if (!this->dataFile) {
                packet.status |= 1<<7;
                Serial.println("Failed opening data file.");
                break;
            }

            // SUMMARY FILE

            n_file = 0;
            this->summaryFileName = SUMMARY_FILE_BASE_NAME + "0000" + SUMMARY_FILE_BASE_TYPE;
            while(sd.exists(this->summaryFileName))
            {
                n_file++;
                char fIdx[32];
                sprintf(fIdx, "%04d", n_file);
                this->summaryFileName = DATA_FILE_BASE_NAME + fIdx + DATA_FILE_BASE_TYPE;
            }

            this->summaryFile = sd.open(this->summaryFileName, FILE_WRITE);
            Serial.print("Writing summary to: ");
            Serial.println(this->summaryFileName);
            if (!this->dataFile) {
                packet.status |= 1<<7;
                Serial.println("Failed opening summary file.");
                break;
            }

            this->sdExists = true;
        }
    }

    this->max_acc = 0;
    this->max_vel = 0;
    this->max_alt = 0;

    this->bufferCount = 0;
};

template <typename T> void Logging::logBinaryPacket(const T* packet)
{
    bufferCount++;
    this->dataFile.write((const uint8_t *)packet, sizeof(T));
    if (bufferCount >= LOGGING_BUFFER_SIZE)
    {
        this->dataFile.sync();   
        bufferCount = 0;
    }
};

void Logging::updateSummary(const minerva_II_packet packet1, const state_packet packet2)
{
    if(packet1.kf_acceleration_mss > max_acc)
    {
        max_acc = packet1.kf_acceleration_mss;
    }
    if(packet1.kf_velocity_ms > max_vel)
    {
        max_vel = packet1.kf_velocity_ms;
    }
    if(packet1.kf_position_m > max_alt)
    {
        max_alt = packet1.kf_position_m;
    }

    sprintf(maxSummary, "Maximum Altitude: %f \nMaximum Velocity: %f \nMaximum Acceleration: %f \n", max_alt, max_vel, max_acc);
    sprintf(eventsSummary, "Armed: %hhd \nFired Drogue: %hhd \nFired Main: %hhd \n", packet2.state_flags & ARMED, packet2.state_flags & FIRED_DROGUE, packet2.state_flags & FIRED_MAIN);

    String full = String(maxSummary) + String(eventsSummary);

    updateCount++;
    
    if(updateCount >= 250)
    {
        updateCount = 0;
        this->summaryFile.print(full);
        this->summaryFile.seekSet(0);
    }

};