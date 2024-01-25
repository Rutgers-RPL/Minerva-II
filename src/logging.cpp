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
                char fIdx[5];
                snprintf(fIdx, 5, "%04d", n_file);
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
                char fIdx[5];
                snprintf(fIdx, 5, "%04d", n_file);
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
    if (bufferCount >= LOGGING_BUFFER_SIZE)
    {
        this->dataFile.write((const uint8_t *)packet, sizeof(T));
        bufferCount = 0;
    }
}