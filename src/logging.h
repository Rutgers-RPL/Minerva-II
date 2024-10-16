/**
 * @file logging.h
 * @author Shivam Patel (shivam.patel94@rutgers.edu)
 * @brief Outlines the functions and variables necessary to manage data logging
 * @version 1.0
 * @date 2024-04-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef Logging_H
#define Logging_H

#include "structs.h"
#include <SdFat.h>
#include <Arduino.h>
#include <strings.h>
#include "state.h"

#define DATA_FILE_BASE_NAME String("FlightLog_")
#define DATA_FILE_BASE_TYPE String(".bin")
#define LOGGING_BUFFER_SIZE 50 // leq 255

class Logging{
    public:
        SdFs sd;
        FsFile dataFile;
        
        bool sdExists;

        String dataFileName;

        Logging(minerva_II_packet packet);

        template <typename T> void logBinaryPacket(const T* packet)
        {
            bufferCount++;
            this->dataFile.write((const uint8_t *)packet, sizeof(T));
            if (bufferCount >= LOGGING_BUFFER_SIZE)
            {
                this->dataFile.sync();   
                bufferCount = 0;
            }
        };

        void logM2Packet(const minerva_II_packet packet);
        void printM2Packet(const minerva_II_packet packet);

        static minerva_II_packet empty_M2_packet();

    private:
        u_int8_t bufferCount;
};


#endif 