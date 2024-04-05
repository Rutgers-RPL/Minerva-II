/**
 * @file pyro.cpp
 * @author Carlton Wu (carlton.wu@rutgers.edu)
 * @brief This declares the necessary variables and functions to use the onboard pyro channels
 * @version 1.1
 * @date 2024-06-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef Pyro_H
#define Pyro_H

#include <Arduino.h>
#include <structs.h>

class Pyro {
    private:
        uint8_t fire_pin;
        uint8_t sense_pin;
        uint8_t fire_bit;
        uint8_t sense_bit;
        unsigned long start_time;
    public:
        Pyro(uint8_t fire_pin, uint8_t sense_pin, uint8_t fire_bit, uint8_t sense_bit);
        void fire(elapsedMillis millis, unsigned long ms);
        uint8_t sense();
        void update(minerva_II_packet *packet, elapsedMillis millis);
};

#endif