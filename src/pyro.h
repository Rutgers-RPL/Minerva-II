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