#ifndef Pyro_H
#define Pyro_H

#include <Arduino.h>

class Pyro {
    private:
        uint8_t fire_pin;
        uint8_t sense_pin;
        unsigned long start_time;

    Pyro(uint8_t fire_pin, uint8_t sense_pin);
    void fire(elapsedMillis millis, unsigned long ms);
    uint8_t sense();
    void update(elapsedMillis millis);
};

#endif