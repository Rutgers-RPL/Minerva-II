#include <pyro.h>

Pyro::Pyro(uint8_t fire_pin, uint8_t sense_pin) {
    this->fire_pin = fire_pin;
    this->sense_pin = sense_pin;

    pinMode(this->fire_pin, OUTPUT);
    pinMode(this->sense_pin, INPUT);
    digitalWrite(this->fire_pin, LOW);
}

void Pyro::fire(elapsedMillis millis, unsigned long ms) {
    start_time = millis + ms;
    
    digitalWrite(this->fire_pin, HIGH);
}

uint8_t Pyro::sense() {
    return digitalRead(this->sense_pin);
}

void Pyro::update(elapsedMillis millis) {
    if (millis > start_time)
        digitalWrite(this->fire_pin, LOW);
}