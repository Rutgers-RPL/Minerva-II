#include <pyro.h>
#include <structs.h>

// Function to set the kth bit of n
int setBit(int n, int k)
{
    return (n | (1 << (k - 1)));
}
  
// Function to clear the kth bit of n
int clearBit(int n, int k)
{
    return (n & (~(1 << (k - 1))));
}
  
// Function to toggle the kth bit of n
int toggleBit(int n, int k)
{
    return (n ^ (1 << (k - 1)));
}

Pyro::Pyro(uint8_t fire_pin, uint8_t sense_pin, uint8_t fire_bit, uint8_t sense_bit) {
    this->fire_pin = fire_pin;
    this->sense_pin = sense_pin;
    this->fire_bit = fire_bit;
    this->sense_bit = sense_bit;


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

void Pyro::update(minerva_II_packet *packet, elapsedMillis millis) {
    if (millis > start_time) {
        digitalWrite(this->fire_pin, LOW);
        packet->status = clearBit(packet->status, this->fire_bit);
    } else 
        packet->status = setBit(packet->status, this->fire_bit);
    
    if (digitalRead(sense_pin) == 1 && packet->pyro_voltage_v > 1)
        packet->status = setBit(packet->status, this->sense_bit);
    else
        packet->status = clearBit(packet->status, this->sense_bit);

}

