#include "state.h"

State::State(double arm_alt, double arm_jump, double arm_acc, double del_drogue, double dep_main_alt)
{
    this->arming_acc = arm_acc;
    this->arming_altitude = arm_alt;
    this->arming_delta = arm_jump;

    this->drogue_delay = del_drogue;
    this->main_alt = dep_main_alt;
};

uint16_t State::update(Quaternion q, double acc, double vel, double, double alt)
{

};

uint16_t State::fetch()
{
    return this->state;
};

bool State::armed(){
    return this->state & (1 << 0);
}

state_packet State::dump()
{

};