#include "state.h"

State::State(double arm_time, double arm_alt, double arm_vel, double arm_acc, double del_drogue, double dep_main_alt, Pyro drogue, Pyro main, Pyro sust)
{
    this->arming_acc = arm_acc;
    this->arming_vel = arm_vel;
    this->arming_delay = arm_time;

    this->drogue_delay = del_drogue;
    this->main_alt = dep_main_alt+arm_alt;

    this->apogee_time = 0;
    this->drogue_time = 0;
    this->main_time = 0;
    this->sus_time = 0;

    this->state = 0;

    this->drogue_channel = &drogue;
    this->main_channel = &main;
    this->sus_channel = &sust;
};

void State::init()
{
    this->init_alt = 0;
    this->last_alt = 0;
    this->arming_altitude = this->arming_altitude;

}

uint16_t State::update(double acc, double vel, double alt, elapsedMillis pyro_time, uint32_t curr_time)
{
    if(!this->armed())
    {
        if((!this->checkState(REACHED_ARMING_ALTITUDE)) && alt > this->arming_altitude)
        {
            this->setState(REACHED_ARMING_ALTITUDE);
        }
        if((!this->checkState(REACHED_ARMING_VELOCITY)) && vel > this->arming_vel)
        {
            this->setState(REACHED_ARMING_VELOCITY);
        }
        if((!this->checkState(REACHED_ARMING_ACCELERATION)) && acc > this->arming_acc)
        {
            this->setState(REACHED_ARMING_ACCELERATION);
        }
        if((!this->checkState(REACHED_ARMING_DELAY)) && curr_time > this->arming_delay)
        {
            this->setState(REACHED_ARMING_DELAY);
        }

        if(this->state == 0b11110)
        {
            this->setState(ARMED);
            this->arming_time = curr_time;
        }
    }
    else
    {

        //TODO: data population for flags + structs
        if((!this->checkState(FIRED_MAIN)) && vel < 0 && alt <= this->main_alt)
        {
            // this->main_channel->fire(pyro_time, 2 * 1000);
            this->setState(FIRED_MAIN);
            this->main_time = curr_time;
        }

        if((!this->checkState(FIRED_DROGUE)) && (vel < 2 && vel > 2)) // velocity condition + sanity check
        {
            this->apg_detection_sum++;
        }
        else if (this->apg_detection_sum > 0)
        {
            this->apg_detection_sum--;
        }

        if((!this->checkState(REACHED_APOGEE)) && ((exp(0.2*this->apg_detection_sum)-1)/(exp(0.2*this->apg_detection_sum)+1) > 0.8)) // needs approx 11 net "good" datapoints
        {
            // say that drogue should deploy using state, then once there have been enough elapsed millis, check to fire channel if drogue_deploy time is still 0
            this->setState(REACHED_APOGEE);
            this->apogee_time = curr_time;
        }

        if(!this->checkState(FIRED_DROGUE) && (this->checkState(REACHED_APOGEE)) && curr_time >= (this->apogee_time + this->drogue_delay))
        {
            this->setState(FIRED_DROGUE);
            this->drogue_time = curr_time;
        }
        

    }
    return state;


};

uint16_t State::fetch()
{
    return this->state;
};

bool State::armed(){
    return (bool) (this->state & ARMED);
}

bool State::checkState(u_int8_t flag)
{
    return (bool) (this->state & flag);
}

state_packet State::dump()
{
    state_packet s;
    s.magic = 0xFA77;
    s.state_flags = this->state;
    s.arming_time = this->arming_time;
    s.apogee_time = this->apogee_time;
    s.drogue_time = this->drogue_time;
    s.main_time = this->main_time;
    s.sus_time = this->sus_time;

    return s;
}

void State::setState(uint8_t flag)
{
    this->state |= flag;
}