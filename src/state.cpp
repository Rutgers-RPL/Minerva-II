#include "state.h"

State::State(double arm_alt, double arm_vel, double arm_acc, double del_drogue, double dep_main_alt, double curr_alt, Pyro drogue, Pyro main, Pyro sust)
{
    this->last_alt = curr_alt;
    this->init_alt = curr_alt;

    this->arming_acc = arm_acc;
    this->arming_vel = arm_vel;
    this->arming_altitude = arm_alt+curr_alt;

    this->drogue_delay = del_drogue;
    this->main_alt = dep_main_alt+curr_alt;

    u_int32_t apogee_time = 0;
    u_int32_t drogue_time = 0;
    u_int32_t main_time = 0;
    u_int32_t sus_time = 0;

    this->state = 0;

    this->drogue_channel = &drogue;
    this->main_channel = &main;
    this->sus_channel = &sust;


};

uint16_t State::update(Quaternion q, float dt, double acc, double vel, double alt, elapsedMillis pyro_time)
{
    if(!(this->state & 1))
    {
        if((!(this->state & (1<<1))) && alt > this->arming_altitude)
        {
            this->state |= (1<<1);
        }
        if((!(this->state & (1<<2))) && vel > this->arming_vel)
        {
            this->state |= (1<<2);
        }
        if((!(this->state & (1<<3))) && acc > this->arming_acc)
        {
            this->state |= (1<<3);
        }

        if(this->state & 0b111)
        {
            this->state != 1;
        }
    }
    else
    {

        //TODO: data population for flags + structs
        if(vel < 0 && alt <= this->main_alt)
        {
            this->main_channel->fire(pyro_time, 2 * 1000);
        }

        if((vel < 1 && vel > 1) && alt > this->arming_altitude) // velocity condition + sanity check
        {
            this->apg_detection_sum++;
        }
        else if (this->apg_detection_sum > 0)
        {
            this->apg_detection_sum--;
        }

        if((exp(0.2*this->apg_detection_sum)-1)/(exp(0.2*this->apg_detection_sum)+1) > 0.8) // needs approx 11 net "good" datapoints
        {
            // say that drogue should deploy using state, then once there have been enough elapsed millis, check to fire channel if drogue_deploy time is still 0
        }

        

    }

    


};

uint16_t State::fetch()
{
    return this->state;
};

bool State::armed(){
    return (this->state & (1 << 0)) == 1;
}

state_packet State::dump()
{
    state_packet s;
    s.magic = 0xCDA1;
    s.state_flags = this->state;
    s.apogee_time = this->apogee_time;
    s.drogue_time = this->drogue_time;
    s.main_time = this->main_time;
    s.sus_time = this->sus_time;

    return s;
};