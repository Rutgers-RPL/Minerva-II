/**
 * @file state.cpp
 * @author Shivam Patel (shivam.patel94@rutgers.edu)
 * @brief This detects and records the estimated state of the rocket over the course of the flight
 * @version 1.1
 * @date 2024-04-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "state.h"


//setter! Assign packet values figma

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



//takes all necessary data to update (acc, vel, alt) all at once
uint16_t State::update(double acc, double vel, double alt, elapsedMillis pyro_time, uint32_t curr_time)
{
    
    //if not in armed state
    if(!this->armed())
    {
        //utilizing bitmask

        //if arming altitude bit isn't flagged ON and the arming altitude has been passed
        if((!this->checkState(REACHED_ARMING_ALTITUDE)) && alt > this->arming_altitude)
        {
            //flip the bit
            this->setState(REACHED_ARMING_ALTITUDE);
        }
        //if arming velocity hasn't been flagged ON and the arming velocity has been reached
        if((!this->checkState(REACHED_ARMING_VELOCITY)) && vel > this->arming_vel)
        {
            //flip the bit
            this->setState(REACHED_ARMING_VELOCITY);
        }
        //If the arming acceleration bit isn't flagged ON and the arming acceleration has been reached
        if((!this->checkState(REACHED_ARMING_ACCELERATION)) && acc > this->arming_acc)
        {
            //flip the bit
            this->setState(REACHED_ARMING_ACCELERATION);
        }
        //arming delay: the time between when a command is issued and when it is executed.
        //if it's been long enough to execute the arming command, and the flag isn't raised yet,
        if((!this->checkState(REACHED_ARMING_DELAY)) && curr_time > this->arming_delay)
        {
            //raise the flag to execute the command (flag indicates that arming delay has been reached)
            this->setState(REACHED_ARMING_DELAY);
        }
//
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

        if((!this->checkState(FIRED_DROGUE)) && (vel < 2 && vel > -2)) // velocity condition + sanity check
        {
            this->apg_detection_sum++;
        }
        else if (this->apg_detection_sum > 0)
        {
            this->apg_detection_sum--;
        }

        if((!this->checkState(REACHED_APOGEE)) && (this->apg_detection_sum > 11)) // needs approx 11 net "good" datapoints
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

//getter
uint16_t State::fetch()
{
    return this->state;
};

//function to return arming state (checks first bit)
bool State::armed(){
    return (bool) (this->state & ARMED);
}

//checks bit at given flag 
bool State::checkState(u_int8_t flag)
{
    return (bool) (this->state & flag);
}


static bool checkState(state_packet packet, uint8_t flag)
{
    return (bool) (packet.state_flags & flag);
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