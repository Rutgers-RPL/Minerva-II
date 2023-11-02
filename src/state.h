#ifndef State_H
#define State_H

#include <pyro.h>
#include <Vec3.h>
#include <Quaternion.h>
#include <BasicLinearAlgebra.h>
#include <structs.h>

class State{
    public:
        Quaternion orientation;
        double acc_z;
        double vel_z;
        double pos_z;

        State(double arm_alt,double  arm_jump, double arm_acc, double del_drogue, double dep_main_alt);
        uint16_t update(Quaternion q, double acc, double vel, double, double alt);
        uint16_t fetch();
        state_packet dump();
        bool armed();

    private:
        // arming conditions
        double arming_altitude;
        double arming_delta;
        double arming_acc;

        // deploy conditins
        double drogue_delay;
        double main_alt;

        // Sustainer Ignition (Not Implemented)
        double max_angle;
        double min_altitude;
        double min_time;

        
        /*
        state & (1 << 0) -> armed
        state & (1 << 1) -> reached arming altitude
        state & (1 << 2) -> reached arming delta
        state & (1 << 3) -> reached arming acceleration
        state & (1 << 4) -> reached apogee
        state & (1 << 5) -> fired drogue
        state & (1 << 6) -> fired main
        state & (1 << 7) -> fired sustainer (not used [yet])
        state & (1 << 8) -> 
        state & (1 << 9) -> 
        state & (1 << 10) -> 
        state & (1 << 11) -> 
        state & (1 << 12) -> 
        state & (1 << 13) -> 
        state & (1 << 14) -> 
        state & (1 << 15) -> 
        */
        u_int16_t state;

        u_int32_t apogee_time;
        u_int32_t drogue_time;
        u_int32_t main_time;
        u_int32_t sus_time;

};

#endif