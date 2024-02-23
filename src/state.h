#ifndef State_H
#define State_H

#include <pyro.h>
#include <Vec3.h>
#include <Quaternion.h>
#include <BasicLinearAlgebra.h>
#include <structs.h>
#include "math.h"

class State{
    public:
        // Quaternion orientation;
        // double acc_z;
        // double vel_z;
        // double pos_z;

        double last_alt;
        double init_alt;

        Pyro* drogue_channel;
        Pyro* main_channel;
        Pyro* sus_channel;

        State(double arm_alt, double  arm_vel, double arm_acc, double del_drogue, double dep_main_alt, Pyro drogue, Pyro main, Pyro sust);
        void init(double curr_alt);
        uint16_t update(double acc, double vel, double alt, elapsedMillis pyro_time, uint32_t curr_time);
        uint16_t fetch();
        state_packet dump();
        bool armed();

    private:
        // arming conditions
        double arming_altitude;
        double arming_vel;
        double arming_acc;

        // deploy conditins
        double drogue_delay;
        double main_alt;

        // Sustainer Ignition (Not Implemented)
        double max_angle;
        double min_altitude;
        double min_time;
        
        int apg_detection_sum;
        
        /*
        state & (1 << 0) -> armed
        state & (1 << 1) -> reached arming altitude
        state & (1 << 2) -> reached arming velocity
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

        u_int32_t arming_time;

        u_int32_t apogee_time;
        u_int32_t drogue_time;
        u_int32_t main_time;
        u_int32_t sus_time;

};

#endif