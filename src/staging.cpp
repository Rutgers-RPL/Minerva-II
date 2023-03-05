#include <staging.h>
#include <Vec3.h>

void updateStaging(minerva_II_packet packet, Vec3 worldFrameAccVec_mss) {
    switch (stage) {
        case on_pad:
            if (worldFrameAccVec_mss.magnitude() > (minimum_launch_g * 9.81)) {
                if (launched_count++ >= launched_count_threshold) {
                    stage = powered_ascent;
                }
            } else {
                if (launched_count > 0) {
                    launched_count--;
                }
            }
            break;
        case powered_ascent:
            if (worldFrameAccVec_mss.magnitude() < (minimum_launch_g * 9.81)) {
                if (coasting_ascent_count++ >= coasting_ascent_threshold) {
                    stage = coasting_ascent;
                }
            } else {
                if (coasting_ascent_count > 0) {
                    coasting_ascent_count--;
                }
            }
            break;
        case coasting_ascent:
            if (packet.kf_position_m > minimum_apogee_height_m && fabs(packet.kf_velocity_ms) < apogee_requirement_ms) {
                if (apogee_count++ >= apogee_count_threshold) {
                    stage = apogee;
                }
            } else {
                if (apogee_count > 0) {
                    apogee_count--;
                }
            }
            break;
        case apogee:
            if (apogee_count-- == 0) {
                //boom
                stage = uncontrolled_descent;
            }
            break;
        case uncontrolled_descent:
        case drogue_descent:
        case main_descent:
            if (packet.kf_position_m < main_deploy_height_m) {
                if (main_count++ >= main_count_threshold) {
                    //boom
                }
                else {
                    if (main_count > 0) {
                        main_count--;
                    }
                }
            }
            float velocity = fabs(packet.kf_velocity_ms);

            
            if (velocity < main_descent_ms) {
                stage = main_descent;
            } else if (velocity < drogue_descent_ms) {
                stage = drogue_descent;
            } else {
                stage = uncontrolled_descent;
            }


            break;
        case landed:
            // something
            break;


    }
}