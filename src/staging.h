#ifndef staging_H
#define staging_H

#include <Arduino.h>
#include <structs.h>

enum Stage {
    on_pad,
    powered_ascent,
    coasting_ascent,
    apogee,
    uncontrolled_descent,
    drogue_descent,
    main_descent,
    landed,
};
Stage stage;

const unsigned int loop_rate_hz = 600;
const unsigned int main_deploy_height_m = 100;

const float minimum_apogee_height_m = 75.0F;
const float minimum_launch_g = 4.0F;

const float apogee_requirement_ms = 1.0F;

const float launched_requirement_s = 0.5F;
const float coasting_ascent_requirement_s = 0.1F;
const float main_requirement_s = 0.5F;
const float landed_requirement_s = 1.0F;

const float drogue_descent_ms = 35.0f;
const float main_descent_ms = 10.0f;

const unsigned int launched_count_threshold = loop_rate_hz * launched_requirement_s;
const unsigned int coasting_ascent_threshold = loop_rate_hz * coasting_ascent_requirement_s;
const unsigned int apogee_count_threshold = loop_rate_hz * (2.0 * apogee_requirement_ms / 9.81);
const unsigned int main_count_threshold = loop_rate_hz * main_requirement_s;
const unsigned int landed_count_threshold = loop_rate_hz * landed_requirement_s;

unsigned int launched_count = 0;
unsigned int coasting_ascent_count = 0;
unsigned int apogee_count = 0;
unsigned int main_count = 0;
unsigned int landed_count = 0;

void updateStaging(minerva_II_packet packet, Vec3 worldFrameAccVec_ms);

#endif