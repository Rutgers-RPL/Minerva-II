#ifndef structs_H
#define structs_H

#include <Arduino.h>

typedef struct {
  u_int16_t magic;               // 2 bytes -   2
  u_int32_t status;              // 4 byte  -   6
  u_int32_t time_us;             // 4 bytes -  10
  float main_voltage_v;          // 4 bytes -  14
  float pyro_voltage_v;          // 4 bytes -  18
  uint8_t numSatellites;         // 1 byte  -  19
  uint8_t gpsFixType;            // 1 byte  -  20
  float latitude_degrees;        // 4 bytes -  24
  float longitude_degrees;       // 4 bytes -  28
  float gps_hMSL_m;              // 4 bytes -  32
  float barometer_hMSL_m;        // 4 bytes -  36
  float temperature_c;           // 4 bytes -  40
  float acceleration_x_mss;      // 4 bytes -  44
  float acceleration_y_mss;      // 4 bytes -  48
  float acceleration_z_mss;      // 4 bytes -  52
  float angular_velocity_x_rads; // 4 bytes -  56
  float angular_velocity_y_rads; // 4 bytes -  60
  float angular_velocity_z_rads; // 4 bytes -  64
  float gauss_x;                 // 4 bytes -  68
  float gauss_y;                 // 4 bytes -  72
  float gauss_z;                 // 4 bytes -  76
  float kf_acceleration_mss;     // 4 bytes -  80
  float kf_velocity_ms;          // 4 bytes -  84
  float kf_position_m;           // 4 bytes -  88
  float w;                       // 4 bytes -  92
  float x;                       // 4 bytes -  96
  float y;                       // 4 bytes - 100
  float z;                       // 4 bytes - 104
  unsigned int checksum;         // 4 bytes - 108
} __attribute__((packed)) minerva_II_packet;

typedef struct  {
  uint16_t magic;
  uint16_t state_flags;
  uint32_t arming_time;
  u_int32_t apogee_time;
  u_int32_t drogue_time;
  u_int32_t main_time;
  u_int32_t sus_time;
} __attribute__((packed)) state_packet;

#endif