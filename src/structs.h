#ifndef structs_H
#define structs_H

#include <Arduino.h>


typedef struct {
  u_int16_t magic;                   // 2 bytes -   2
  u_int32_t status;           // 4 byte  -   3
  u_int32_t time_us;          // 4 bytes -  11
  float main_voltage_v;          // 4 bytes -  15
  float pyro_voltage_v;          // 4 bytes -  19
  uint8_t numSatellites;            // 1 byte  -  16
  uint8_t gpsFixType;               // 1 byte  -  17
  float latitude_degrees;        // 4 bytes -  21
  float longitude_degrees;       // 4 bytes -  25
  float gps_hMSL_m;              // 4 bytes -  29
  float barometer_hMSL_m;        // 4 bytes -  33
  float temperature_c;           // 4 bytes -  37
  float acceleration_x_mss;      // 4 bytes -  41
  float acceleration_y_mss;      // 4 bytes -  45
  float acceleration_z_mss;      // 4 bytes -  49
  float angular_velocity_x_rads; // 4 bytes -  53
  float angular_velocity_y_rads; // 4 bytes -  57
  float angular_velocity_z_rads; // 4 bytes -  61
  float gauss_x;                 // 4 bytes -  65
  float gauss_y;                 // 4 bytes -  69
  float gauss_z;                 // 4 bytes -  73
  float kf_acceleration_mss;     // 4 bytes -  77
  float kf_velocity_ms;          // 4 bytes -  81
  float kf_position_m;           // 4 bytes -  85
  float w;                       // 4 bytes -  89
  float x;                       // 4 bytes -  93
  float y;                       // 4 bytes -  97
  float z;                       // 4 bytes - 101
  unsigned int checksum;         // 4 bytes - 105
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