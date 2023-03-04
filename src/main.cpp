/**
 * @file main.cpp
 * @author Shivam Patel (shivam.patel94@rutgers.edu), Carlton Wu (carlton.wu@rutgers.edu), William Freitag (william.h.freitag@gmail.com)
 * @brief This runs the main data collection, processing, and transmission loop for the Minerva II flight computer
 * @version 1.0
 * @date 2022-09-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <Arduino.h>
#include <SdFat.h>
#include <FastCRC.h>
#include <Quaternion.h>
#include <Vec3.h>
#include <Ahrs.h>
#include <Sensors.h>
#include <kf.h>
#include <BasicLinearAlgebra.h>
#include <structs.h>

#define _g_ (9.80665)

#define radioHZ 10
#define sdLogHZ 500
#define sdSaveHZ 10

minerva_II_packet packet;

FastCRC32 CRC32;

short magic = 0xBEEF;

bool first_loop = true;
unsigned long offset_time = 0;
unsigned long previous_time = 0;

uint32_t bCount = 0;
uint32_t aCount = 0;
uint32_t gCount = 0;
uint32_t mCount = 0;
uint32_t tCount = 0;
uint32_t gpsCount = 0;

const int led = 0;
long blinkCounter;
bool ledOn;
bool firstGPS = true;

float initialAltitude = 0.0;
float initialHMSL = 0.0;
unsigned long file_flush_time = 0;
unsigned long file_log_time = 0;
unsigned long lastTime = 0;
unsigned long printTime = 0;
unsigned long kfTime = 0;
unsigned long packetTime = 0;
Ahrs thisahrs;
Sensors sen;
KalmanFilter kf;
uint32_t gpsTime = 0;

void printPVTData(UBX_NAV_PVT_data_t *ubxDataStruct){
  gpsCount++;
  gpsTime = ubxDataStruct->iTOW;
  packet.gps_hMSL_m = ubxDataStruct->hMSL / 1000.0;
  packet.numSatellites = ubxDataStruct->numSV;
  packet.latitude_degrees = ubxDataStruct->lat * 1e-7;
  packet.longitude_degrees = ubxDataStruct->lon * 1e-7;
  packet.gpsFixType = ubxDataStruct->fixType;
}

const uint8_t acc_int_pin = 9;
volatile bool acc_interrupt = false;
const uint8_t gyro_int_pin = 10;
volatile bool gyro_interrupt = false;
const uint8_t baro_int_pin = 31;
volatile bool baro_interrupt = false;
const uint8_t mag_int_pin = 40;
volatile bool mag_interrupt = true;

void baroInterruptHandler() {
    baro_interrupt = true;
}

void magInterruptHandler() {
    mag_interrupt = true;
}

void accInterruptHandler() {
    acc_interrupt = true;
}

void gyroInterruptHandler() {
    gyro_interrupt = true;
}

void setup() {
  packet.magic = magic;
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.flush();
  Serial2.flush();

  sen.beginSD();

  byte attempts = 1;
  while (!gps.setAutoPVTcallbackPtr(&printPVTData) && attempts <= 10) {
    delay(1000);
    Serial.print("GPS Callback attachement failed, Retrying "); Serial.print(10 - attempts++); Serial.println(" more times ...");
  }
  
  pinMode(baro_int_pin, INPUT);
  pinMode(mag_int_pin, INPUT);
  pinMode(acc_int_pin, INPUT);
  pinMode(gyro_int_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(baro_int_pin), baroInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(mag_int_pin), magInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(acc_int_pin), accInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(gyro_int_pin), gyroInterruptHandler, RISING); 

  initialAltitude = sen.readAltitude();

  Serial.print("Runnign Main Loop.");
}

Quaternion orientation = Quaternion();

void loop() {
  if (first_loop) {
    first_loop = false;
    offset_time = micros();
  }
  packet.time_us = micros() - offset_time;

  gps.checkUblox(); // Check for the arrival of new data and process it.
  gps.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  if (millis() - blinkCounter >= 500) {
    if (ledOn) {
      ledOn = false;
      digitalWrite(led, LOW);
    } else {
      ledOn = true;
      digitalWrite(led, HIGH);
    }
    blinkCounter = millis();
  }

  kf.predict((micros() - kfTime) / 1000000.0);
  kfTime = micros();
  packet.kf_acceleration_mss = kf.X(0, 2);
  packet.kf_velocity_ms = kf.X(0, 1);
  packet.kf_position_m = kf.X(0, 0);

  if (baro_interrupt) {
    baro_interrupt = false;
    bCount++;
    packet.barometer_hMSL_m = sen.readAltitude();

    kf.H = {1.0, 0.0, 0.0};
    kf.update((micros() - kfTime) / 1000000.0, sen.readAltitude() - initialAltitude);
    kfTime = micros();
  }

  if (acc_interrupt && gyro_interrupt && mag_interrupt) {
    mag.clearMeasDoneInterrupt();
    acc_interrupt = false;
    gyro_interrupt = false;
    mag_interrupt = false;
    aCount++;
    gCount++;
    mCount++;
    Vec3 magVec = sen.readMag();
    Vec3 accVec = sen.readAccel();
    Vec3 gyrVec = sen.readGyro();
    thisahrs.update(accVec,gyrVec,magVec);
    orientation = thisahrs.q;
    packet.temperature_c = sen.readTemperature();
    packet.acceleration_x_mss = accVec.x;
    packet.acceleration_y_mss = accVec.y;
    packet.acceleration_z_mss = accVec.z;
    packet.angular_velocity_x_rads = gyrVec.x;
    packet.angular_velocity_y_rads = gyrVec.y;
    packet.angular_velocity_z_rads = gyrVec.z;
    packet.gauss_x = magVec.x;
    packet.gauss_y = magVec.y;
    packet.gauss_z = magVec.z;
    packet.w = orientation.a;
    packet.x = orientation.b;
    packet.y = orientation.c;
    packet.z = orientation.d;

    kf.H = {0.0, 0.0, 1.0};
    kf.update((micros() - kfTime) / 1000000.0, thisahrs.aglobal.d);
    kfTime = micros();
  }

  if (micros() - file_log_time >= (1000000 / sdLogHZ)) {
    file_log_time = micros();
    if (sen.sdexists && sen.f) {
      sen.logPacket(packet);
    } else {
      //Serial.println("No sd writing");
      // data.code = -1;
      // data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
    }
    
  }

  packet.main_voltage_v = sen.readBatteryVoltage();


  if (sen.sdexists && (micros() - file_flush_time >= (1000000 / sdSaveHZ))) {
    file_flush_time = micros();
    sen.f.flush();
  }

  if (micros() - lastTime >= 50000) {
    lastTime = micros();
    //Serial.write((const uint8_t *)&packet, sizeof(minerva_II_packet));
    //Serial.println();
    //Serial.println("\tLoop:\tAcc:\tGyro:\tBaro:\tMag:\tGPS:");
    //Serial.print("HZ:\t"); Serial.print(tCount); Serial.print("\t"); Serial.print(aCount); Serial.print("\t"); Serial.print(gCount); Serial.print("\t"); Serial.print(bCount); Serial.print("\t"); Serial.print(mCount); Serial.print("\t"); Serial.println(gpsCount);
    // Serial.print("Int:\t\t"); Serial.print(acc_interrupt); Serial.print("\t"); Serial.print(gyro_interrupt); Serial.print("\t"); Serial.print(baro_interrupt); Serial.print("\t"); Serial.println(mag_interrupt);
    // Serial.println("\tVolt:\thMSL:\tLat:\tLon:\tFix:\tSat:\tBaro:");
    // Serial.print("Packet:\t");  Serial.print(packet.voltage_v); Serial.print("\t"); Serial.print(packet.gps_hMSL_m); Serial.print("\t"); Serial.print(packet.latitude_degrees); Serial.print("\t"); Serial.print(packet.longitude_degrees); Serial.print("\t"); Serial.print(packet.gpsFixType); Serial.print("\t"); Serial.print(packet.numSatellites); Serial.print("\t"); Serial.println(packet.barometer_hMSL_m);
    // //sen.printPacket(packet);
    // Serial.println("\tAcc:\tVel:\tPos:");
    // Serial.print("KF:\t"); Serial.print(kf.X(0,2)); Serial.print("\t"); Serial.print(kf.X(0,1)); Serial.print("\t"); Serial.println(kf.X(0,0));
    // float heading = atan2(packet.gauss_x, 0 - packet.gauss_y);

    // heading /= PI;
    // heading *= 180;
    // heading += 180;

    // Serial.print("Heading: ");
    // Serial.println(heading, 1);

    tCount = 0;
    aCount = 0;
    gCount = 0;
    bCount = 0;
    mCount = 0;
    gpsCount = 0;
    //Serial.println(packet.temperature_c);
    //Serial2.print("Kalman Filter Altitude: "); Serial2.println(packet.kf_position_m);
    //Serial.write((const uint8_t *)&packet, sizeof(minerva_II_packet));
    //Serial.println(sizeof(packet));
  }

 if (micros() - packetTime >= 100000) {
  packet.time_us = micros();
  packet.checksum = CRC32.crc32((const uint8_t *)&packet+sizeof(short), sizeof(minerva_II_packet) - 6);
  Serial.write((const uint8_t *)&packet, sizeof(minerva_II_packet));
  Serial2.write((const uint8_t *)&packet, sizeof(minerva_II_packet));
  packetTime = micros();
 }
 tCount++;
}