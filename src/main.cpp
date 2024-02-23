/**
 * @file main.cpp
 * @author Shivam Patel (shivam.patel94@rutgers.edu), Carlton Wu (carlton.wu@rutgers.edu)
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
#include <Sensors.h>
#include <kf.h>
#include <BasicLinearAlgebra.h>
#include <structs.h>
#include <MadgwickAHRS.h>
#include <pyro.h>
#include <state.h>

#define _g_ (9.80665)

#define PYRO0_FIRE 19
#define PYRO0_CONN 20
#define PYRO1_FIRE 17
#define PYRO1_CONN 18
#define PYRO2_FIRE 15
#define PYRO2_CONN 16
#define PYRO3_FIRE 41
#define PYRO3_CONN 14

float radioHZ = 10;
#define sdLogHZ 500
#define sdSaveHZ 10

minerva_II_packet packet;

FastCRC32 CRC32;

Madgwick AHRS;
//Ahrs thisahrs;

union ArrayToInteger {
  char array[4];
  uint32_t integer;
};

ArrayToInteger converter;

short magic = 0xBEEF;

bool first_loop = true;
unsigned long offset_time = 0;
unsigned long previous_time = 0;

float test_long = 0;
float test_lat = 0;

uint32_t bCount = 0;
uint32_t aCount = 0;
uint32_t gCount = 0;
uint32_t mCount = 0;
uint32_t tCount = 0;
uint32_t gpsCount = 0;

const int led = 0;
bool ledOn;
bool firstGPS = true;

float initialAltitude = 0.0;
float initialHMSL = 0.0;
elapsedMicros file_flush_time;
elapsedMicros file_log_time;
elapsedMicros printTime;
elapsedMicros kfTime;
elapsedMicros packetTime;
elapsedMicros mainTime;
elapsedMicros blinkTime;
elapsedMillis pyroMillis;

Pyro p0 = Pyro(PYRO0_FIRE, PYRO0_CONN, 32, 31);
Pyro p1 = Pyro(PYRO1_FIRE, PYRO1_CONN, 30, 29);
Pyro p2 = Pyro(PYRO2_FIRE, PYRO2_CONN, 28, 27);
Pyro p3 = Pyro(PYRO3_FIRE, PYRO3_CONN, 26, 25);

Sensors sen;
KalmanFilter kf;

State state = State(100, 35, 15, 0.1, 600, p1, p2, p3);
u_int16_t stateFlags;

void printPVTData(UBX_NAV_PVT_data_t *ubxDataStruct){
  gpsCount++;
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

  sen.init(packet);
  sen.beginSD(packet);

  byte attempts = 1;
  while (!gps.setAutoPVTcallbackPtr(&printPVTData) && attempts <= 10) {
    Serial.println(packet.status);
    packet.status |= 1<<6;
    delay(1000);
    Serial.print("GPS Callback attachement failed, Retrying "); Serial.print(10 - attempts++); Serial.println(" more times ...");
  }
  if (attempts < 11) {
    packet.status &= ~(1<<6);
  }
  Serial.println(packet.status);
  pinMode(baro_int_pin, INPUT);
  pinMode(mag_int_pin, INPUT);
  pinMode(acc_int_pin, INPUT);
  pinMode(gyro_int_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(baro_int_pin), baroInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(mag_int_pin), magInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(acc_int_pin), accInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(gyro_int_pin), gyroInterruptHandler, RISING); 

  initialAltitude = sen.readAltitude();

  state.init(initialAltitude);
  stateFlags = state.fetch();

  Serial.print("Running Main Loop.");

  AHRS.begin(600);
}

Quaternion orientation = Quaternion();

void loop() {
  if (first_loop) {
    first_loop = false;
    offset_time = micros();
  }
  packet.time_us = mainTime - offset_time;

  gps.checkUblox(); // Check for the arrival of new data and process it.
  gps.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  if (blinkTime >= 50000) {
    blinkTime = 0;
    if (ledOn) {
      ledOn = false;
      digitalWrite(led, LOW);
    } else {
      ledOn = true;
      digitalWrite(led, HIGH);
    }
  }

  kf.predict(kfTime / 1000000.0);
  kfTime = 0;
  packet.kf_acceleration_mss = kf.X(0, 2);
  packet.kf_velocity_ms = kf.X(0, 1);
  packet.kf_position_m = kf.X(0, 0);

  if (baro_interrupt) {
    baro_interrupt = false;
    bCount++;
    packet.barometer_hMSL_m = sen.readAltitude();
    packet.temperature_c = sen.readTemperature();

    kf.H = {1.0, 0.0, 0.0};
    kf.update(kfTime / 1000000.0, sen.readAltitude() - initialAltitude);
    kfTime = 0;
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
    // thisahrs.update(accVec,gyrVec,magVec);
    // orientation = thisahrs.q;

    packet.acceleration_x_mss = accVec.x;
    packet.acceleration_y_mss = accVec.y;
    packet.acceleration_z_mss = accVec.z;
    packet.angular_velocity_x_rads = gyrVec.x;
    packet.angular_velocity_y_rads = gyrVec.y;
    packet.angular_velocity_z_rads = gyrVec.z;
    packet.gauss_x = magVec.x;
    packet.gauss_y = magVec.y;
    packet.gauss_z = magVec.z;
    AHRS.update(gyrVec.x, gyrVec.y, gyrVec.z, accVec.x, accVec.y, accVec.z, magVec.x, magVec.y, magVec.z);
    packet.w = AHRS.q0;
    packet.x = AHRS.q1;
    packet.y = AHRS.q2;
    packet.z = AHRS.q3;
    // packet.w = orientation.a;
    // packet.x = orientation.b;
    // packet.y = orientation.c;
    // packet.z = orientation.d;
    Quaternion q;
    q.a = packet.w;
    q.b = packet.x;
    q.c = packet.y;
    q.d = packet.z;

    Quaternion worldFrame = q.rotate(Quaternion(packet.acceleration_x_mss, packet.acceleration_y_mss, packet.acceleration_z_mss));
    Vec3 accWorldVec(worldFrame.b, worldFrame.c, worldFrame.d);
    Vec3 gravityVec(0.0, 0.0, -1.0 * _g_);
    accWorldVec = accWorldVec + gravityVec;
    
    kf.H = {0.0, 0.0, 1.0};
    kf.update(kfTime / 1000000.0, accWorldVec.z);
    kfTime = 0;
  }

  u_int16_t newStateFlags = state.update(packet.kf_acceleration_mss, packet.kf_velocity_ms, packet.kf_position_m, pyroMillis, mainTime);
  if (stateFlags != newStateFlags)
  {
    state_packet s_packet = state.dump();
    sen.logBinaryPacket(&s_packet, sizeof(state_packet));
  }

  if (file_log_time >= (1000000.0 / sdLogHZ)) {
    file_log_time = 0;
    if (sen.sdexists && sen.f) {
      packet.status &= ~(1<<7);
      //sen.logPacket(packet);
      sen.logBinaryPacket(&packet, sizeof(minerva_II_packet));
    } else {
      // sets 1st bit of code to true
      packet.status |= (1<<7);
    }
    
  }

  packet.main_voltage_v = sen.readBatteryVoltage();
  packet.pyro_voltage_v = sen.readPyroBatteryVoltage();

  if (sen.sdexists && file_flush_time >= (1000000.0 / sdSaveHZ)) {
    file_flush_time = 0;
    sen.f.flush();
  }

  if (printTime >= 50000) {
    printTime = 0;
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

  while (Serial.available() >= 9 || Serial2.available() >= 9) {
    char message[4];
    message[3] = '\0';
    char checksum[4];

    Serial.println("Reading");

    if (Serial.available() >= 9) {
      if (Serial.read() == 0x44) {
        if (Serial.read() == 0x55) {
          Serial.readBytes(message, 3);
          Serial.readBytes(checksum, 4);
        } else {
          break;
        }
      } else {
        break;
      }
    } else if (Serial2.available() >= 9) {
      if (Serial2.read() == 0x44) {
        if (Serial2.read() == 0x55) {
          Serial2.readBytes(message, 3);
          Serial2.readBytes(checksum, 4);
        } else {
          break;
        }
      } else {
        break;
      }
    }

    uint32_t cs = CRC32.crc32((const uint8_t *)message, 3);
        
    converter.array[0] = checksum[0];
    converter.array[1] = checksum[1];
    converter.array[2] = checksum[2];
    converter.array[3] = checksum[3];
    // Serial.println(CRC32.crc32((const uint8_t *)"P0H", 3));
    // Serial.println(converter.integer);
    // Serial.println(cs);
    if (converter.integer == cs) {
      // Serial.println(message);
      // Serial.println(strcmp(message, "H0P"));
      if (strcmp(message, "P0H") == 0) {
        p0.fire(pyroMillis, 2 * 1000);
      }
      if (strcmp(message, "P1H") == 0) {
        p1.fire(pyroMillis, 2 * 1000);
      }
      if (strcmp(message, "P2H") == 0) {
        p2.fire(pyroMillis, 2 * 1000);
      }
      if (strcmp(message, "P3H") == 0) {
        p3.fire(pyroMillis, 2 * 1000);
      }    
      if (strcmp(message, "CAM") == 0) {
        // 10 minutes
        p0.fire(pyroMillis, 10*60*1000);
        p2.fire(pyroMillis, 10*60*1000);
      }
      if (strcmp(message, "FUL") == 0) {
        radioHZ = 10;
      }
      if (strcmp(message, "SAV") == 0) {
        radioHZ = 0.5;
      }
    }
  }

  p0.update(&packet, pyroMillis);
  p1.update(&packet, pyroMillis);
  p2.update(&packet, pyroMillis);
  p3.update(&packet, pyroMillis);
 if (packetTime >= 1000000.0 / radioHZ) {
  packetTime = 0;
  packet.checksum = CRC32.crc32((const uint8_t *)&packet+sizeof(short), sizeof(minerva_II_packet) - 6);
  Serial.write((const uint8_t *)&packet, sizeof(minerva_II_packet));
  Serial2.write((const uint8_t *)&packet, sizeof(minerva_II_packet));
 }
 tCount++;
}