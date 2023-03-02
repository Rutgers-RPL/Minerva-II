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

#define _g_ (9.80665)

typedef struct {
  short magic; // 2 bytes - 2
  float time; // 4 bytes - 6
  byte code; // 1 bytes - 11
  float voltage; // 4 bytes - 14
  float accx; // 4 bytes - 18
  float accy; // 4 bytes - 22
  float accz; // 4 bytes - 26
  float avelx; // 4 bytes - 30
  float avely; // 4 bytes - 34
  float avelz; // 4 bytes - 38
  float magx; // 4 bytes - 42
  float magy; // 4 bytes - 46
  float magz; // 4 bytes - 50
  float baro_alt; // 4 bytes - 54
  float temp; // 4 bytes - 58
  float w; // 4 bytes - 62
  float x; // 4 bytes - 66
  float y; // 4 bytes - 70
  float z; // 4 bytes - 74
  byte numSatellites;
  byte gpsFixType;
  float latitude;
  float longitude;
  float hMSL;
  float kfAcc;
  float kfVel;
  float kfPos;
  unsigned int checksum; // 4 bytes - 78
} __attribute__((packed)) realPacket;

realPacket packet;

FastCRC32 CRC32;

unsigned long offset = 0;
unsigned long previousTime = 0;

double am[3];
double wm[3];
int count = 0;
uint32_t bCount = 0;
uint32_t aCount = 0;
uint32_t gCount = 0;
uint32_t mCount = 0;
uint32_t tCount = 0;
uint32_t gpsCount = 0;
int start;
double accCutoff = 0.05;
double velocity[3];
double position[3];

const int led = 0;
long blinkCounter;
bool ledOn;
bool firstGPS = true;

float initialAltitude = 0.0;
float initialHMSL = 0.0;
uint32_t accTime = 0;
uint32_t altTime = 0;
uint32_t lastTime = 0;
uint32_t printTime = 0;
uint32_t kfTime = 0;
Ahrs thisahrs;
Sensors sen;
KalmanFilter kf;
uint32_t gpsTime = 0;

void printPVTData(UBX_NAV_PVT_data_t *ubxDataStruct){
  gpsCount++;
  if (ubxDataStruct->iTOW == gpsTime) {
    Serial.println("Too fast for GPS.");
  } else {
    //Serial.println("GPS Good.");
  }
  gpsTime = ubxDataStruct->iTOW;
  packet.hMSL = ubxDataStruct->hMSL / 1000.0;
  packet.numSatellites = ubxDataStruct->numSV;
  packet.latitude = ubxDataStruct->lat * 1e-7;
  packet.longitude = ubxDataStruct->lon * 1e-7;
  packet.gpsFixType = ubxDataStruct->fixType;
  //Serial.println(ubxDataStruct->velD / 1000.0);
  // if (firstGPS && packet.gpsFixType != 0) {
  //   firstGPS = false;
  //   initialHMSL = packet.hMSL;
  // }
  // if (!firstGPS) {
  //   //Serial.println(packet.hMSL - initialHMSL);
  //   kf.H = {1.0, 0.0, 0.0};
  //   kf.update((micros() - kfTime) / 1000000.0, packet.hMSL - initialHMSL);
  //   kfTime = micros();
  // }
}
void printPVATData(UBX_NAV_PVAT_data_t *ubxDataStruct){
  Serial.print("Latitude: "); Serial.println(ubxDataStruct->lat);
  Serial.print("Longitude: "); Serial.println(ubxDataStruct->lon);
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
  Serial.begin(115200);
  Serial2.begin(115200);
  //while(!Serial) {}
  //while(!Serial2) {}
  sen.beginSD();
  Serial.println("test");
  Serial2.flush();
  delay(500);
  initialAltitude = sen.readAltitude();
  Serial.println("Starting ...");
  while (!gps.setAutoPVTcallbackPtr(&printPVTData)) {
    delay(1000);
    Serial.println("Retrying...");
  }
  initialAltitude = sen.readAltitude();
  pinMode(baro_int_pin, INPUT);
  pinMode(mag_int_pin, INPUT);
  pinMode(acc_int_pin, INPUT);
  pinMode(gyro_int_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(baro_int_pin), baroInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(mag_int_pin), magInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(acc_int_pin), accInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(gyro_int_pin), gyroInterruptHandler, RISING); 
}

Quaternion orientation = Quaternion();

void loop() {
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
    packet.accx = accVec.x;
    packet.accy = accVec.y;
    packet.accz = accVec.z;
    packet.avelx = gyrVec.x;
    packet.avely = gyrVec.y;
    packet.avelz = gyrVec.z;
    packet.magx = magVec.x;
    packet.magy = magVec.y;
    packet.magz = magVec.z;
    packet.w = orientation.a;
    packet.x = orientation.b;
    packet.y = orientation.c;
    packet.z = orientation.d;

    //kf.predict((micros() - kfTime) / 1000000.0);
    kf.H = {0.0, 0.0, 1.0};
    kf.update((micros() - kfTime) / 1000000.0, thisahrs.aglobal.d);
    kfTime = micros();
  }

  if (baro_interrupt) {
    baro_interrupt = false;
    bCount++;
    packet.baro_alt = sen.readAltitude();
    //kf.predict((micros() - kfTime) / 1000000.0);
    kf.H = {1.0, 0.0, 0.0};
    kf.update((micros() - kfTime) / 1000000.0, packet.baro_alt - initialAltitude);
    kfTime = micros();
  }

  Quaternion groundToSensorFrame = orientation;

  //data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  
  if (sen.sdexists && sen.f) {
    sen.f.print(packet.time); sen.f.print(","); sen.f.print(packet.code); sen.f.print(","); sen.f.print(packet.voltage); sen.f.print(",");
    // sen.f.print(acc.x); sen.f.print(","); sen.f.print(acc.y); sen.f.print(","); sen.f.print(acc.z); sen.f.print(",");
    sen.f.print(packet.accx); sen.f.print(","); sen.f.print(packet.accy); sen.f.print(","); sen.f.print(packet.accz); sen.f.print(",");
    sen.f.print(packet.avelx); sen.f.print(","); sen.f.print(packet.avely); sen.f.print(","); sen.f.print(packet.avelz); sen.f.print(",");
    sen.f.print(packet.magx); sen.f.print(","); sen.f.print(packet.magy); sen.f.print(","); sen.f.print(packet.magz); sen.f.print(",");
    sen.f.print(packet.baro_alt); sen.f.print(","); sen.f.print(packet.temp); sen.f.print(",");
    sen.f.print(packet.w); sen.f.print(","); sen.f.print(packet.x); sen.f.print(","); sen.f.print(packet.y); sen.f.print(","); sen.f.print(packet.z); sen.f.println(",");
  } else {
    //Serial.println("No sd writing");
    // data.code = -1;
    // data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  }

  if (sen.sdexists && micros() - lastTime >= 1000000) {
    sen.f.flush();
  }
    //count++;
  if (micros() - lastTime >= 1000000) {
    Serial.println();
    Serial.println("\tLoop:\tAcc:\tGyro:\tBaro:\tMag:\tGPS:");
    Serial.print("HZ:\t"); Serial.print(tCount); Serial.print("\t"); Serial.print(aCount); Serial.print("\t"); Serial.print(gCount); Serial.print("\t"); Serial.print(bCount); Serial.print("\t"); Serial.print(mCount); Serial.print("\t"); Serial.println(gpsCount);
    Serial.print("Int:\t\t"); Serial.print(acc_interrupt); Serial.print("\t"); Serial.print(gyro_interrupt); Serial.print("\t"); Serial.print(baro_interrupt); Serial.print("\t"); Serial.println(mag_interrupt);
    Serial.println("\thMSL:\tLat:\tLon:\tFix:\tSat:\tBaro:");
    Serial.print("Packet:\t"); Serial.print(packet.hMSL); Serial.print("\t"); Serial.print(packet.latitude); Serial.print("\t"); Serial.print(packet.longitude); Serial.print("\t"); Serial.print(packet.gpsFixType); Serial.print("\t"); Serial.print(packet.numSatellites); Serial.print("\t"); Serial.println(packet.baro_alt);
    Serial.println("\tAcc:\tVel:\tPos:");
    Serial.print("KF:\t"); Serial.print(kf.X(0,2)); Serial.print("\t"); Serial.print(kf.X(0,1)); Serial.print("\t"); Serial.println(kf.X(0,0));
    lastTime = micros();
    Serial2.println("Hello World.");
    tCount = 0;
    aCount = 0;
    gCount = 0;
    bCount = 0;
    mCount = 0;
    gpsCount = 0;
  }

  
  if (thisahrs.aglobal.b > accCutoff) {
    velocity[0] += thisahrs.aglobal.b * (micros()-lastTime) / 1000000.0;
  }
  if (thisahrs.aglobal.c > accCutoff) {
    velocity[1] += thisahrs.aglobal.c * (micros()-lastTime) / 1000000.0;
  }
  if (thisahrs.aglobal.d > accCutoff) {
    velocity[2] += thisahrs.aglobal.d * (micros()-lastTime) / 1000000.0;
  }

  
  /*
  if (count % 15 == 0) {
    //Serial.write((const uint8_t *)&data, sizeof(data));
    Serial2.write((const uint8_t *)&data, sizeof(data));
  }
  */

 tCount++;
}