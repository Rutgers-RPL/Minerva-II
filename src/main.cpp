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
  float gps_alt;
  float temp; // 4 bytes - 58
  float w; // 4 bytes - 62
  float x; // 4 bytes - 66
  float y; // 4 bytes - 70
  float z; // 4 bytes - 74
  unsigned int checksum; // 4 bytes - 78
} __attribute__((packed)) realPacket;

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
int start;
double accCutoff = 0.05;
double velocity[3];
double position[3];

const int led = 13;
long blinkCounter;
bool ledOn;

Ahrs thisahrs;
Sensors sen;
bool firstGPS = true;
float xOffset = 0.0;
float yOffset = 0.0;
float zOffset = 0.0;
float lat = 0.0;
float lon = 0.0;

void printECEFData(UBX_NAV_POSECEF_data_t *ubxDataStruct) {
  if (firstGPS) {
    xOffset = ubxDataStruct->ecefX/100.0;
    yOffset = ubxDataStruct->ecefY/100.0;
    zOffset = ubxDataStruct->ecefZ/100.0;
    firstGPS = false;
  } else {
    Serial.println();
    Serial.print("Time:\t"); Serial.println(ubxDataStruct->iTOW);
    Serial.println("\t\tX\tY\tZ");
    Serial.print("ECEF (m):\t"); Serial.print(ubxDataStruct->ecefX/100.0 - xOffset); Serial.print("\t"); Serial.print(ubxDataStruct->ecefY/100.0 - yOffset); Serial.print("\t"); Serial.println(ubxDataStruct->ecefZ/100.0 - zOffset);
  }
}
  void printPVATData(UBX_NAV_PVAT_data_t *ubxDataStruct){
    Serial.println("I made it to the callback");
    if(firstGPS) {
      lat = ubxDataStruct->lat;
      lon = ubxDataStruct->lon;
      firstGPS=false;
    } else {
      Serial.println();
      Serial.print("Latitude: "); Serial.println(lat);
      Serial.print("Longitude: "); Serial.println(lon);
    }
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
  Serial.println("Starting ...");
  Serial.println(gps.setAutoNAVPVATcallbackPtr(&printPVATData));
  //Serial.print("Got data");
  //gps.setAutoNAVPOSECEFcallbackPtr(&printECEFData);
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
long lastTime = micros();
double threshold = 0.05;

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

  // if (baro_interrupt) {
  //   bCount++;
  //   baro_interrupt = false;
  // }
  // if (acc_interrupt) {
  //   aCount++;
  //   acc_interrupt = false;
  // }
  // if (gyro_interrupt) {
  //   gCount++;
  //   gyro_interrupt = false;
  // }
  // if (mag_interrupt) {
  //   mCount++;
  //   mag_interrupt = false;
  //   mag.clearMeasDoneInterrupt();
  // }

  if (acc_interrupt && gyro_interrupt && mag_interrupt) {
    aCount++;
    gCount++;
    mCount++;
    acc_interrupt = false;
    gyro_interrupt = false;
    mag_interrupt = false;
    Vec3 magr = sen.readMag();
    mag.clearMeasDoneInterrupt();
    Vec3 acc = sen.readAccel();
    Vec3 gyr = sen.readGyro();
    thisahrs.update(acc,gyr,magr);
    orientation = thisahrs.q;
  }

  if (baro_interrupt) {
    bCount++;
    baro_interrupt = false;
  }

  Quaternion groundToSensorFrame = orientation;


  //data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  
  // if (sen.sdexists && sen.f) {
  //   sen.f.print(data.time); sen.f.print(","); sen.f.print(data.code); sen.f.print(","); sen.f.print(data.voltage); sen.f.print(",");
  //   sen.f.print(acc.x); sen.f.print(","); sen.f.print(acc.y); sen.f.print(","); sen.f.print(acc.z); sen.f.print(",");
  //   sen.f.print(data.accx); sen.f.print(","); sen.f.print(data.accy); sen.f.print(","); sen.f.print(data.accz); sen.f.print(",");
  //   sen.f.print(data.avelx); sen.f.print(","); sen.f.print(data.avely); sen.f.print(","); sen.f.print(data.avelz); sen.f.print(",");
  //   sen.f.print(data.magx); sen.f.print(","); sen.f.print(data.magy); sen.f.print(","); sen.f.print(data.magz); sen.f.print(",");
  //   sen.f.print(data.baro_alt); sen.f.print(","); sen.f.print(data.temp); sen.f.print(",");
  //   sen.f.print(data.w); sen.f.print(","); sen.f.print(data.x); sen.f.print(","); sen.f.print(data.y); sen.f.print(","); sen.f.print(data.z); sen.f.println(",");
  // } else {
  //   //Serial.println("No sd writing");
  //   data.code = -1;
  //   data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  // }

  if (sen.sdexists) {
      //sen.f.flush();
  }
    //count++;
  if (micros() - lastTime >= 1000000) {
    Serial.println("\tAcc:\tGyro:\tBaro:\tMag:");
    Serial.print("HZ:\t"); Serial.print(aCount); Serial.print("\t"); Serial.print(gCount); Serial.print("\t"); Serial.print(bCount); Serial.print("\t"); Serial.println(mCount);
    lastTime = micros();
    aCount = 0;
    gCount = 0;
    bCount = 0;
    mCount = 0;
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

    if (sen.sdexists) {
      sen.f.flush();
    }
  }

  count += 1;
  */
}