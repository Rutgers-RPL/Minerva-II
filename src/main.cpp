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
int start;

const int led = 13;
long blinkCounter;
bool ledOn;

Ahrs thisahrs;
Sensors sen;

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

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  //while(!Serial) {}
  //while(!Serial2) {}
  sen.beginSD();
  Serial.println("test");
  Serial2.flush();
  Serial.println("Starting ...");
  pinMode(baro_int_pin, INPUT);
  pinMode(mag_int_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(baro_int_pin), baroInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(mag_int_pin), magInterruptHandler, RISING);
}

Quaternion orientation = Quaternion();
long lastTime = micros();
double threshold = 0.05;

void loop() {

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

  // /* read the accel */
  Vec3 acc = sen.readAccel();
  // /* read the gyr */
  Vec3 gyr = sen.readGyro();
  // /* read the mag */
  Vec3 magr = sen.readMag();



  thisahrs.update(acc,gyr,magr);
  orientation = thisahrs.q;

  Quaternion groundToSensorFrame = orientation;




  realPacket data = {0xBEEF, (micros()-offset) / 1000000.0, 0, sen.readVoltage(), thisahrs.aglobal.b, thisahrs.aglobal.c, thisahrs.aglobal.d,
                      gyr.x, gyr.y, gyr.z, magr.x, magr.y, magr.z, 0,
                      0, groundToSensorFrame.a, groundToSensorFrame.b, groundToSensorFrame.c, groundToSensorFrame.d};




  //Serial.printf("(%f, %f, %f)\n", data.accx, data.accy, data.accz);

  data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  
  if (sen.sdexists && sen.f) {
    sen.f.print(data.time); sen.f.print(","); sen.f.print(data.code); sen.f.print(","); sen.f.print(data.voltage); sen.f.print(",");
    sen.f.print(acc.x); sen.f.print(","); sen.f.print(acc.y); sen.f.print(","); sen.f.print(acc.z); sen.f.print(",");
    sen.f.print(data.accx); sen.f.print(","); sen.f.print(data.accy); sen.f.print(","); sen.f.print(data.accz); sen.f.print(",");
    sen.f.print(data.avelx); sen.f.print(","); sen.f.print(data.avely); sen.f.print(","); sen.f.print(data.avelz); sen.f.print(",");
    sen.f.print(data.magx); sen.f.print(","); sen.f.print(data.magy); sen.f.print(","); sen.f.print(data.magz); sen.f.print(",");
    sen.f.print(data.baro_alt); sen.f.print(","); sen.f.print(data.temp); sen.f.print(",");
    sen.f.print(data.w); sen.f.print(","); sen.f.print(data.x); sen.f.print(","); sen.f.print(data.y); sen.f.print(","); sen.f.print(data.z); sen.f.println(",");
  } else {
    //Serial.println("No sd writing");
    data.code = -1;
    data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  }

  if (sen.sdexists) {
      //sen.f.close();
      //sen.f = sen.sd.open(sen.fileName, FILE_WRITE);
      //sen.f.flush();
      //sen.f = sen.sd.open(sen.fileName, FILE_WRITE);
    }

  if (baro_interrupt) {
    count++;
    baro_interrupt = false;
  }
  if (mag_interrupt) {
    count++;
    mag_interrupt = false;
    mag.clearMeasDoneInterrupt();
    uint32_t rawValueX = 0;
    uint32_t rawValueY = 0;
    uint32_t rawValueZ = 0;
    mag.readFieldsXYZ(&rawValueX, &rawValueY, &rawValueZ);
  }
    //count++;
  if (micros() - lastTime >= 1000000) {
    Serial.print("HZ:\t"); Serial.println(count);
    lastTime = micros();
    count = 0;
  }

  /*

  if (count % 15 == 0) {
    //Serial.write((const uint8_t *)&data, sizeof(data));
    Serial2.write((const uint8_t *)&data, sizeof(data));

    if (sen.sdexists) {
      sen.f.close();
      sen.f = sen.sd.open(sen.fileName, FILE_WRITE);
    }
  }

  count += 1;
  */
}