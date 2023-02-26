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
#include <bkf.h>
#include <BasicLinearAlgebra.h>

#define _g_ (9.80665)

typedef struct {
  short magic; // 2 bytes - 2
  float time; // 4 bytes - 6
  int code; // 4 bytes - 10
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
  float altitude; // 4 bytes - 54
  float temp; // 4 bytes - 58
  float w; // 4 bytes - 62
  float x; // 4 bytes - 66
  float y; // 4 bytes - 70
  float z; // 4 bytes - 74
  // float lat; // 4 bytes - 78
  // float lon; // 4 bytes - 82
  // byte sat; // 4 bytes - 83
  unsigned int checksum; // 4 bytes - 87
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

float initialAltitude = 0.0;
uint32_t accTime = 0.0;
uint32_t altTime = 0.0;
uint32_t lastTime = 0.0;
uint32_t printTime = 0.0;
Ahrs thisahrs;
Sensors sen;
KalmanFilter kf;
BetterKalmanFilter bkf;

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
  accTime = micros();
  altTime = micros();
  printTime = micros();
  lastTime = micros();
}

Quaternion orientation = Quaternion();
//long lastTime = micros();
double threshold = 0.05;

void loop() {
  // if (micros() - accTime >= 625) {
  //   bkf.H = {0.0, 0.0, 1.0};
  //   float acc = thisahrs.aglobal.d;

  //   kf.predict((micros() - lastTime) / 1000000.0, acc);

  //   bkf.predict((micros() - lastTime) / 1000000.0);
  //   bkf.update((micros() - lastTime) / 1000000.0, acc);
  //   accTime = micros();
  //   lastTime = accTime;
  // }

  // if (micros() - altTime >= 4167) {
  //   bkf.H = {1.0, 0.0, 0.0};
  //   float alt = sen.readAltitude() - initialAltitude;

  //   kf.update((micros() - lastTime) / 1000000.0, alt);

  //   bkf.predict((micros() - lastTime) / 1000000.0);
  //   bkf.update((micros() - lastTime) / 1000000.0, alt);
  //   altTime = micros();
  //   lastTime = altTime;
  // }

  // if (micros() - printTime >= 100000) {
  //   Serial.println("\tPos(m) \t Vel(m/s) \t Acc(m/s/s)");
  //   Serial.print("KF\t"); Serial.print(kf.X(0,0)); Serial.print("\t "); Serial.print(kf.X(0,1)); Serial.print("\t\t"); Serial.println(kf.X(0,2));
  //   Serial.print("BKF\t"); Serial.print(bkf.X(0,0)); Serial.print("\t "); Serial.print(bkf.X(0,1)); Serial.print("\t\t"); Serial.println(bkf.X(0,2));
  //   printTime = micros();
  // }
  
  // Vec3 magnetometer_data = sen.readMag();
  // Serial.println("Magneteometer Data: ");Serial.print(magnetometer_data.x);Serial.print(" ");
  // Serial.print(magnetometer_data.y); Serial.print(" "); Serial.println(magnetometer_data.z);
  // Vec3 accelerometer_data = sen.readAccel();
  // Serial.println("Accelerometer Data: ");Serial.print(accelerometer_data.x);Serial.print(" ");
  // Serial.print(accelerometer_data.y); Serial.print(" "); Serial.println(accelerometer_data.z);
  // Vec3 gyroscope_data = sen.readGyro();
  // Serial.println("Gyroscope Data: ");Serial.print(gyroscope_data.x);Serial.print(" ");
  // Serial.print(gyroscope_data.x); Serial.print(" "); Serial.println(gyroscope_data.x);
  // double pressure_data = sen.readPressure();
  // Serial.println("Pressure Data: "); Serial.println(pressure_data);
  // double altitude = sen.readAltitude();
  // Serial.println("Altitude: "); Serial.println(altitude);
  // double temperatureC = sen.readTemperature();
  // Serial.println("Temperature: "); Serial.println(temperatureC);
  // float longitude = sen.readLongitude();
  // float latitude = sen.readLatitude();
  // Serial.println("Altitude (Latitude, Longitude):");
  // Serial.print("("); Serial.print(latitude); Serial.print(", "); Serial.print(longitude); Serial.println(")");
  // delay(10);

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

  /* read the accel */
  Vec3 acc = sen.readAccel();
  /* read the mag */
  Vec3 mag = sen.readMag();

  /* read the gyr */
  Vec3 gyr = sen.readGyro();

  thisahrs.update(acc,gyr,mag);
  orientation = thisahrs.q;

  Quaternion groundToSensorFrame = orientation;

  if (micros() - accTime >= 625) {
    bkf.H = {0.0, 0.0, 1.0};
    float acc = thisahrs.aglobal.d;

    kf.predict((micros() - lastTime) / 1000000.0, acc);

    bkf.predict((micros() - lastTime) / 1000000.0);
    bkf.update((micros() - lastTime) / 1000000.0, acc);
    accTime = micros();
    lastTime = accTime;
  }

  if (micros() - altTime >= 4167) {
    bkf.H = {1.0, 0.0, 0.0};
    float alt = sen.readAltitude() - initialAltitude;

    kf.update((micros() - lastTime) / 1000000.0, alt);

    bkf.predict((micros() - lastTime) / 1000000.0);
    bkf.update((micros() - lastTime) / 1000000.0, alt);
    altTime = micros();
    lastTime = altTime;
  }
  if (micros() - printTime >= 100000) {
    Serial.println("\tPos(m) \t Vel(m/s) \t Acc(m/s/s)");
    Serial.print("KF\t"); Serial.print(kf.X(0,0)); Serial.print("\t "); Serial.print(kf.X(0,1)); Serial.print("\t\t"); Serial.println(kf.X(0,2));
    Serial.print("BKF\t"); Serial.print(bkf.X(0,0)); Serial.print("\t "); Serial.print(bkf.X(0,1)); Serial.print("\t\t"); Serial.println(bkf.X(0,2));
    printTime = micros();
  }
  
  // realPacket data = {0xBEEF, (micros()-offset) / 1000000.0, 0, sen.readVoltage(), thisahrs.aglobal.b, thisahrs.aglobal.c, thisahrs.aglobal.d,
  //                     gyr.x, gyr.y, gyr.z, mag.x, mag.y, mag.z, bkf.X(0,0),
  //                     (sen.readTemperature()) / 1.0, groundToSensorFrame.a, groundToSensorFrame.b, groundToSensorFrame.c, groundToSensorFrame.d};

  // //Serial.printf("(%f, %f, %f)\n", data.accx, data.accy, data.accz);

  // data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  
  // if (sen.sdexists && sen.f) {
  //   sen.f.print(data.time); sen.f.print(","); sen.f.print(data.code); sen.f.print(","); sen.f.print(data.voltage); sen.f.print(",");
  //   sen.f.print(acc.x); sen.f.print(","); sen.f.print(acc.y); sen.f.print(","); sen.f.print(acc.z); sen.f.print(",");
  //   sen.f.print(data.accx); sen.f.print(","); sen.f.print(data.accy); sen.f.print(","); sen.f.print(data.accz); sen.f.print(",");
  //   sen.f.print(data.avelx); sen.f.print(","); sen.f.print(data.avely); sen.f.print(","); sen.f.print(data.avelz); sen.f.print(",");
  //   sen.f.print(data.magx); sen.f.print(","); sen.f.print(data.magy); sen.f.print(","); sen.f.print(data.magz); sen.f.print(",");
  //   sen.f.print(data.altitude); sen.f.print(","); sen.f.print(data.temp); sen.f.print(",");
  //   sen.f.print(data.w); sen.f.print(","); sen.f.print(data.x); sen.f.print(","); sen.f.print(data.y); sen.f.print(","); sen.f.print(data.z); sen.f.println(",");
  // } else {
  //   data.code = -1;
  //   data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  
  // }

  // if (count % 1 == 0) {
  //   Serial.write((const uint8_t *)&data, sizeof(data));
  //   Serial2.write((const uint8_t *)&data, sizeof(data));

  //   if (sen.sdexists) {
  //     sen.f.close();
  //     sen.f = sen.sd.open(sen.fileName, FILE_WRITE);
  //   }
  // }

  count += 1;
}