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
bool firstGPS = true;
float xOffset = 0.0;
float yOffset = 0.0;
float zOffset = 0.0;

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

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  //while(!Serial) {}
  //while(!Serial2) {}
  sen.beginSD();
  Serial.println("test");
  Serial2.flush();
  Serial.println("Starting ...");
  gps.setAutoNAVPOSECEFcallbackPtr(&printECEFData);
}



Quaternion orientation = Quaternion();
long lastTime = micros();
double threshold = 0.05;

void loop() {
  gps.checkUblox(); // Check for the arrival of new data and process it.
  gps.checkCallbacks(); // Check if any callbacks are waiting to be processed.
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

  
  realPacket data = {0xBEEF, (micros()-offset) / 1000000.0, 0, sen.readVoltage(), thisahrs.aglobal.b, thisahrs.aglobal.c, thisahrs.aglobal.d,
                      gyr.x, gyr.y, gyr.z, mag.x, mag.y, mag.z, sen.readAltitude(),
                      (sen.readTemperature()) / 1.0, groundToSensorFrame.a, groundToSensorFrame.b, groundToSensorFrame.c, groundToSensorFrame.d};

  //Serial.printf("(%f, %f, %f)\n", data.accx, data.accy, data.accz);

  data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  
  if (sen.sdexists && sen.f) {
    sen.f.print(data.time); sen.f.print(","); sen.f.print(data.code); sen.f.print(","); sen.f.print(data.voltage); sen.f.print(",");
    sen.f.print(acc.x); sen.f.print(","); sen.f.print(acc.y); sen.f.print(","); sen.f.print(acc.z); sen.f.print(",");
    sen.f.print(data.accx); sen.f.print(","); sen.f.print(data.accy); sen.f.print(","); sen.f.print(data.accz); sen.f.print(",");
    sen.f.print(data.avelx); sen.f.print(","); sen.f.print(data.avely); sen.f.print(","); sen.f.print(data.avelz); sen.f.print(",");
    sen.f.print(data.magx); sen.f.print(","); sen.f.print(data.magy); sen.f.print(","); sen.f.print(data.magz); sen.f.print(",");
    sen.f.print(data.altitude); sen.f.print(","); sen.f.print(data.temp); sen.f.print(",");
    sen.f.print(data.w); sen.f.print(","); sen.f.print(data.x); sen.f.print(","); sen.f.print(data.y); sen.f.print(","); sen.f.print(data.z); sen.f.println(",");
  } else {
    data.code = -1;
    data.checksum = CRC32.crc32((const uint8_t *)&data+sizeof(short), sizeof(realPacket) - 6);
  
  }

  if (count % 15 == 0) {
    //Serial.write((const uint8_t *)&data, sizeof(data));
    Serial2.write((const uint8_t *)&data, sizeof(data));

    if (sen.sdexists) {
      sen.f.close();
      sen.f = sen.sd.open(sen.fileName, FILE_WRITE);
    }
  }

  count += 1;
}