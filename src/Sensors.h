#ifndef Sensors_H
#define Sensors_H
#include <Arduino.h>
#include <SdFat.h>
#include <Quaternion.h>
#include <Vec3.h>
#include <Mat3x3.h>
#include <EEPROM.h>
#include <filters.h>
#include <string.h>

#include <BMI088.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#define FILE_BASE_NAME "FlightLog_"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

const uint8_t batteryPin = 22;
const uint8_t baro_i2c_address = 0x46;
const uint8_t gps_i2c_address = 0x42;

const uint8_t acc_cs = 37;
const uint8_t gyr_cs = 36;
const uint8_t mag_cs = 38;

/* accel object */
Bmi088Accel accel(SPI, acc_cs);
/* gyro object */
Bmi088Gyro gyro(SPI, gyr_cs);
/* baro object */
BMP581 baro;
/* mag object*/
SFE_MMC5983MA mag;
/* gps object*/
SFE_UBLOX_GNSS gps;



class Sensors{
    public:
        
        Quaternion imuRot;
        Quaternion magRot;
        Quaternion allRot;

        Mat3x3 M;
        Vec3 b;

        SdFs sd;
        FsFile f;
        bool sdexists = false;

        char* fileName = FILE_BASE_NAME "0000.csv";

        Sensors(){
            short x = readShort(0);
            short y = readShort(2);
            short z = readShort(4);

            magRot = Quaternion::from_euler_rotation(0, 0, 0);
            magRot = Quaternion::from_euler_rotation(0, 0, (-1 * PI)/ 2.0);
            allRot = Quaternion::from_euler_rotation(PI/2.0, 0, 0);

            int status;
            status = accel.begin();
            while (status < 0) {
                Serial.println("Accel Initialization Error");
                Serial.println(status);
                delay(1000);
                status = accel.begin();
            }
            status = gyro.begin();
            while (status < 0) {
                Serial.println("Gyro Initialization Error");
                Serial.println(status);
                delay(1000);
                status = gyro.begin();
            }

            Wire2.begin();
            status = baro.beginI2C(baro_i2c_address, Wire2);
            while (status != BMP5_OK) {
                Serial.println("Baro Initialization Error");
                Serial.println(status);
                delay(1000);
                status = baro.beginI2C(baro_i2c_address, Wire2);
            }

            SPI1.setCS(38);
            SPI1.setSCK(27);
            SPI1.setMISO(39);
            SPI1.setMOSI(26);
            SPI1.begin();
            status = mag.begin(mag_cs, SPI1);
            while (!status) {
                Serial.println("Mag Initialization Error");
                Serial.println(status);
                delay(1000);
                status = mag.begin(mag_cs, SPI1);
            }
            mag.performResetOperation();

            status = gps.begin(Wire2, gps_i2c_address);
            while (!status) {
                Serial.println("GPS Initialization Error");
                Serial.println(status);
                delay(1000);
                status = gps.begin(Wire2, gps_i2c_address);
            }
            
            Serial.println("Sensor initialization complete...");
        }

        Vec3 readAccel(){
            /* read the accel */
            accel.readSensor();
            Quaternion q(accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss());
            // q = imuRot.rotate(q);
            // q = allRot.rotate(q);

            return Vec3(q.b, q.c, q.d);
        }

        Vec3 readGyro(){
            gyro.readSensor();

            Quaternion q(gyro.getGyroX_rads(),gyro.getGyroY_rads(),gyro.getGyroZ_rads());
            // q = imuRot.rotate(q);
            // q = allRot.rotate(q);
            return Vec3(q.b, q.c, q.d);
        }

        Vec3 readMag(){
            uint32_t x = mag.getMeasurementX();
            uint32_t y = mag.getMeasurementY();
            uint32_t z = -1.0 * mag.getMeasurementZ();
            //sBmm150MagData_t magData = mag.getGeomagneticData();
            Quaternion q(x, y, z);
            // q = magRot.rotate(q);
            // q = allRot.rotate(q);
            return Vec3(q.b, q.c, q.d);
        }


        float readPressure(){
            bmp5_sensor_data data = {0, 0}; 
            baro.getSensorData(&data);
            return data.pressure;
        }

        // Converts pressure to feet using the following formula: https://en.wikipedia.org/wiki/Pressure_altitude
        float readAltitude(){
            float pascal = readPressure();
            float mbar = pascal/100.0;
            float feet = 145366.46 * (1 - pow((mbar / 1013.25), 0.190284));

            return feet * 0.3048; //Convert to meters
        }

        //Parse string to get latitude and lobgitude
        float readLatitude() {
            float lat = gps.getLatitude();
            return lat;
        }
        float readLongitude() {
            float longitude = gps.getLongitude();
            return longitude;
        }
        float readTemperature(){
            bmp5_sensor_data data = {0, 0}; 
            baro.getSensorData(&data);
            return data.temperature;//(accel.getTemperature_C() + accel.getTemperature_C()) / 2.0;
        }

        float readVoltage(){
            return (15.721519 * ((double) analogRead(batteryPin) / 1023.0));
        }


        void beginSD() {
            if (!sd.begin(SdioConfig(FIFO_SDIO))) {
                Serial.println("SD Begin Failed");
            } else {
                Serial.println("\nFIFO SDIO mode.");
                while (sd.exists(fileName)) {
                    if (fileName[BASE_NAME_SIZE + 3] != '9') {
                        fileName[BASE_NAME_SIZE + 3]++;
                    } else if (fileName[BASE_NAME_SIZE + 2] != '9') {
                        fileName[BASE_NAME_SIZE + 3] = '0';
                        fileName[BASE_NAME_SIZE + 2]++;
                    } else if (fileName[BASE_NAME_SIZE + 1] != '9') {
                        fileName[BASE_NAME_SIZE + 2] = '0';
                        fileName[BASE_NAME_SIZE + 3] = '0';
                        fileName[BASE_NAME_SIZE + 1]++;
                    } else if (fileName[BASE_NAME_SIZE] != '9') {
                        fileName[BASE_NAME_SIZE + 1] = '0';
                        fileName[BASE_NAME_SIZE + 2] = '0';
                        fileName[BASE_NAME_SIZE + 3] = '0';
                        fileName[BASE_NAME_SIZE]++;
                    } else {
                        Serial.println("Can't create file name");
                    }
                }

                f = sd.open(fileName, FILE_WRITE);
                Serial.print("Writing to: ");
                Serial.println(fileName);
                if (!f) {
                    Serial.println("Failed opening file.");
                }
                sdexists = true;
            }
        }


    private:

        short int readShort(short address)
        {
            short b1 = EEPROM.read(address);
            short b2 = EEPROM.read(address+1);
            return (b1<<8)+b2;
        }

        void writeShort(short address, short data)
        {
            EEPROM.write(address, data>>8);
            EEPROM.write(address+1, (data<<8)>>8);
        }

};

#endif