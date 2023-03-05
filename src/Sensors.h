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
#include <structs.h>

#include <BMI088.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#define FILE_BASE_NAME "FlightLog_"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

const uint8_t batteryPin = 21;
const uint8_t pyroBatteryPin = 22;
const uint8_t baro_i2c_address = 0x46;
const uint8_t gps_i2c_address = 0x42;

const uint8_t acc_cs = 37;
const uint8_t gyr_cs = 36;
const uint8_t mag_cs = 38;

const float hard_iron[3] = {
  -8.93,
  -31.33,
  37.49,
};
const float soft_iron[3][3] = {
  {0.908, -0.007, -0.021},
  {-0.006, 0.992, 0.016},
  {-0.020, 0.016, 1.109}
};

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
        }

        void init(minerva_II_packet packet) {
            // Turns off digital input logic circuitry so it doesn't interfere with the analog signal
            pinMode(batteryPin, INPUT_DISABLE);
            pinMode(pyroBatteryPin, INPUT_DISABLE);


            // magRot = Quaternion::from_euler_rotation(0, 0, 0);
            // magRot = Quaternion::from_euler_rotation(0, 0, (-1 * PI)/ 2.0);
            allRot = Quaternion::from_euler_rotation(0.0, PI/2.0, 0.0);

            int status;
            status = accel.begin();
            status = accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_280HZ);
            while (status < 0) {
                Serial.println("Accel Initialization Error");
                Serial.println(status);
                delay(1000);
                status = accel.begin();
            }
            accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_280HZ);
            accel.pinModeInt1(Bmi088Accel::PUSH_PULL,Bmi088Accel::ACTIVE_HIGH);
            accel.mapDrdyInt1(true);

            status = gyro.begin();
            while (status < 0) {
                Serial.println("Gyro Initialization Error");
                Serial.println(status);
                delay(1000);
                status = gyro.begin();
            }
            gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);
            gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL,Bmi088Gyro::ACTIVE_HIGH);
            gyro.mapDrdyInt3(true);

            Wire2.begin();
            status = baro.beginI2C(baro_i2c_address, Wire2);
            status = baro.setODRFrequency(BMP5_ODR_240_HZ);
            while (status != BMP5_OK) {
                Serial.println("Baro Initialization Error");
                Serial.println(status);
                delay(1000);
                status = baro.beginI2C(baro_i2c_address, Wire2);
            }
            baro.setODRFrequency(BMP5_ODR_240_HZ);
            BMP581_InterruptConfig interruptConfig =
            {
                .enable   = BMP5_INTR_ENABLE,    // Enable interrupts
                .drive    = BMP5_INTR_PUSH_PULL, // Push-pull or open-drain
                .polarity = BMP5_ACTIVE_HIGH,    // Active low or high
                .mode     = BMP5_PULSED,         // Latch or pulse signal
                .sources  =
                {
                    .drdy_en = BMP5_ENABLE,        // Trigger interrupts when data is ready
                    .fifo_full_en = BMP5_DISABLE,  // Trigger interrupts when FIFO is full
                    .fifo_thres_en = BMP5_DISABLE, // Trigger interrupts when FIFO threshold is reached
                    .oor_press_en = BMP5_DISABLE    // Trigger interrupts when pressure goes out of range
                }
            };
            baro.setInterruptConfig(&interruptConfig);

            SPI1.setCS(mag_cs);
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
            mag.softReset();
            mag.performResetOperation();
            mag.enableAutomaticSetReset();
            mag.setFilterBandwidth(800);
            mag.setContinuousModeFrequency(1000);
            mag.enableContinuousMode();
            mag.enableInterrupt();
            
            byte attempts = 1;
            while (!gps.begin(Wire2, gps_i2c_address) && attempts <= 10) {
                packet.status |= 1<<6;
                Serial.print("GPS Initialization Error, Retrying "); Serial.print(10-attempts); Serial.println(" more times.");
                attempts++;
                delay(1000);
            }
            if (attempts < 11) {
                packet.status &= ~(1<<6);
            }

            gps.setI2COutput(COM_TYPE_UBX);
            gps.setDynamicModel(DYN_MODEL_AIRBORNE4g);
            gps.setNavigationFrequency(18);
            gps.saveConfiguration();

            Serial.println("Sensor initialization complete...");

        }

        Vec3 readAccel(){
            /* read the accel */
            accel.readSensor();
            Quaternion q(accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss());
            
            //q = imuRot.rotate(q);
            q = allRot.rotate(q);

            return Vec3(q.b, q.c, q.d);
        }

        Vec3 readGyro(){
            gyro.readSensor();
            Quaternion q(gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads());
            // q = imuRot.rotate(q);
            q = allRot.rotate(q);
            return Vec3(q.b, q.c, q.d);
        }

        Vec3 readMag(){
            uint32_t raw[3];
            uint32_t temp[3];
            uint32_t cal[3];
            float normalized[3];
            // use readMeasurement if not using interrupts
            mag.readFieldsXYZ(&raw[0], &raw[1], &raw[2]);

            for (int i = 0; i < 3; i++) {
                temp[i] = raw[i] - hard_iron[i];
            }

            for (int i = 0; i < 3; i++) {
                cal[i] = (soft_iron[i][0] * temp[0]) +
                         (soft_iron[i][1] * temp[1]) +
                         (soft_iron[i][2] * temp[2]);
            }
            normalized[0] = (double)raw[0] - 131072.0;
            normalized[0] /= 131072.0;
            normalized[0] *= 8.0;

            normalized[1] = (double)raw[1] - 131072.0;
            normalized[1] /= 131072.0;
            normalized[1]*= 8.0;

            normalized[2] = (double)raw[2] - 131072.0;
            normalized[2] /= 131072.0;
            normalized[2] *= 8.0;

            Quaternion q(normalized[0], normalized[1], -1.0 * normalized[2]);
            // q = magRot.rotate(q);
            q = allRot.rotate(q);
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
        float readTemperature(){;
            return accel.getTemperature_C();//(accel.getTemperature_C() + accel.getTemperature_C()) / 2.0;
        }

        float readBatteryVoltage(){
            return 4.0 * ((float) analogRead(batteryPin) * (3.3 / 1024.0));
        }

        float readPyroBatteryVoltage() {
            return 4.0 * ((float) analogRead(pyroBatteryPin) * (3.3 / 1024.0));
        }


        void beginSD(minerva_II_packet packet) {
            delay(1000);
            byte attempts = 1;
            while (attempts <= 10) {
                if (!sd.begin(SdioConfig(FIFO_SDIO))) {
                    packet.status |= 1<<7;
                    Serial.print("SD Begin Failed, Attempting "); Serial.print(10 - attempts++); Serial.println(" more tries ...");
                    delay(1000);
                } else {
                    packet.status &= ~(1<<7);
                    Serial.println("\nFIFO SDIO mode.");

                    // Enumerates File Name
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
                        packet.status |= 1<<7;
                        Serial.println("Failed opening file.");
                        break;
                    }
                    sdexists = true;
                    break;
                }
            }
        }

        void logPacket(const minerva_II_packet packet) {
            f.print(packet.magic); f.print(","); 
            f.print(packet.status); f.print(","); 
            f.print(packet.time_us); f.print(",");
            f.print(packet.main_voltage_v); f.print(",");
            f.print(packet.pyro_voltage_v); f.print(",");
            f.print(packet.numSatellites); f.print(",");
            f.print(packet.gpsFixType); f.print(",");
            f.print(packet.latitude_degrees); f.print(",");
            f.print(packet.longitude_degrees); f.print(",");
            f.print(packet.gps_hMSL_m); f.print(",");
            f.print(packet.barometer_hMSL_m); f.print(",");
            f.print(packet.temperature_c); f.print(",");
            f.print(packet.acceleration_x_mss); f.print(",");
            f.print(packet.acceleration_y_mss); f.print(",");
            f.print(packet.acceleration_z_mss); f.print(",");
            f.print(packet.angular_velocity_x_rads); f.print(",");
            f.print(packet.angular_velocity_y_rads); f.print(",");
            f.print(packet.angular_velocity_z_rads); f.print(",");
            f.print(packet.gauss_x); f.print(",");
            f.print(packet.gauss_y); f.print(",");
            f.print(packet.gauss_z); f.print(",");
            f.print(packet.kf_acceleration_mss); f.print(",");
            f.print(packet.kf_velocity_ms); f.print(",");
            f.print(packet.kf_position_m); f.print(",");
            f.print(packet.w); f.print(",");
            f.print(packet.x); f.print(",");
            f.print(packet.y); f.print(",");
            f.print(packet.z); f.print(",");
            f.print(packet.checksum); f.print(",");
            f.println();
        }

        void printPacket(const minerva_II_packet packet) {
            Serial.print(packet.magic); Serial.print("\t"); 
            Serial.print(packet.status); Serial.print("\t"); 
            Serial.print(packet.time_us); Serial.print("\t");
            Serial.print(packet.main_voltage_v); Serial.print("\t");
            Serial.print(packet.pyro_voltage_v); Serial.print(",");
            Serial.print(packet.numSatellites); Serial.print("\t");
            Serial.print(packet.gpsFixType); Serial.print("\t");
            Serial.print(packet.latitude_degrees); Serial.print("\t");
            Serial.print(packet.longitude_degrees); Serial.print("\t");
            Serial.print(packet.gps_hMSL_m); Serial.print("\t");
            Serial.print(packet.barometer_hMSL_m); Serial.print("\t");
            Serial.print(packet.temperature_c); Serial.print("\t");
            Serial.print(packet.acceleration_x_mss); Serial.print("\t");
            Serial.print(packet.acceleration_y_mss); Serial.print("\t");
            Serial.print(packet.acceleration_z_mss); Serial.print("\t");
            Serial.print(packet.angular_velocity_x_rads); Serial.print("\t");
            Serial.print(packet.angular_velocity_y_rads); Serial.print("\t");
            Serial.print(packet.angular_velocity_z_rads); Serial.print("\t");
            Serial.print(packet.gauss_x); Serial.print("\t");
            Serial.print(packet.gauss_y); Serial.print("\t");
            Serial.print(packet.gauss_z); Serial.print("\t");
            Serial.print(packet.kf_acceleration_mss); Serial.print("\t");
            Serial.print(packet.kf_velocity_ms); Serial.print("\t");
            Serial.print(packet.kf_position_m); Serial.print("\t");
            Serial.print(packet.w); Serial.print("\t");
            Serial.print(packet.x); Serial.print("\t");
            Serial.print(packet.y); Serial.print("\t");
            Serial.print(packet.z); Serial.print("\t");
            Serial.print(packet.checksum); Serial.print("\t");
            Serial.println();
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