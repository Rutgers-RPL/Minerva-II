#include "logging.h"


Logging::Logging(minerva_II_packet packet)
{
    delay(1000);
    byte attempts = 1;
    this->sdExists = false;
    while (attempts <= 10 && !this->sdExists) {
        if (!this->sd.begin(SdioConfig(FIFO_SDIO))) {
            packet.status |= 1<<7;
            Serial.print("SD Begin Failed, Attempting "); Serial.print(10 - attempts++); Serial.println(" more tries ...");
            delay(1000);
        } else {
            packet.status &= ~(1<<7);
            Serial.println("\nFIFO SDIO mode.");

            // INIT DATA FILE

            uint16_t n_file = 0;
            this->dataFileName = DATA_FILE_BASE_NAME + "0000" + DATA_FILE_BASE_TYPE;
            while(sd.exists(this->dataFileName))
            {
                n_file++;
                char fIdx[10];
                sprintf(fIdx, "%04d", n_file);
                this->dataFileName = DATA_FILE_BASE_NAME + fIdx + DATA_FILE_BASE_TYPE;
            }

            this->dataFile = sd.open(this->dataFileName, FILE_WRITE);
            Serial.print("Writing data to: ");
            Serial.println(this->dataFileName);
            if (!this->dataFile) {
                packet.status |= 1<<7;
                Serial.println("Failed opening data file.");
                break;
            }

            this->sdExists = true;
        }
    }

    this->max_acc = 0;
    this->max_vel = 0;
    this->max_alt = 0;

    this->bufferCount = 0;
};

void Logging::logM2Packet(const minerva_II_packet packet) {
    this->dataFile.print(packet.magic); this->dataFile.print(","); 
    this->dataFile.print(packet.status); this->dataFile.print(","); 
    this->dataFile.print(packet.time_us); this->dataFile.print(",");
    this->dataFile.print(packet.main_voltage_v); this->dataFile.print(",");
    this->dataFile.print(packet.pyro_voltage_v); this->dataFile.print(",");
    this->dataFile.print(packet.numSatellites); this->dataFile.print(",");
    this->dataFile.print(packet.gpsFixType); this->dataFile.print(",");
    this->dataFile.print(packet.latitude_degrees, 5); this->dataFile.print(",");
    this->dataFile.print(packet.longitude_degrees, 5); this->dataFile.print(",");
    this->dataFile.print(packet.gps_hMSL_m); this->dataFile.print(",");
    this->dataFile.print(packet.barometer_hMSL_m); this->dataFile.print(",");
    this->dataFile.print(packet.temperature_c); this->dataFile.print(",");
    this->dataFile.print(packet.acceleration_x_mss, 3); this->dataFile.print(",");
    this->dataFile.print(packet.acceleration_y_mss, 3); this->dataFile.print(",");
    this->dataFile.print(packet.acceleration_z_mss, 3); this->dataFile.print(",");
    this->dataFile.print(packet.angular_velocity_x_rads, 3); this->dataFile.print(",");
    this->dataFile.print(packet.angular_velocity_y_rads, 3); this->dataFile.print(",");
    this->dataFile.print(packet.angular_velocity_z_rads, 3); this->dataFile.print(",");
    this->dataFile.print(packet.gauss_x, 3); this->dataFile.print(",");
    this->dataFile.print(packet.gauss_y, 3); this->dataFile.print(",");
    this->dataFile.print(packet.gauss_z, 3); this->dataFile.print(",");
    this->dataFile.print(packet.kf_acceleration_mss, 3); this->dataFile.print(",");
    this->dataFile.print(packet.kf_velocity_ms, 3); this->dataFile.print(",");
    this->dataFile.print(packet.kf_position_m, 3); this->dataFile.print(",");
    this->dataFile.print(packet.w, 3); this->dataFile.print(",");
    this->dataFile.print(packet.x, 3); this->dataFile.print(",");
    this->dataFile.print(packet.y, 3); this->dataFile.print(",");
    this->dataFile.print(packet.z, 3); this->dataFile.print(",");
    this->dataFile.print(packet.checksum); this->dataFile.print(",");
    this->dataFile.println();
};

void Logging::printM2Packet(const minerva_II_packet packet) {
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
};

minerva_II_packet Logging::empty_M2_packet()
{
    minerva_II_packet test_packet;
    test_packet.time_us = 0;
    test_packet.main_voltage_v = 0;
    test_packet.pyro_voltage_v = 0;
    test_packet.numSatellites = 0;
    test_packet.gpsFixType = 0;
    test_packet.latitude_degrees = 0;
    test_packet.longitude_degrees = 0;
    test_packet.gps_hMSL_m = 0;
    test_packet.barometer_hMSL_m = 0;
    test_packet.temperature_c = 0;
    test_packet.acceleration_x_mss = 0;
    test_packet.acceleration_y_mss = 0;
    test_packet.acceleration_z_mss = 0;
    test_packet.angular_velocity_x_rads = 0;
    test_packet.angular_velocity_y_rads = 0;
    test_packet.angular_velocity_z_rads = 0;
    test_packet.gauss_x = 0;
    test_packet.gauss_y = 0;
    test_packet.gauss_z = 0;
    test_packet.kf_acceleration_mss = 0;
    test_packet.kf_velocity_ms = 0;
    test_packet.kf_position_m = 0;
    test_packet.w = 0;
    test_packet.x = 0;
    test_packet.y = 0;
    test_packet.z = 0;
    test_packet.checksum = 0;

  return test_packet;
}