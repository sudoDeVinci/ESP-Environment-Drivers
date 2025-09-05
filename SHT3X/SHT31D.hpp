#pragma once

#include "I2CSensor.hpp"
#include <Wire.h>
#include <array>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <chrono>

#define SHT31_ADDRESS 0x44

namespace sht {
    /**
     * Enumeration for SHT31D command codes in hexadecimal.
     * These commands are used to interact with the SHT31D sensor.
     */
    enum CMDS {
        READSTATUS = 0xF32D,
        CLEARSTATUS = 0x3041,
        SOFTRESET = 0x30A2,
        HEATER_ON = 0x306D,
        HEATER_OFF = 0x3066,
        HEATER_STATUS = 0x0d
    };

    /**
     * Enumeration for SHT31D measurement modes.
     * These modes determine the frequency and type of measurements taken by the sensor.
     */
    enum MEASUREMENT_MODE {
        LOWREP = 0x2416,
        MEDREP = 0x2C0B,
        HIGHREP = 0x2400,
        LOWREP_STRETCH  = 0x2C10,
        MEDREP_STRETCH = 0x2C0D,
        HIGHREP_STRETCH = 0x2C06, 
    };
};


/**
 * SHT31 class for interfacing with the SHT31D temperature and humidity sensor.
 */
struct SHT31 : public I2CSensor {

    protected:
        /**Whether the heater is enabled or not. */
        bool heaterEnabled;
        /**Internal humidity rep. */
        float humidity;
        /**Internal temperature rep. */
        float temperature;

        bool deviceSpecificSetup() override {
            reset();
            return readStatus() != 0xFFFF;
        }

    public:

        SHT31(
            uint8_t bus_num,
            uint8_t sda,
            uint8_t scl
        ) : I2CSensor(SHT31_ADDRESS, bus_num, sda, scl, 1000, 1000000) {
            this -> humidity = NAN;
            this -> temperature = NAN;
        }

        /**
         * Resets the SHT31 sensor.
         * This method sends a soft reset command to the sensor.
         * It is recommended to call this method after initialization.
         */
        void reset(void) const {
            writeCommand(sht::CMDS::SOFTRESET);
            vTaskDelay(i2cs::I2C_INIT_DELAY_MS);
        }

        /**
         * Enables the heater of the SHT31 sensor.
         * This method sends a command to turn on the heater.
         */
        void enableHeater(void) {
            this -> heaterEnabled = true;
            writeCommand(sht::CMDS::HEATER_ON);
        }

        /**
         * Disables the heater of the SHT31 sensor.
         * This method sends a command to turn off the heater.
         */
        void disableHeater(void) {
            writeCommand(sht::CMDS::HEATER_OFF);
            this -> heaterEnabled = false;
        }

        /**
         * Returns the current humidity reading from the sensor.
         * @return The humidity as a percentage.
         */
        float getHumidity(void) const {
            return this -> humidity;
        }

        /** 
         * Returns the current temperature reading from the sensor.
         * @return The temperature in degrees Celsius.
         */
        float getTemperature(void) const {
            return this -> temperature;
        }

        /**
         * Reads the current status of the SHT31 sensor.
         * This method sends a command to read the status and returns the status code.
         * @return The status code as a 16-bit unsigned integer.
         */
        uint16_t readStatus(void) const;

        /**
         * Reads the temperature and humidity from the SHT31 sensor.
         * This method sends a command to read the temperature and humidity data,
         * processes the data, and updates the internal temperature and humidity values.
         * @return True if the read was successful, false otherwise.
         */
        bool update(void) override;
};
