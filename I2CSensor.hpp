#pragma once

#include <Wire.h>
#include "I2CManager.hpp"
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <mutex>


using UniqueTimedMutex = std::unique_lock<std::timed_mutex>;


class I2CSensor {
    public:
        // The init method orchestrates the registration with the I2CManager.
        virtual bool init() {
            if (!I2CManager::getInstance().registerSensor(this)) {
                // Handle registration failure
                _is_initialized = false;
                return false;
            }
            // Perform device-specific setup (e.g., writing to config registers)
            if (!deviceSpecificSetup()) {
                 _is_initialized = false;
                return false;
            }
            _is_initialized = true;
            return true;
        }
    
        // Pure virtual functions to be implemented by derived sensor classes
        virtual void update() = 0; // For reading new data from the sensor
    
        // Getters for sensor configuration, used by the I2CManager
        uint8_t getAddress() const { return _i2c_addr; }
        int getBusNum() const { return _bus_num; }
        int getSdaPin() const { return _sda_pin; }
        int getSclPin() const { return _scl_pin; }
        uint32_t getMinClock() const { return _min_clock_hz; }
        uint32_t getMaxClock() const { return _max_clock_hz; }
        bool isInitialized() const { return _is_initialized; }
    
        // Setter for the manager to provide the configured TwoWire instance
        void setWire(TwoWire* wire) { _wire = wire; }
    
    protected:
        // Constructor for derived classes
        I2CSensor(uint8_t addr, int bus_num, int sda, int scl, uint32_t min_clk, uint32_t max_clk)
            : _i2c_addr(addr), _bus_num(bus_num), _sda_pin(sda), _scl_pin(scl),
              _min_clock_hz(min_clk), _max_clock_hz(max_clk) {}
    
        // Device-specific initialization called after bus is configured
        virtual bool deviceSpecificSetup() = 0;
    
        // Member variables
        uint8_t _i2c_addr;
        int _bus_num;
        int _sda_pin;
        int _scl_pin;
        uint32_t _min_clock_hz;
        uint32_t _max_clock_hz;
        mutable std::timed_mutex _i2cMutex;
        static constexpr std::chrono::milliseconds I2C_TIMEOUT_MS{100};
        static constexpr TickType_t I2C_DELAY_MS = 5 / portTICK_PERIOD_MS;
        static constexpr TickType_t I2C_INIT_DELAY_MS = 250 / portTICK_PERIOD_MS;
    
        bool _is_initialized = false;
        TwoWire* _wire = nullptr; // The manager will set this

        /**
         * Writes a single byte to the specified register.
         * @param reg The register address to write to.
         * @param value The byte value to write.
         */
        void writeToReg(uint8_t reg, uint8_t value) const {
            UniqueTimedMutex lock(_i2cMutex, std::defer_lock);
            if (lock.try_lock_for(I2C_TIMEOUT_MS)) {
                wire.beginTransmission(_i2c_addr);
                wire.write(reg);
                wire.write(value);
                wire.endTransmission();
            } else {
                // TODO: Some logging - will handle later after base functionality is working
            }
            vTaskDelay(I2C_DELAY_MS);
        }

        /**
         * Writes a single byte to the specified register without any values.
         * This is useful for operations that only require a register address.
         * @param reg The register address to write to.
         */
        void writeToReg(uint8_t reg) const {
            UniqueTimedMutex lock(_i2cMutex, std::defer_lock);
            if (lock.try_lock_for(I2C_TIMEOUT_MS)) {
                wire.beginTransmission(_i2c_addr);
                wire.write(reg);
                wire.endTransmission();
            } else {
                // TODO: Some logging - will handle later after base functionality is working
            }
            vTaskDelay(I2C_DELAY_MS);
        }
        
        /**
         * Writes a command to the sensor.
         * The command is a 16-bit value split into two bytes.
         * @param cmd The command to write, represented as a 16-bit unsigned integer.
         */
        void writeCommand(uint16_t cmd) const {
            std::array<uint8_t, 2> cmdBytes = {
                static_cast<uint8_t>(cmd >> 8), // High byte
                static_cast<uint8_t>(cmd & 0xFF) // Low byte
            };

            UniqueTimedMutex lock(i2cMutex, std::defer_lock);
            if (lock.try_lock_for(I2C_TIMEOUT_MS)) {
                wire.beginTransmission(this->address);
                wire.write(cmdBytes.data(), cmdBytes.size());
                wire.endTransmission();
            } else {
                // TODO: Some logging - will handle later after base functionality is working
            }

            vTaskDelay(I2C_DELAY_MS);
        }
    };