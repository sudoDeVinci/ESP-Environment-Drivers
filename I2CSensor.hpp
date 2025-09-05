#pragma once

#include <Wire.h>
#include "I2CManager.hpp"
#include <array>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <chrono>
#include <mutex>


using UniqueTimedMutex = std::unique_lock<std::timed_mutex>;

namespace i2cs{
    /**
     * AUnit doesn't have RTOS functions so we give whatever values we want here.
     * These technically don't matter for testing.
     */
    #ifdef EPOXY_DUINO
        constexpr uint32_t I2C_DELAY_MS = 5;
        constexpr uint32_t I2C_INIT_DELAY_MS = 250;
    #else
        constexpr uint32_t I2C_DELAY_MS = 5 / portTICK_PERIOD_MS;
        constexpr uint32_t I2C_INIT_DELAY_MS = 250 / portTICK_PERIOD_MS;
    #endif
    constexpr std::chrono::milliseconds I2C_TIMEOUT_MS = std::chrono::milliseconds(100);
    constexpr uint16_t MAX_SAMPLES = 100;
};

/**
 * Base class for I2C sensors.
 * This class provides common functionality for I2C sensors, including initialization and communication methods.
 */
class I2CSensor {
    public:

        virtual bool init() {
            if (!I2CManager::getInstance().registerSensor(*this)) {
                this -> _is_initialized = false;
                return false;
            }
            
            if (!deviceSpecificSetup()) {
                this -> _is_initialized = false;
                return false;
            }

            this -> _is_initialized = true;
            return true;
        }
    
       /**
        * Pure virtual function to be implemented by derived sensor classes.
        * This function is called periodically to update the sensor's readings.
        * Only some sensors do this - other return the reading on demand.
        */
        virtual bool update(void) = 0;
    
        uint8_t getAddress(void) const { return _i2c_addr; }
        int getBusNum(void) const { return _bus_num; }
        int getSdaPin(void) const { return _sda_pin; }
        int getSclPin(void) const { return _scl_pin; }
        uint32_t getMinClock(void) const { return _min_clock_hz; }
        uint32_t getMaxClock(void) const { return _max_clock_hz; }
        uint32_t getMaxSamples(void) const { return i2cs::MAX_SAMPLES; }
        bool isInitialized(void) const { return _is_initialized; }
        void setWire(TwoWire* wire) { _wire = wire; }
        void setInitialized(bool initialized) { _is_initialized = initialized; }

    protected:

        I2CSensor(
            uint8_t addr,
            uint8_t bus_num,
            uint8_t sda,
            uint8_t scl,
            uint32_t min_clk,
            uint32_t max_clk)
            :
            _i2c_addr(addr),
            _bus_num(bus_num),
            _sda_pin(sda),
            _scl_pin(scl),
            _min_clock_hz(min_clk),
            _max_clock_hz(max_clk) {}
    
        /**
         * Pure virtual function to be implemented by derived sensor classes.
         * This function is called during initialization to perform device-specific setup.
         * @return True if the setup is successful, false otherwise.
         */
        virtual bool deviceSpecificSetup() = 0;
    
        bool _is_initialized = false;
        uint8_t _i2c_addr;
        uint8_t _bus_num;
        uint8_t _sda_pin;
        uint8_t _scl_pin;
        uint32_t _min_clock_hz;
        uint32_t _max_clock_hz;
        mutable std::timed_mutex _i2cMutex;

        /**
         * Pointer to the TwoWire instance used for I2C communication.
         * This will be set during registration.
         */
        TwoWire* _wire = nullptr;


        /**
         * @brief Writes a single byte to the specified register.
         * @param reg The register address to write to.
         * @param value The byte value to write.
         */
        void writeToReg(uint8_t reg, uint8_t value) const {
            UniqueTimedMutex lock(_i2cMutex, std::defer_lock);
            if (lock.try_lock_for(i2cs::I2C_TIMEOUT_MS)) {
                _wire->beginTransmission(_i2c_addr);
                _wire->write(reg);
                _wire->write(value);
                _wire->endTransmission();
            } else {
                // TODO: Some logging - will handle later after base functionality is working
            }
            vTaskDelay(i2cs::I2C_DELAY_MS);
        }

        /**
         * @brief Writes a single byte to the specified register without any values.
         * @param reg The register address to write to.
         */
        void writeToReg(uint8_t reg) const {
            UniqueTimedMutex lock(_i2cMutex, std::defer_lock);
            if (lock.try_lock_for(i2cs::I2C_TIMEOUT_MS)) {
                _wire->beginTransmission(_i2c_addr);
                _wire->write(reg);
                _wire->endTransmission();
            } else {
                // TODO: Some logging - will handle later after base functionality is working
            }
            vTaskDelay(i2cs::I2C_DELAY_MS);
        }
        
        /**
         * @brief Writes a command to the sensor.
         * @param cmd The command to write, represented as a 16-bit unsigned integer.
         */
        void writeCommand(uint16_t cmd) const {
            std::array<uint8_t, 2> cmdBytes = {
                static_cast<uint8_t>(cmd >> 8), // High byte
                static_cast<uint8_t>(cmd & 0xFF) // Low byte
            };

            UniqueTimedMutex lock(_i2cMutex, std::defer_lock);
            if (lock.try_lock_for(i2cs::I2C_TIMEOUT_MS)) {
                _wire->beginTransmission(_i2c_addr);
                _wire->write(cmdBytes.data(), cmdBytes.size());
                _wire->endTransmission();
            } else {
                // TODO: Some logging - will handle later after base functionality is working
            }

            vTaskDelay(i2cs::I2C_DELAY_MS);
        }

        /**
         * Performs a CRC8 calculation on the supplied values.
         * @param data  Pointer to the data to use when calculating the CRC8.
         * @param len   The number of bytes in 'data'.
         * @return The computed CRC8 value.
         */
        static uint8_t crc8(const uint8_t *data, int len) {
            const uint8_t POLYNOMIAL(0x31);
            uint8_t crc(0xFF);

            for (int j = len; j; --j) {
                crc ^= *data++;
                for (int i = 8; i; --i) {
                    crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
                }
            }
            return crc;
        }

        /**
         * Find the mean of an array of numerical type T.
         * @param arr The array of type T with size N.
         * @return The mean value of the array.
         */
        template <typename T, size_t N>
        float mean(const std::array<T, N> &arr) const {
            long sum = std::accumulate(arr.begin(), arr.end(), 0L);
            return sum / (float)N;
        }

        /**
         * Find the mean of an vector of numerical type T.
         * @param arr The vector of type T with size N.
         * @return The mean value of the array.
         */
        template <typename T>
        float mean(const std::vector<T> &vec) const {
            long sum = std::accumulate(vec.begin(), vec.end(), 0L);
            return sum / (float)vec.size();
        }

        /**
         * Find the standard deviation of an array of numerical type T.
         * @tparam T The type of the elements in the array.
         * @tparam N The size of the array.
         * @param arr The array of type T with size N.
         * @param meanValue The mean value to use for the calculation. If NaN, it will be calculated from the array.
         * @return The standard deviation of the array.
         */
        template <typename T, size_t N>
        float stddev(const std::array<T, N> &arr, const float meanValue = NAN) const {
            float mval = std::isnan(meanValue) ? mean(arr) : meanValue;
            if (N == 0) return 0.0f;

            float sum = 0.0f;
            for (size_t i = 0; i < N; ++i) {
                sum += (arr[i] - mval) * (arr[i] - mval);
            }
            return sqrt(sum / (float)N);
        }

        /**
         * Find the standard deviation of a vector of numerical type T.
         * NOTE: If the meanValue is NaN, it will be calculated from the vector.
         * @tparam T The type of the elements in the vector.
         * @param vec The vector of type T.
         * @param meanValue The mean value to use for the calculation. If NaN, it will be calculated from the array.
         * @return The standard deviation of the vector.
         */
        template <typename T>
        float stddev(const std::vector<T> &vec, const float meanValue = NAN) const {
            float mval = std::isnan(meanValue) ? mean(vec) : meanValue;
            if (vec.empty()) return 0.0f;

            float sum = 0.0f;
            for (const T& num : vec) {
                sum += (num - mval) * (num - mval);
            }
            return sqrt(sum / (float)vec.size());
        }

        /**
         * Find the quartiles of an array of numerical type T.
         * NOTE: This function mutates the input array by sorting it.
         * @tparam T The type of the elements in the array.
         * @tparam N The size of the array.
         * @param arr The array of type T with size N.
         * @return An array containing the first quartile, median, and third quartile.
         */
        template <typename T, size_t N>
        std::array<float, 3> quartiles(std::array<T, N> &arr) const {
            std::sort(arr.begin(), arr.end());
            float q1 = arr[N / 4];
            float q3 = arr[(3 * N) / 4];
            float median = arr[N / 2];
            
            std::array<float, 3> quartiles = {q1, median, q3};
            return quartiles;
        }

        /**
         * Find the quartiles of an array of numerical type T.
         * NOTE: This function mutates the input array by sorting it.
         * @tparam T The type of the elements in the array.
         * @param vec The array of type T with size N.
         * @return An array containing the first quartile, median, and third quartile.
         */
        template <typename T>
        std::array<float, 3> quartiles(std::vector<T> &vec) const {
            std::sort(vec.begin(), vec.end());
            size_t N = vec.size();
            float q1 = vec[N / 4];
            float q3 = vec[(3 * N) / 4];
            float median = vec[N / 2];
            
            std::array<float, 3> quartiles = {q1, median, q3};
            return quartiles;
        }

        /**
         * Remove outliers from an array of numerical type T using the IQR method.
         * @param arr The array of type T.
         * @param q1 The first quartile.
         * @param q3 The third quartile.
         * @return A vector containing the filtered values without outliers.
         */
        template <typename T, size_t N>
        std::vector<T> removeOutliers(
            const std::array<T, N> &arr,
            float q1,
            float q3
        ) const {
            float iqr = q3 - q1;
            float lower_bound = q1 - 1.5 * iqr;
            float upper_bound = q3 + 1.5 * iqr;

            std::vector<T> filtered;
            filtered.reserve(N);

            std::copy_if(
                arr.begin(),
                arr.end(),
                std::back_inserter(filtered),
                [lower_bound, upper_bound](const T& value){
                    return value >= lower_bound && value <= upper_bound;
                }
            );

            return filtered;
        }

        /**
         * Remove outliers from a vector of numerical type T using the IQR method.
         * @param vec The vector of type T.
         * @param q1 The first quartile.
         * @param q3 The third quartile.
         * @return A vector containing the filtered values without outliers.
         */
        template <typename T>
        std::vector<T> removeOutliers(
            const std::vector<T> &vec,
            float q1,
            float q3
        ) const {
            float iqr = q3 - q1;
            float lower_bound = q1 - 1.5 * iqr;
            float upper_bound = q3 + 1.5 * iqr;

            std::vector<T> filtered;
            filtered.reserve(vec.size());

            std::copy_if(
                vec.begin(),
                vec.end(),
                std::back_inserter(filtered),
                [lower_bound, upper_bound](const T& value) {
                    return value >= lower_bound && value <= upper_bound;
                }
            );

            return filtered;
        }
};
