#pragma once

#include "I2CSensor.hpp"
#include <unordered_map>

#define MPU6050_ADDR 0x68
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1_REG 0x6B


#ifdef EPOXY_DUINO
    /**
     * std::clamp doesn't exist before cpp17, so we may not have it.
     * the STL is also still lacking for AUnit right now - drop in replacement.
     * constexpr is here to match the STL version and speed up unit tests.
     */
    template<class T>
    constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
        return (v < lo) ? lo : (hi < v) ? hi : v;
    }
#endif

namespace mpu {
    /**
     * Enumeration for Digital Low Pass Filter (DLPF)
     * configuration registers addresses in hexadecimal.
     * These determine the cutoff frequency for the low pass filter.
     */
    enum DLPF_CFG {
        DLPF_256HZ = 0x00,
        DLPF_188HZ = 0x01,
        DLPF_98HZ  = 0x02,
        DLPF_42HZ  = 0x03,
        DLPF_20HZ  = 0x04,
        DLPF_10HZ  = 0x05,
        DLPF_5HZ   = 0x06
    };

    /**
     * Enumeration for LSB sensitivity settings.
     * These determine the sensitivity of the gyro readings,
     * mapped to the corresponding register addresses in hexadecimal.
     * The values are in degrees per second per LSB.
     */
    enum LSB_SENSITIVITY {
        LSB_131P0 = 0x00,
        LSB_65P5 = 0x08,
        LSB_32P8 = 0x10,
        LSB_16P4 = 0x18
    };

    /**
     * Enumeration for MPU6050 register addresses.
     * These are used to read and write data from/to the MPU6050 sensor.
     */
    enum MPU6050_REG {
        GYRO_LPF = 0x1A,
        GYRO_SENS = 0x1B,
        PWR_MGMT_1 = 0x6B,
        GYRO_XOUT_H = 0x43,
        GYRO_XOUT_L = 0x44,
        GYRO_YOUT_H = 0x45,
        GYRO_YOUT_L = 0x46,
        GYRO_ZOUT_H = 0x47,
        GYRO_ZOUT_L = 0x48
    };
};


using MPU_XYZ = std::array<float, 3>;

class MPU6050 : public I2CSensor {
    
    protected:
    
        mpu::DLPF_CFG filter;
        float sensitivity;
        MPU_XYZ lastGyro;
        MPU_XYZ gyroOffsets = {0.0f, 0.0f, 0.0f};

        bool deviceSpecificSetup() override {
            powerOn();
            setLPF(mpu::DLPF_CFG::DLPF_256HZ);
            setSensitivity(mpu::LSB_SENSITIVITY::LSB_65P5);
            return true;
        }


    public:

        static const std::unordered_map<mpu::LSB_SENSITIVITY, float> LSB_MAP;

        MPU6050(
            uint8_t bus_num,
            uint8_t sda,
            uint8_t scl,
            mpu::DLPF_CFG filter = mpu::DLPF_CFG::DLPF_256HZ,
            mpu::LSB_SENSITIVITY lsb = mpu::LSB_SENSITIVITY::LSB_65P5
        ): I2CSensor(MPU6050_ADDR, bus_num, sda, scl, 10000, 400000), filter(filter)  {
            this->sensitivity = LSB_MAP.at(lsb);
        }

        /**
         * This sensor doesn't update internally so we don't do anythig here.
         */
        bool update(void) override {
            return _is_initialized;
        }

        /**
         * Powers on the MPU6050 sensor.
         * This method writes to the PWR_MGMT_1 register to wake up the sensor.
         */
        void powerOn(void) const {
            writeToReg(mpu::MPU6050_REG::PWR_MGMT_1, 0x00);
        }

        /**
         * Sets the digital low pass filter (DLPF) configuration.
         * This method updates the filter setting and writes it to the sensor's register.
         * @param newFilter The new DLPF configuration to apply.
         */
        void setLPF(mpu::DLPF_CFG newFilter = mpu::DLPF_CFG::DLPF_10HZ) {
            this->filter = newFilter;
            writeToReg(mpu::MPU6050_REG::GYRO_LPF, static_cast<uint8_t>(newFilter));                         
        }

        /**
         * Sets the sensitivity of the gyro.
         * This method updates the sensitivity value and writes it to the sensor's register.
         * @param newSensitivity The new sensitivity setting to apply.
         */
        void setSensitivity(mpu::LSB_SENSITIVITY newSensitivity = mpu::LSB_SENSITIVITY::LSB_65P5) {
            this->sensitivity = LSB_MAP.at(newSensitivity);
            writeToReg(mpu::MPU6050_REG::GYRO_SENS, static_cast<uint8_t>(newSensitivity));
        }

        /**
         * Calibrates the gyro by averaging multiple samples.
         * This method collects a specified number of samples and computes the average offsets for each axis.
         * @param samples The number of samples to average for calibration.
         */
        void calibrateGyro(uint16_t samples = MAX_SAMPLES) {
            this -> gyroOffsets = {0.0f, 0.0f, 0.0f};
            MPU_XYZ offsets = this->readGyroSampled(samples);
            this->gyroOffsets = offsets;
        }

        /**
         * Reads the gyro data from the MPU6050 sensor.
         * This method reads raw gyro data from the sensor and applies sensitivity scaling and offsets.
         * @return An MPU_XYZ containing the scaled gyro values for each axis.
         */
        MPU_XYZ readGyro(void) const;

        /**
         * Reads multiple gyro samples and averages them.
         * This method collects a specified number of samples and computes the average for each axis.
         * @param samples The number of samples to average.
         * @return An averaged MPU_XYZ containing the mean values for each axis.
         */
        MPU_XYZ readGyroSampled(uint16_t samples = MAX_SAMPLES) const;
};

/**
 * Mapping of LSB sensitivity values to their corresponding sensitivity factors.
 * This is used to convert raw gyro readings into meaningful values.
 */
const std::unordered_map<mpu::LSB_SENSITIVITY, float> MPU6050::LSB_MAP = {
    {mpu::LSB_SENSITIVITY::LSB_131P0, 131.00f},
    {mpu::LSB_SENSITIVITY::LSB_65P5, 65.50f},
    {mpu::LSB_SENSITIVITY::LSB_32P8, 32.80f},
    {mpu::LSB_SENSITIVITY::LSB_16P4, 16.40f}
};