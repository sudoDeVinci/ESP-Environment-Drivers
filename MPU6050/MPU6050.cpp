#include "MPU6050/MPU6050.hpp"


/**
 * Reads the gyro data from the MPU6050 sensor.
 * This method reads raw gyro data from the sensor and applies sensitivity scaling and offsets.
 * @return An MPU_XYZ containing the scaled gyro values for each axis.
 */
MPU_XYZ MPU6050::readGyro() const{
    MPU_XYZ buffer = {0.0f, 0.0f, 0.0f};
    
    if (!this->_is_initialized) return buffer;

    this->writeToReg(mpu::MPU6050_REG::GYRO_XOUT_H);

    {
        UniqueTimedMutex lock(this->_i2cMutex, std::defer_lock);
        if (lock.try_lock_for(MPU6050::I2C_TIMEOUT_MS)) {
        
            _wire->requestFrom(this->_i2c_addr, (uint8_t)6);
            for (size_t i = 0; i < 3; ++i) {
                if (_wire->available() >= 2) {
                    int16_t rawValue = (_wire->read() << 8) | _wire->read();
                    buffer[i] = (rawValue / this-> sensitivity) - this->gyroOffsets[i];
                }
            }
        } else {
            // TODO: Some logging - will handle later after base functionality is working
        }
    }
    
    return buffer;
}

 /**
 * Reads multiple gyro samples and averages them.
 * This method collects a specified number of samples and computes the average for each axis.
 * @param samples The number of samples to average.
 * @return An averaged MPU_XYZ containing the mean values for each axis.
 */
MPU_XYZ MPU6050::readGyroSampled(uint16_t samples = MAX_SAMPLES) const {
    
    if (!this->_is_initialized) return {0.0f, 0.0f, 0.0f};

    #ifndef EPOXY_DUINO
        samples = std::clamp(samples, (uint16_t)30, MAX_SAMPLES);
    #else
        samples = clamp(samples, (uint16_t)30, MAX_SAMPLES);
    #endif
    
    std::array<std::vector<float>, 3> gyroSamples;
    
    for (size_t i = 0; i < 3; ++i) {
        gyroSamples[i].reserve(samples);
    }

    for (int i = 0; i < samples; ++i) {
        MPU_XYZ sample = readGyro();
        for (size_t j = 0; j < 3; ++j) {
            gyroSamples[j].push_back(sample[j]);
        }
    }

    MPU_XYZ filteredGyro = {0.0f, 0.0f, 0.0f};
    for (size_t i = 0; i < 3; ++i) {
        std::array<float, 3> quartiles = this->quartiles(gyroSamples[i]);
        float q1 = quartiles[0];
        float q3 = quartiles[2];
        std::vector<float> filteredValues = this->removeOutliers(gyroSamples[i], q1, q3);
        if (!filteredValues.empty()) filteredGyro[i] = mean(filteredValues);
    }

    return filteredGyro;
}

