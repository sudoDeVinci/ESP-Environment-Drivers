#include "SHT3X/SHT31D.hpp"

 /**
 * Reads the current status of the SHT31 sensor.
 * This method sends a command to read the status and returns the status code.
 * @return The status code as a 16-bit unsigned integer.
 */
uint16_t SHT31::readStatus(void) const {
    writeCommand(sht::CMDS::READSTATUS);
    uint8_t data[3] = {0, 0, 0};

    UniqueTimedMutex lock(this->_i2cMutex, std::defer_lock);
    if (lock.try_lock_for(SHT31::I2C_TIMEOUT_MS)) {
        if (this->_wire->requestFrom(this->_i2c_addr, (uint8_t)3) == 3) {
            data[0] = this->_wire->read();
            data[1] = this->_wire->read();
            data[2] = this->_wire->read();

            if (data[2] == I2CSensor::crc8(data, 2)) {
                return (uint16_t)data[0] << 8 | data[1];
            }
        }
    }
    return 0xFFFF;
}


/**
 * Reads the temperature and humidity from the SHT31 sensor.
 * This method sends a command to read the temperature and humidity data,
 * processes the data, and updates the internal temperature and humidity values.
 * @return True if the read was successful, false otherwise.
 */
bool SHT31::update(void) override {
    writeCommand(sht::MEASUREMENT_MODE::MEDREP);
    uint8_t readbuffer[6];
    vTaskDelay(20 / portTICK_PERIOD_MS);

    {
        UniqueTimedMutex lock(this->_i2cMutex, std::defer_lock);
        if (lock.try_lock_for(SHT31::I2C_TIMEOUT_MS)) {
            
            if (this->_wire->requestFrom(this->_i2c_addr, (uint8_t)6) != 6) return false;

            for (size_t i = 0; i < 6; ++i) {
                if (_wire->available()) readbuffer[i] = this->_wire->read();
            }
        } else {
            return false; // Timeout trying to grab the lock
        }
    }

    if (readbuffer[2] != I2CSensor::crc8(readbuffer, 2) || readbuffer[5] != I2CSensor::crc8(readbuffer + 3, 2)) {
        return false; // CRC check failed
    }

    int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
    stemp = ((4375 * stemp) >> 14) - 4500;
    this->temperature = (float)stemp / 100.0f;

    uint32_t shum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
    shum = (625 * shum) >> 12;
    this->humidity = (float)shum / 100.0f;

    return true;
}


